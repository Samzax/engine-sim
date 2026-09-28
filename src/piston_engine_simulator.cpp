#include "../include/piston_engine_simulator.h"
#include "../include/simulation_profile.h"
#include "../include/gpu_coupled.h"
#include "../include/sim_thread_pool.h"

#include "../include/constants.h"
#include "../include/units.h"

#include <cmath>
#include <assert.h>
#include <chrono>
#include <set>
#include <unordered_set>
#include <stdexcept>
#include <string>

PistonEngineSimulator::PistonEngineSimulator() {
    m_engine = nullptr;
    m_transmission = nullptr;
    m_vehicle = nullptr;
    m_delayFilters = nullptr;

    m_crankConstraints = nullptr;
    m_cylinderWallConstraints = nullptr;
    m_linkConstraints = nullptr;
    m_crankshaftFrictionConstraints = nullptr;
    m_crankshaftLinks = nullptr;

    m_exhaustFlowStagingBuffer = nullptr;

    m_derivativeFilter.m_dt = 1.0;
    m_fluidSimulationSteps = 8;
}

PistonEngineSimulator::~PistonEngineSimulator() {
    destroy();
}

void PistonEngineSimulator::loadSimulation(Engine *engine, Vehicle *vehicle, Transmission *transmission) {
    Simulator::loadSimulation(engine, vehicle, transmission);

    m_engine = engine;
    m_vehicle = vehicle;
    m_transmission = transmission;

    m_pipes.clear();
    m_coupledGpuSteps=m_coupledGpuFallbacks=0;
    m_portsRegionMerged=false;
    const bool variableGas = engine->variableGasProperties();
    for (int i = 0; i < engine->getIntakeCount(); ++i)
        engine->getIntake(i)->configureGas(variableGas, engine->getFuel()->getMolecularMass(), engine->getFuel()->getMolecularAfr());
    for (int i = 0; i < engine->getExhaustSystemCount(); ++i)
        engine->getExhaustSystem(i)->configureGas(variableGas);
    for (int i = 0; i < engine->getCylinderCount(); ++i) {
        auto *chamber = engine->getChamber(i);
        chamber->configureGas(variableGas,engine->getFuel()->getMolecularMass(),engine->getFuel()->getMolecularAfr());
        if (variableGas) {
            if(chamber->intakePipe()->active()) m_pipes.push_back(chamber->intakePipe());
            if(chamber->exhaustPipe()->active()) m_pipes.push_back(chamber->exhaustPipe());
        }
    }
    m_pipeWeights.clear();
    m_pipeWeights.reserve(m_pipes.size());
    for (GasPipe *pipe : m_pipes) m_pipeWeights.push_back(pipe->count());
    // Reservoir chains and cylinder port stages may share one barrier when
    // every pipe has at least two cells: chains then touch the opposite pipe
    // end from the cylinder stages (single-cell pipes would alias them).
    m_portsRegionMerged=true;
    for (int i = 0; i < engine->getCylinderCount(); ++i) {
        auto *chamber = engine->getChamber(i);
        if (chamber->intakePipe()->count() < 2 || chamber->exhaustPipe()->count() < 2) {
            m_portsRegionMerged = false;
            break;
        }
    }

    const int crankCount = m_engine->getCrankshaftCount();
    const int cylinderCount = m_engine->getCylinderCount();
    const int linkCount = cylinderCount * 2;

    if (crankCount <= 0) return;

    m_crankConstraints = new atg_scs::FixedPositionConstraint[crankCount];
    m_cylinderWallConstraints = new atg_scs::LineConstraint[cylinderCount];
    m_linkConstraints = new atg_scs::LinkConstraint[linkCount];
    m_crankshaftFrictionConstraints = new atg_scs::RotationFrictionConstraint[crankCount];
    m_crankshaftLinks = new atg_scs::ClutchConstraint[crankCount - 1];
    m_delayFilters = new DelayFilter[cylinderCount];

    const double ks = 5000;
    const double kd = 10;

    for (int i = 0; i < crankCount; ++i) {
        Crankshaft *outputShaft = m_engine->getCrankshaft(0);
        Crankshaft *crankshaft = m_engine->getCrankshaft(i);

        m_crankConstraints[i].setBody(&crankshaft->m_body);
        m_crankConstraints[i].setWorldPosition(
            crankshaft->getPosX(),
            crankshaft->getPosY());
        m_crankConstraints[i].setLocalPosition(0.0, 0.0);
        m_crankConstraints[i].m_kd = kd;
        m_crankConstraints[i].m_ks = ks;

        crankshaft->m_body.p_x = crankshaft->getPosX();
        crankshaft->m_body.p_y = crankshaft->getPosY();
        crankshaft->m_body.theta = 0;
        crankshaft->m_body.m =
            crankshaft->getMass() + crankshaft->getFlywheelMass();
        crankshaft->m_body.I = crankshaft->getMomentOfInertia();

        m_crankshaftFrictionConstraints[i].m_minTorque = -crankshaft->getFrictionTorque();
        m_crankshaftFrictionConstraints[i].m_maxTorque = crankshaft->getFrictionTorque();
        m_crankshaftFrictionConstraints[i].setBody(&m_engine->getCrankshaft(i)->m_body);

        m_system->addRigidBody(&m_engine->getCrankshaft(i)->m_body);
        m_system->addConstraint(&m_crankConstraints[i]);
        m_system->addConstraint(&m_crankshaftFrictionConstraints[i]);

        if (crankshaft != outputShaft) {
            atg_scs::ClutchConstraint *crankLink = &m_crankshaftLinks[i - 1];
            crankLink->setBody1(&outputShaft->m_body);
            crankLink->setBody2(&crankshaft->m_body);

            m_system->addConstraint(crankLink);
        }
    }

    m_transmission->addToSystem(m_system, &m_vehicleMass, m_vehicle, m_engine);
    m_vehicle->addToSystem(m_system, &m_vehicleMass);
    m_engine->setVehicleSpeed(0.0);

    m_vehicleDrag.initialize(&m_vehicleMass, m_vehicle);
    m_system->addConstraint(&m_vehicleDrag);

    m_vehicleMass.reset();
    m_vehicleMass.m = 1.0;
    m_vehicleMass.I = 1.0;
    m_system->addRigidBody(&m_vehicleMass);

    for (int i = 0; i < cylinderCount; ++i) {
        Piston *piston = m_engine->getPiston(i);
        ConnectingRod *connectingRod = piston->getRod();

        CylinderBank *bank = piston->getCylinderBank();
        const double dx = std::cos(bank->getAngle() + constants::pi / 2);
        const double dy = std::sin(bank->getAngle() + constants::pi / 2);

        m_cylinderWallConstraints[i].setBody(&piston->m_body);
        m_cylinderWallConstraints[i].m_dx = dx;
        m_cylinderWallConstraints[i].m_dy = dy;
        m_cylinderWallConstraints[i].m_local_x = 0.0;
        m_cylinderWallConstraints[i].m_local_y = piston->getWristPinLocation();
        m_cylinderWallConstraints[i].m_p0_x = bank->getX();
        m_cylinderWallConstraints[i].m_p0_y = bank->getY();
        m_cylinderWallConstraints[i].m_ks = ks;
        m_cylinderWallConstraints[i].m_kd = kd;

        piston->setCylinderConstraint(&m_cylinderWallConstraints[i]);

        m_linkConstraints[i * 2 + 0].setBody1(&connectingRod->m_body);
        m_linkConstraints[i * 2 + 0].setBody2(&piston->m_body);
        m_linkConstraints[i * 2 + 0]
            .setLocalPosition1(0.0, connectingRod->getLittleEndLocal());
        m_linkConstraints[i * 2 + 0].setLocalPosition2(0.0, piston->getWristPinLocation());
        m_linkConstraints[i * 2 + 0].m_ks = ks;
        m_linkConstraints[i * 2 + 0].m_kd = kd;

        double journal_x = 0.0, journal_y = 0.0;
        if (connectingRod->getMasterRod() == nullptr) {
            Crankshaft *crankshaft = connectingRod->getCrankshaft();
            crankshaft->getRodJournalPositionLocal(
                connectingRod->getJournal(),
                &journal_x,
                &journal_y);
            m_linkConstraints[i * 2 + 1].setBody2(&crankshaft->m_body);
        }
        else {
            connectingRod->getMasterRod()->getRodJournalPositionLocal(
                connectingRod->getJournal(),
                &journal_x,
                &journal_y);
            m_linkConstraints[i * 2 + 1].setBody2(&connectingRod->getMasterRod()->m_body);
        }

        m_linkConstraints[i * 2 + 1].setBody1(&connectingRod->m_body);
        m_linkConstraints[i * 2 + 1]
            .setLocalPosition1(0.0, connectingRod->getBigEndLocal());
        m_linkConstraints[i * 2 + 1]
            .setLocalPosition2(journal_x, journal_y);
        m_linkConstraints[i * 2 + 1].m_ks = ks;
        m_linkConstraints[i * 2 + 1].m_kd = kd;

        piston->m_body.m = piston->getMass();
        piston->m_body.I = 1.0;

        connectingRod->m_body.m = connectingRod->getMass();
        connectingRod->m_body.I = connectingRod->getMomentOfInertia();

        m_system->addRigidBody(&piston->m_body);
        m_system->addRigidBody(&connectingRod->m_body);
        m_system->addConstraint(&m_linkConstraints[i * 2 + 0]);
        m_system->addConstraint(&m_linkConstraints[i * 2 + 1]);
        m_system->addConstraint(&m_cylinderWallConstraints[i]);
        m_system->addForceGenerator(m_engine->getChamber(i));
    }

    m_dyno.connectCrankshaft(m_engine->getOutputCrankshaft());
    m_system->addConstraint(&m_dyno);

    m_starterMotor.connectCrankshaft(m_engine->getOutputCrankshaft());
    m_starterMotor.m_maxTorque = m_engine->getStarterTorque();
    m_starterMotor.m_rotationSpeed = -m_engine->getStarterSpeed();
    m_system->addConstraint(&m_starterMotor);

    placeAndInitialize();
    initializeSynthesizer();
}

double PistonEngineSimulator::getAverageOutputSignal() const {
    double sum = 0.0;
    for (int i = 0; i < m_engine->getExhaustSystemCount(); ++i) {
        sum += m_engine->getExhaustSystem(i)->getSystem()->pressure();
    }

    return sum / m_engine->getExhaustSystemCount();
}

void PistonEngineSimulator::placeAndInitialize() {
    const int cylinderCount = m_engine->getCylinderCount();
    std::unordered_set<ConnectingRod *> placed;
    while (placed.size() < static_cast<size_t>(cylinderCount)) {
        const size_t previousCount = placed.size();
        for (int i = 0; i < cylinderCount; ++i) {
            ConnectingRod *rod = m_engine->getConnectingRod(i);
            if (placed.count(rod) != 0) continue;
            if (rod->getMasterRod() == nullptr || placed.count(rod->getMasterRod()) != 0) {
                placeCylinder(i);
                placed.insert(rod);
            }
        }
        if (placed.size() == previousCount)
            throw std::invalid_argument("Connecting rod masters must form an acyclic assembly within the engine");
    }

    for (int i = 0; i < cylinderCount; ++i) {
        m_engine->getChamber(i)->m_system.initialize(
            units::pressure(1.0, units::atm),
            m_engine->getChamber(i)->getVolume(),
            units::celcius(25.0),
            m_engine->variableGasProperties() ? GasSystem::Mix{0,0.79,0.21} : GasSystem::Mix{}
        );

        Piston *piston = m_engine->getChamber(i)->getPiston();
        CylinderHead *head = m_engine->getChamber(i)->getCylinderHead();
        ExhaustSystem *exhaust = head->getExhaustSystem(piston->getCylinderIndex());
        const double exhaustLength =
            head->getHeaderPrimaryLength(piston->getCylinderIndex())
            + exhaust->getLength();
        const double speedOfSound = 343.0 * units::m / units::sec;
        const double delay = exhaustLength / speedOfSound;
        m_delayFilters[i].initialize(delay, getSimulationFrequency());
    }

    m_engine->getIgnitionModule()->reset();

    m_exhaustFlowStagingBuffer = new double[m_engine->getExhaustSystemCount()];
}

void PistonEngineSimulator::placeCylinder(int i) {
    ConnectingRod *rod = m_engine->getConnectingRod(i);
    Piston *piston = m_engine->getPiston(i);
    CylinderBank *bank = piston->getCylinderBank();

    const auto invalidGeometry = [i]() {
        return std::invalid_argument("Cylinder " + std::to_string(i + 1)
            + " has invalid initial rod geometry: the rod must reach the cylinder axis above the bank origin");
    };
    if (!std::isfinite(rod->getLength()) || rod->getLength() <= 0)
        throw invalidGeometry();

    double p_x, p_y;
    if (rod->getMasterRod() != nullptr) {
        rod->getMasterRod()->getRodJournalPositionGlobal(rod->getJournal(), &p_x, &p_y);
    }
    else {
        rod->getCrankshaft()->getRodJournalPositionGlobal(rod->getJournal(), &p_x, &p_y);
    }

    // (bank->m_x + bank->m_dx * s - p_x)^2 + (bank->m_y + bank->m_dy * s - p_y)^2 = (rod->m_length)^2
    const double a = bank->getDx() * bank->getDx() + bank->getDy() * bank->getDy();
    const double b = -2 * bank->getDx() * (p_x - bank->getX()) - 2 * bank->getDy() * (p_y - bank->getY());
    const double c =
        (p_x - bank->getX()) * (p_x - bank->getX())
        + (p_y - bank->getY()) * (p_y - bank->getY())
        - rod->getLength() * rod->getLength();

    const double det = b * b - 4 * a * c;
    if (!std::isfinite(a) || a <= 0 || !std::isfinite(det) || det < 0)
        throw invalidGeometry();

    const double sqrt_det = std::sqrt(det);
    const double s0 = (-b + sqrt_det) / (2 * a);
    const double s1 = (-b - sqrt_det) / (2 * a);

    const double s = std::max(s0, s1);
    if (!std::isfinite(s) || s < 0) throw invalidGeometry();

    const double e_x = s * bank->getDx() + bank->getX();
    const double e_y = s * bank->getDy() + bank->getY();

    const double theta = ((e_y - p_y) > 0)
        ? std::acos((e_x - p_x) / rod->getLength())
        : 2 * constants::pi - std::acos((e_x - p_x) / rod->getLength());
    rod->m_body.theta = theta - constants::pi / 2;

    double cl_x, cl_y;
    rod->m_body.localToWorld(0, rod->getBigEndLocal(), &cl_x, &cl_y);
    rod->m_body.p_x += p_x - cl_x;
    rod->m_body.p_y += p_y - cl_y;

    piston->m_body.p_x = e_x;
    piston->m_body.p_y = e_y;
    piston->m_body.theta = bank->getAngle() + constants::pi;
    double pin_x, pin_y;
    piston->m_body.localToWorld(0, piston->getWristPinLocation(), &pin_x, &pin_y);
    piston->m_body.p_x += e_x - pin_x;
    piston->m_body.p_y += e_y - pin_y;
}

void PistonEngineSimulator::simulateStep_() {
    ENGINE_SIM_PROFILE_SCOPE(Fluid);
    const double timestep = getTimestep();
    IgnitionModule *im = m_engine->getIgnitionModule();
    im->update(timestep);

    const int cylinderCount = m_engine->getCylinderCount();
    bool separatedPorts=!m_pipes.empty();
    for (int i = 0; i < cylinderCount; ++i) {
        if (im->getIgnitionEvent(i)) {
            m_engine->getChamber(i)->ignite();
        }
    }

    // Chambers write per-chamber state and read shared functions const-only,
    // so updates and per-step flow resets run concurrently. Ignition stays
    // serial above: it consumes rand() in cylinder order, and reordering those
    // draws would change results.
    sim_pool::parallelFor(cylinderCount,[&](int i) {
        m_engine->getChamber(i)->update(timestep);
        m_engine->getChamber(i)->resetLastTimestepExhaustFlow();
        m_engine->getChamber(i)->resetLastTimestepIntakeFlow();
    });
    for (int i = 0; i < cylinderCount; ++i) {
        separatedPorts=separatedPorts && m_engine->getChamber(i)->supportsSeparatedPorts();
    }

    const int exhaustSystemCount = m_engine->getExhaustSystemCount();
    const int intakeCount = m_engine->getIntakeCount();
    const double fluidTimestep = timestep / m_fluidSimulationSteps;
    bool coupled=false;
    if(gpu_coupled::requested()) {
        coupled=advanceCoupledFluids(timestep);
        if(coupled) ++m_coupledGpuSteps; else ++m_coupledGpuFallbacks;
    }
    for (int i = 0; !coupled && i < m_fluidSimulationSteps; ++i) {
        if(m_pipes.empty()) {
            {
                ENGINE_SIM_PROFILE_SCOPE(Reservoirs);
                // Every exhaust system and intake owns its plenum/collector and
                // atmosphere members, so the units are mutually disjoint.
                sim_pool::parallelFor(exhaustSystemCount,[&](int j) {
                    m_engine->getExhaustSystem(j)->process(fluidTimestep);
                });
                sim_pool::parallelFor(intakeCount,[&](int j) {
                    Intake *intake=m_engine->getIntake(j);
                    intake->process(fluidTimestep);
                    intake->m_flowRate+=intake->m_flow;
                });
            }
            for (int j=0;j<cylinderCount;++j) m_engine->getChamber(j)->flow(fluidTimestep);
        } else {
            // Reservoirs run once per fluid step and share the first substep's
            // CFL barrier: reservoir systems and pipe cells are disjoint, and
            // reservoir outputs are only read by the port phase that follows.
            bool reservoirsPending=true;
            // Common CFL substeps let independent pipe interiors run in one batch.
            // Port exchanges retain cylinder order for shared plenums/collectors.
            double remaining=fluidTimestep;
            int steps=0;
            while(remaining>0) {
                if(++steps>10000) throw std::runtime_error("Pipe coupling exceeded substep limit");
                double h=remaining;
                {
                    ENGINE_SIM_PROFILE_SCOPE(Cfl);
                    // Each pipe's stable step depends only on its own cells;
                    // compute them concurrently (together with any pending
                    // reservoir units), then fold the minimum in index order
                    // so the reduction matches the serial loop.
                    const int pipeCount=static_cast<int>(m_pipes.size());
                    if(m_cflScratch.size()!=static_cast<size_t>(pipeCount)) m_cflScratch.resize(pipeCount);
                    if(reservoirsPending) {
                        reservoirsPending=false;
                        const int reservoirCount=exhaustSystemCount+intakeCount;
                        sim_pool::parallelFor(reservoirCount+pipeCount,sim_pool::Split::Pull,[&](int unit) {
                            if(unit<exhaustSystemCount) {
                                m_engine->getExhaustSystem(unit)->process(fluidTimestep);
                            } else if(unit<reservoirCount) {
                                Intake *intake=m_engine->getIntake(unit-exhaustSystemCount);
                                intake->process(fluidTimestep);
                                intake->m_flowRate+=intake->m_flow;
                            } else {
                                const int j=unit-reservoirCount;
                                m_cflScratch[j]=m_pipes[j]->stableTimestep();
                            }
                        });
                    } else {
                        sim_pool::parallelFor(pipeCount,m_pipeWeights.data(),[&](int j) {
                            m_cflScratch[j]=m_pipes[j]->stableTimestep();
                        });
                    }
                    // Fold the minimum in index order so the reduction matches
                    // the serial loop.
                    for(int j=0;j<pipeCount;++j) h=(std::min)(h,m_cflScratch[j]);
                }
                {
                    ENGINE_SIM_PROFILE_SCOPE(Ports);
                    if(separatedPorts) {
                        if(m_portsRegionMerged) {
                            // One barrier for the whole port phase: reservoir
                            // chains keep their cylinder order but touch
                            // opposite pipe ends from the cylinder stages,
                            // and the two chains are disjoint, so all units
                            // commute when pipe cells >= 2. Pull keeps the two
                            // chains off one thread's static slice.
                            sim_pool::parallelFor(2+cylinderCount,sim_pool::Split::Pull,[&](int unit) {
                                if(unit<2) {
                                    for(int j=0;j<cylinderCount;++j) {
                                        if(unit==0) m_engine->getChamber(j)->flowIntakeReservoir(h);
                                        else m_engine->getChamber(j)->flowExhaustReservoir(h);
                                    }
                                } else {
                                    m_engine->getChamber(unit-2)->flowCylinderPorts(h);
                                }
                            });
                        } else {
                            // Single-cell pipes alias chain and cylinder ends;
                            // keep the phases separate (reservoir order first).
                            sim_pool::parallelFor(2,[&](int chain) {
                                for(int j=0;j<cylinderCount;++j) {
                                    if(chain==0) m_engine->getChamber(j)->flowIntakeReservoir(h);
                                    else m_engine->getChamber(j)->flowExhaustReservoir(h);
                                }
                            });
                            sim_pool::parallelFor(cylinderCount,[&](int j) {
                                m_engine->getChamber(j)->flowCylinderPorts(h);
                            });
                        }
                    } else {
                        for(int j=0;j<cylinderCount;++j) m_engine->getChamber(j)->flowPorts(h);
                    }
                }
                if(gpu_pipe::enabled()) {
                    GasPipe::advanceBatch(m_pipes.data(),static_cast<int>(m_pipes.size()),h);
                } else {
                    // Pipe interiors are independent (one owns its cells), so
                    // the serial batch becomes one index per pipe; cell counts
                    // weight the static slices (pipes differ ~2x in size).
                    ENGINE_SIM_PROFILE_SCOPE(Pipes);
                    sim_pool::parallelFor(static_cast<int>(m_pipes.size()),m_pipeWeights.data(),[&](int j) {
                        m_pipes[j]->advance(h);
                    });
                }
                remaining-=h;
            }
        }
    }

    // Runner aggregates feed audio/readouts once per mechanical step. Port
    // exchanges use the actual cells, so intermediate snapshots are redundant.
    // Each chamber aggregates into its own two pipes and runner systems, so the
    // per-chamber units are disjoint.
    if(!m_pipes.empty())
        sim_pool::parallelFor(cylinderCount,[&](int j) {
            m_engine->getChamber(j)->aggregatePipes();
        });
    im->resetIgnitionEvents();
}

double PistonEngineSimulator::getTotalExhaustFlow() const {
    double totalFlow = 0.0;
    for (int i = 0; i < m_engine->getCylinderCount(); ++i) {
        totalFlow += m_engine->getChamber(i)->getLastTimestepExhaustFlow();
    }

    return totalFlow;
}

bool PistonEngineSimulator::advanceCoupledFluids(double timestep) {
    if(!m_engine || !gpu_pipe::enabled()) return false;
    const int count=m_engine->getCylinderCount();
    if(count<1 || m_pipes.size()!=static_cast<size_t>(count*2)) return false;
    for(int i=0;i<count;++i) {
        auto *chamber=m_engine->getChamber(i);
        if(!chamber->supportsSeparatedPorts() || !chamber->m_system.variableProperties()
            || m_pipes[2*i]!=chamber->intakePipe() || m_pipes[2*i+1]!=chamber->exhaustPipe()) return false;
    }
    for(auto *pipe:m_pipes) for(int k=0;k<pipe->count();++k)
        if(!pipe->cell(k).variableProperties()) return false;
    thread_local gpu_coupled::Batch batch;
    batch.intakes.resize(m_engine->getIntakeCount()); batch.exhausts.resize(m_engine->getExhaustSystemCount());
    for(size_t i=0;i<batch.intakes.size();++i) {
        auto *intake=m_engine->getIntake(static_cast<int>(i));
        if(!intake->m_system.variableProperties()) return false;
        batch.intakes[i]={intake->flowState(),intake->flowParameters()};
    }
    for(size_t i=0;i<batch.exhausts.size();++i) {
        auto *exhaust=m_engine->getExhaustSystem(static_cast<int>(i));
        if(!exhaust->getSystem()->variableProperties()) return false;
        batch.exhausts[i]={exhaust->flowState(),exhaust->flowParameters()};
    }
    batch.cylinders.resize(count); batch.pipes.resize(m_pipes.size());
    int cells=0;
    for(size_t i=0;i<m_pipes.size();++i) {batch.pipes[i].firstCell=cells; cells+=m_pipes[i]->count();}
    batch.cells.resize(cells);
    for(size_t i=0;i<m_pipes.size();++i)
        m_pipes[i]->exportCoupled(batch.pipes[i].solver,batch.cells.data()+batch.pipes[i].firstCell);
    for(int i=0;i<count;++i) {
        auto *chamber=m_engine->getChamber(i); auto *head=chamber->getCylinderHead();
        auto &out=batch.cylinders[i];
        out.state=chamber->cylinderFlowState(); out.parameters=chamber->cylinderFlowParameters();
        out.intakePipe=2*i; out.exhaustPipe=2*i+1;
        out.manifoldK=chamber->manifoldCouplingK(); out.collectorK=chamber->collectorCouplingK();
        auto *intake=head->getIntake(chamber->getPiston()->getCylinderIndex());
        auto *exhaust=head->getExhaustSystem(chamber->getPiston()->getCylinderIndex());
        out.intake=-1; out.exhaust=-1;
        for(size_t j=0;j<batch.intakes.size();++j) if(m_engine->getIntake(static_cast<int>(j))==intake) out.intake=static_cast<int>(j);
        for(size_t j=0;j<batch.exhausts.size();++j) if(m_engine->getExhaustSystem(static_cast<int>(j))==exhaust) out.exhaust=static_cast<int>(j);
        if(out.intake<0 || out.exhaust<0) return false;
    }
    if(!gpu_coupled::advance(batch,timestep,m_fluidSimulationSteps)) return false;
    for(size_t i=0;i<m_pipes.size();++i)
        m_pipes[i]->importCoupled(batch.cells.data()+batch.pipes[i].firstCell,batch.pipes[i].solver.stableTimestep);
    for(int i=0;i<count;++i) m_engine->getChamber(i)->applyCylinderFlowState(batch.cylinders[i].state);
    for(size_t i=0;i<batch.intakes.size();++i) m_engine->getIntake(static_cast<int>(i))->applyFlowState(batch.intakes[i].state);
    for(size_t i=0;i<batch.exhausts.size();++i) m_engine->getExhaustSystem(static_cast<int>(i))->applyFlowState(batch.exhausts[i].state);
    return true;
}

void PistonEngineSimulator::endFrame() {
    Simulator::endFrame();

    if (m_engine == nullptr) {
        return;
    }

    const double frameTimestep = simulationSteps() * getTimestep();
    // No simulated time elapsed, so retain the last measured flow for gauges.
    // startFrame resets the accumulator when the next physics step is scheduled.
    if (frameTimestep <= 0) return;
    for (int i = 0; i < m_engine->getIntakeCount(); ++i) {
        m_engine->getIntake(i)->m_flowRate /= frameTimestep;
    }
}

void PistonEngineSimulator::destroy() {
    m_pipes.clear();
    m_coupledGpuSteps=m_coupledGpuFallbacks=0;
    endAudioRenderingThread();
    if (m_system != nullptr) m_system->reset();

    if (m_crankConstraints != nullptr) delete[] m_crankConstraints;
    if (m_cylinderWallConstraints != nullptr) delete[] m_cylinderWallConstraints;
    if (m_linkConstraints != nullptr) delete[] m_linkConstraints;
    if (m_crankshaftFrictionConstraints != nullptr) delete[] m_crankshaftFrictionConstraints;
    delete[] m_crankshaftLinks;
    m_crankshaftLinks = nullptr;
    if (m_exhaustFlowStagingBuffer != nullptr) delete[] m_exhaustFlowStagingBuffer;
    if (m_system != nullptr) delete m_system;
    if (m_delayFilters != nullptr) delete[] m_delayFilters;

    m_crankConstraints = nullptr;
    m_cylinderWallConstraints = nullptr;
    m_linkConstraints = nullptr;
    m_crankshaftFrictionConstraints = nullptr;
    m_exhaustFlowStagingBuffer = nullptr;
    m_system = nullptr;

    m_vehicle = nullptr;
    m_transmission = nullptr;
    m_engine = nullptr;
    m_delayFilters = nullptr;
    Simulator::destroy();
}

void PistonEngineSimulator::setSimulationFrequency(int frequency) {
    Simulator::setSimulationFrequency(frequency);
    if (m_delayFilters != nullptr && m_engine != nullptr) {
        for (int i = 0; i < m_engine->getCylinderCount(); ++i) {
            m_delayFilters[i].setSampleRate(frequency);
        }
    }
}

void PistonEngineSimulator::writeToSynthesizer() {
    const int exhaustSystemCount = m_engine->getExhaustSystemCount();
    for (int i = 0; i < exhaustSystemCount; ++i) {
        m_exhaustFlowStagingBuffer[i] = 0;
    }

    const double attenuation = std::min(std::abs(filteredEngineSpeed()), 40.0) / 40.0;
    const double attenuation_3 = attenuation * attenuation * attenuation;

    const double timestep = getTimestep();
    const int cylinderCount = m_engine->getCylinderCount();
    for (int i = 0; i < cylinderCount; ++i) {
        Piston *piston = m_engine->getPiston(i);
        CylinderBank *bank = piston->getCylinderBank();
        CylinderHead *head = m_engine->getHead(bank->getIndex());
        ExhaustSystem *exhaust = head->getExhaustSystem(piston->getCylinderIndex());
        CombustionChamber *chamber = m_engine->getChamber(i);

        const double exhaustLength =
            head->getHeaderPrimaryLength(piston->getCylinderIndex())
            + exhaust->getLength();

        double exhaustFlow =
            attenuation_3 * 1600 * (
                1.0 * (chamber->m_exhaustRunnerAndPrimary.pressure() - units::pressure(1.0, units::atm))
                + 0.1 * chamber->m_exhaustRunnerAndPrimary.dynamicPressure(1.0, 0.0)
                + 0.1 * chamber->m_exhaustRunnerAndPrimary.dynamicPressure(-1.0, 0.0));

        const double delayedExhaustPulse =
            m_delayFilters[i].fast_f(exhaustFlow);

        ExhaustSystem *exhaustSystem = head->getExhaustSystem(piston->getCylinderIndex());
        m_exhaustFlowStagingBuffer[exhaustSystem->getIndex()] +=
            head->getSoundAttenuation(piston->getCylinderIndex())
            * (exhaustSystem->getAudioVolume() * delayedExhaustPulse / cylinderCount)
            * (1 / (exhaustLength * exhaustLength));
    }

    synthesizer().writeInput(m_exhaustFlowStagingBuffer);
}
