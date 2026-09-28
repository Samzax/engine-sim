#include <gtest/gtest.h>
#include "../scripting/include/compiler.h"
#include "../include/simulator.h"
#include "../include/piston_engine_simulator.h"
#include "../include/gpu_chamber.h"
#include "../include/gpu_pipe.h"
#include "../include/constants.h"
#include "../include/vehicle_drag_constraint.h"
#include "../include/vtec_valvetrain.h"
#include <memory>
#include <limits>
#include <stdexcept>
#include <fstream>
#include <iterator>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <cmath>

namespace {
struct EngineOwner {
    es_script::Compiler::Output output;
    ~EngineOwner() {
        if (output.engine) { output.engine->destroy(); delete output.engine; }
        delete output.vehicle;
        delete output.transmission;
    }
};
}

TEST(SimulatorRegression, VtecRespectsMinimumVehicleSpeed) {
    struct RunningEngine : Engine {
        double getManifoldPressure() const override { return units::pressure(1, units::atm); }
        double getSpeed() const override { return units::rpm(7000); }
        double getThrottle() const override { return 0.0; }
    } engine;
    Camshaft normal, high;
    VtecValvetrain valvetrain;
    VtecValvetrain::Parameters parameters{};
    parameters.engine = &engine;
    parameters.intakeCamshaft = parameters.exhaustCamshaft = &normal;
    parameters.vtecIntakeCamshaft = parameters.vtexExhaustCamshaft = &high;
    parameters.minSpeed = 10 * units::mile / units::hour;
    valvetrain.initialize(parameters);
    EXPECT_EQ(valvetrain.getActiveIntakeCamshaft(), &normal);
    engine.setVehicleSpeed(11 * units::mile / units::hour);
    EXPECT_EQ(valvetrain.getActiveIntakeCamshaft(), &high);
    engine.setVehicleSpeed(9 * units::mile / units::hour);
    EXPECT_EQ(valvetrain.getActiveExhaustCamshaft(), &normal);
    parameters.minSpeed = 0;
    valvetrain.initialize(parameters);
    engine.setVehicleSpeed(0);
    EXPECT_EQ(valvetrain.getActiveExhaustCamshaft(), &high);
}

TEST(SimulatorRegression, RodCenterOfMassPreservesPinSpacing) {
    for (double offset : {-0.02, 0.0, 0.02}) {
        ConnectingRod rod;
        ConnectingRod::Parameters parameters;
        parameters.length = 0.15;
        parameters.centerOfMass = offset;
        rod.initialize(parameters);
        EXPECT_NEAR(rod.getLittleEndLocal() - rod.getBigEndLocal(), parameters.length, 1e-12);
        // The body's origin is its center of mass, offset from the pin midpoint.
        EXPECT_NEAR((rod.getLittleEndLocal() + rod.getBigEndLocal()) / 2, -offset, 1e-12);
    }
}

TEST(SimulatorRegression, RoadDragOpposesBothRotationDirections) {
    double finalSpeeds[2];
    for (int direction = 0; direction < 2; ++direction) {
        atg_scs::GaussSeidelSleSolver solver;
        atg_scs::OptimizedNsvRigidBodySystem system;
        system.initialize(&solver);
        atg_scs::RigidBody body;
        body.reset();
        body.m = 1000;
        body.I = 100;
        body.v_theta = direction == 0 ? -10 : 10;
        system.addRigidBody(&body);
        Vehicle vehicle;
        vehicle.initialize({1000, 0.3, 2.0, 3.0, 0.3, 100});
        vehicle.addToSystem(&system, &body);
        VehicleDragConstraint drag;
        drag.initialize(&body, &vehicle);
        system.addConstraint(&drag);
        const double initialSpeed = vehicle.getSpeed();
        system.process(0.1, 100);
        finalSpeeds[direction] = vehicle.getSpeed();
        EXPECT_LT(finalSpeeds[direction], initialSpeed);
        EXPECT_GT(finalSpeeds[direction], 0);
    }
    EXPECT_NEAR(finalSpeeds[0], finalSpeeds[1], 1e-10);
}

TEST(SimulatorRegression, IgnitionFiresAcrossCycleBoundaryInBothDirections) {
    const double cycle = 4 * constants::pi;
    for (bool reverse : {false, true}) {
        Crankshaft crank;
        Function timing;
        timing.initialize(0, 1.0);
        timing.addSample(0, 0);
        IgnitionModule ignition;
        ignition.initialize({3, &crank, &timing});
        ignition.m_enabled = true;
        ignition.setFiringOrder(0, cycle - 0.005);
        ignition.setFiringOrder(1, 0.005);
        ignition.setFiringOrder(2, cycle / 2);
        crank.m_body.v_theta = reverse ? 100 : -100;
        crank.m_body.theta = -(reverse ? 0.01 : cycle - 0.01);
        ignition.reset();
        crank.m_body.theta = -(reverse ? cycle - 0.01 : 0.01);
        ignition.update(0.0002);
        EXPECT_TRUE(ignition.getIgnitionEvent(0)) << "reverse=" << reverse;
        EXPECT_TRUE(ignition.getIgnitionEvent(1)) << "reverse=" << reverse;
        EXPECT_FALSE(ignition.getIgnitionEvent(2));
        ignition.resetIgnitionEvents();
        crank.m_body.theta = -(reverse ? cycle - 0.02 : 0.02);
        ignition.update(0.0001);
        for (int cylinder = 0; cylinder < 3; ++cylinder) {
            EXPECT_FALSE(ignition.getIgnitionEvent(cylinder));
        }
        ignition.destroy();
        timing.destroy();
    }
}

TEST(SimulatorRegression, InvalidEngineRejectedBeforeSimulatorInitialization) {
    Vehicle vehicle;
    Transmission transmission;
    Engine engine;
    EXPECT_THROW(engine.createSimulator(&vehicle, &transmission), std::invalid_argument);

    Engine::Parameters params{};
    params.crankshaftCount = params.cylinderCount = params.cylinderBanks = 1;
    params.exhaustSystemCount = params.intakeCount = 1;
    params.initialSimulationFrequency = std::numeric_limits<double>::infinity();
    engine.initialize(params);
    EXPECT_THROW(engine.createSimulator(&vehicle, &transmission), std::invalid_argument);
    engine.destroy();
}

TEST(SimulatorRegression, GenericSolverRepeatedCleanup) {
    PistonEngineSimulator simulator;
    Simulator::Parameters params;
    params.systemType = Simulator::SystemType::Generic;
    simulator.initialize(params);
    simulator.destroy();
    simulator.destroy();
}

TEST(SimulatorRegression, SlowMotionSchedulingDoesNotDependOnDisplayFramerate) {
    for (int fps : {30, 60, 240}) {
        Engine engine;
        PistonEngineSimulator simulator;
        simulator.loadSimulation(&engine, nullptr, nullptr);
        simulator.setSimulationSpeed(0.001);
        int steps = 0;
        for (int frame = 0; frame < fps * 10; ++frame) {
            simulator.startFrame(1.0 / fps);
            steps += simulator.getFrameIterationCount();
        }
        // 10 seconds at 10 kHz and 1/1000 speed, plus the existing 10%
        // catch-up for an empty audio queue. Rounding may leave one step pending.
        EXPECT_NEAR(steps, 110, 1) << "fps=" << fps;
    }
}

TEST(SimulatorRegression, ExhaustOxygenDoesNotRequireUnburnedFuel) {
    Engine engine;
    Engine::Parameters params{};
    params.exhaustSystemCount = 1;
    engine.initialize(params);
    GasSystem &gas = *engine.getExhaustSystem(0)->getSystem();
    GasSystem::Mix mix{};
    mix.p_fuel = 0;
    mix.p_o2 = 1;
    mix.p_inert = 0;
    gas.initialize(units::atm, units::L, 300, mix);
    EXPECT_DOUBLE_EQ(engine.getExhaustO2(), 1.0);
    mix.p_o2 = mix.p_inert = 0.5;
    gas.changeMix(mix);
    EXPECT_NEAR(engine.getExhaustO2(), 0.5332, 0.0001);
    mix.p_o2 = 0;
    mix.p_inert = 1;
    gas.changeMix(mix);
    EXPECT_DOUBLE_EQ(engine.getExhaustO2(), 0.0);
    gas.setN(0);
    EXPECT_DOUBLE_EQ(engine.getExhaustO2(), 0.0);
    engine.destroy();
}

TEST(SimulatorRegression, MixtureGaugesUseConfiguredFuelMass) {
    Engine engine;
    Engine::Parameters params{};
    params.intakeCount = params.exhaustSystemCount = 1;
    engine.initialize(params);
    const GasSystem::Mix mix{0.1, 0.7, 0.2};
    engine.getIntake(0)->m_system.initialize(units::atm, units::L, 300, mix);
    engine.getExhaustSystem(0)->getSystem()->initialize(units::atm, units::L, 300, mix);
    Fuel::Parameters fuel;
    fuel.molecularMass = units::mass(100, units::g);
    engine.getFuel()->initialize(fuel);
    const double initialAfr = engine.getIntakeAfr();
    // Per mole of mixture: 6.39976 g oxygen + 19.6098 g inert gas,
    // divided by 10 g of the configured fuel.
    EXPECT_NEAR(initialAfr, 2.600956, 1e-9);
    EXPECT_NEAR(engine.getExhaustO2(), 0.1777239155, 1e-9);
    fuel.molecularMass *= 2;
    engine.getFuel()->initialize(fuel);
    EXPECT_NEAR(engine.getIntakeAfr(), initialAfr / 2, 1e-9);
    EXPECT_NEAR(engine.getExhaustO2(), 0.1390963095, 1e-9);
    engine.destroy();
}

TEST(SimulatorRegression, HayabusaAndV12Lifecycle) {
    for (const char *path : {"test/scripts/hayabusa.mr", "test/scripts/ferrari_v12.mr"}) {
        es_script::Compiler compiler;
        compiler.initialize(std::string(ENGINE_SIM_TEST_SOURCE_DIR) + "/es");
        const bool compiled = compiler.compile(std::string(ENGINE_SIM_TEST_SOURCE_DIR) + "/" + path);
        if (!compiled) {
            compiler.destroy();
            FAIL() << "Compilation failed: " << path << "; see error_log.log";
        }
        EngineOwner engine;
        engine.output = compiler.execute();
        compiler.destroy();
        ASSERT_TRUE(engine.output.success);
        ASSERT_NE(engine.output.vehicle, nullptr);
        ASSERT_NE(engine.output.transmission, nullptr);
        // Reject invalid vehicle data before attaching bodies or starting audio.
        const Vehicle *vehicle = engine.output.vehicle;
        const Vehicle::Parameters validVehicle = {
            vehicle->getMass(), vehicle->getDragCoefficient(), vehicle->getCrossSectionArea(),
            vehicle->getDiffRatio(), vehicle->getTireRadius(), vehicle->getRollingResistance()
        };
        for (double Vehicle::Parameters::*field : {
                &Vehicle::Parameters::mass, &Vehicle::Parameters::tireRadius,
                &Vehicle::Parameters::diffRatio, &Vehicle::Parameters::dragCoefficient,
                &Vehicle::Parameters::crossSectionArea, &Vehicle::Parameters::rollingResistance }) {
            Vehicle::Parameters invalid = validVehicle;
            invalid.*field = std::numeric_limits<double>::quiet_NaN();
            Vehicle invalidVehicle;
            invalidVehicle.initialize(invalid);
            EXPECT_THROW(engine.output.engine->createSimulator(
                &invalidVehicle, engine.output.transmission), std::invalid_argument);
        }
        Vehicle::Parameters massless = validVehicle;
        massless.mass = 0;
        Vehicle invalidVehicle;
        invalidVehicle.initialize(massless);
        EXPECT_THROW(engine.output.engine->createSimulator(
            &invalidVehicle, engine.output.transmission), std::invalid_argument);
        for (double ratio : {0.0, std::numeric_limits<double>::infinity()}) {
            Transmission invalidTransmission;
            invalidTransmission.initialize({1, &ratio, 1000.0});
            EXPECT_THROW(engine.output.engine->createSimulator(
                engine.output.vehicle, &invalidTransmission), std::invalid_argument);
        }
        const double ratio = 1.0;
        Transmission invalidTransmission;
        invalidTransmission.initialize({1, &ratio, -1.0});
        EXPECT_THROW(engine.output.engine->createSimulator(
            engine.output.vehicle, &invalidTransmission), std::invalid_argument);
        std::unique_ptr<Simulator> simulator(engine.output.engine->createSimulator(
            engine.output.vehicle, engine.output.transmission));
        simulator->setTargetSynthesizerLatency(0);
        simulator->startFrame(0);
        simulator->endFrame();
        for (int i = 0; i < engine.output.engine->getIntakeCount(); ++i) {
            EXPECT_DOUBLE_EQ(engine.output.engine->getIntake(i)->m_flowRate, 0);
        }
        simulator->startFrame(0.001);
        while (simulator->simulateStep()) {}
        simulator->endFrame();
        simulator->synthesizer().renderAudio();
        const double lastFlow = engine.output.engine->getIntakeFlowRate();
        EXPECT_NE(lastFlow, 0.0);
        simulator->startFrame(0);
        EXPECT_EQ(simulator->getFrameIterationCount(), 0);
        simulator->endFrame();
        EXPECT_DOUBLE_EQ(engine.output.engine->getIntakeFlowRate(), lastFlow);
        simulator->setSimulationFrequency(20000);
        simulator->destroy();
        simulator->destroy();
        EXPECT_EQ(simulator->getEngine(), nullptr);
    }
}

TEST(SimulatorRegression, SeparatedPortsPreserveSharedReservoirAndCylinderEvolution) {
    CombustionChamber lumped;
    EXPECT_FALSE(lumped.supportsSeparatedPorts());
    EXPECT_THROW(lumped.flowReservoirPorts(1e-5),std::logic_error);
    EXPECT_THROW(lumped.flowCylinderPorts(1e-5),std::logic_error);
    for(const char *path : {"test/scripts/hayabusa.mr","test/scripts/ferrari_v12.mr"}) {
        for(int cells : {2,8,64}) {
            SCOPED_TRACE(std::string(path)+" cells="+std::to_string(cells));
            EngineOwner owners[2];
            std::unique_ptr<Simulator> simulators[2];
            std::vector<GasPipe *> pipes[2];
            for(int variant=0;variant<2;++variant) {
                es_script::Compiler compiler;
                compiler.initialize(std::string(ENGINE_SIM_TEST_SOURCE_DIR)+"/es");
                ASSERT_TRUE(compiler.compile(std::string(ENGINE_SIM_TEST_SOURCE_DIR)+"/"+path));
                owners[variant].output=compiler.execute();
                compiler.destroy();
                auto &out=owners[variant].output;
                ASSERT_TRUE(out.success);
                simulators[variant].reset(out.engine->createSimulator(out.vehicle,out.transmission));
                std::srand(17);
                for(int j=0;j<out.engine->getCylinderCount();++j) {
                    auto *chamber=out.engine->getChamber(j);
                    chamber->update(1e-5);
                    chamber->m_system.initialize(2e5+j*10000,chamber->getVolume(),900,{.015,.795,.19});
                    for(auto *pipe : {chamber->intakePipe(),chamber->exhaustPipe()}) {
                        auto prototype=pipe->cell(0);
                        prototype.setVolume(.001);
                        pipe->initialize(prototype,1,.001,cells);
                        for(int k=0;k<cells;++k)
                            pipe->cell(k).initialize(1e5+(j+k)%3*8e4,.001/cells,300+10*k,{.015,.795,.19});
                        pipes[variant].push_back(pipe);
                    }
                    chamber->ignite();
                }
            }
            const auto compareGas=[](const GasSystem &a,const GasSystem &b) {
                EXPECT_EQ(a.n(),b.n()); EXPECT_EQ(a.kineticEnergy(),b.kineticEnergy());
                EXPECT_EQ(a.volume(),b.volume());
                EXPECT_EQ(a.velocity_x(),b.velocity_x()); EXPECT_EQ(a.velocity_y(),b.velocity_y());
                EXPECT_EQ(a.mix().p_fuel,b.mix().p_fuel); EXPECT_EQ(a.mix().p_o2,b.mix().p_o2);
                EXPECT_EQ(a.mix().p_inert,b.mix().p_inert); EXPECT_EQ(a.mix().p_co2,b.mix().p_co2);
                EXPECT_EQ(a.mix().p_h2o,b.mix().p_h2o); EXPECT_EQ(a.mix().residualFraction,b.mix().residualFraction);
            };
            auto *reference=owners[0].output.engine,*separated=owners[1].output.engine;
            for(int step=0;step<20;++step) {
                SCOPED_TRACE(step);
                constexpr double dt=1e-7;
                for(int j=0;j<reference->getCylinderCount();++j) reference->getChamber(j)->flowPorts(dt);
                for(int j=0;j<separated->getCylinderCount();++j) separated->getChamber(j)->flowReservoirPorts(dt);
                std::vector<gpu_chamber::Cylinder> deviceCylinders;
                if(gpu_pipe::enabled()) for(int j=0;j<separated->getCylinderCount();++j) {
                    auto *chamber=separated->getChamber(j);
                    deviceCylinders.push_back({chamber->cylinderFlowState(),chamber->cylinderFlowParameters(),
                        chamber->intakePipe()->last(),chamber->exhaustPipe()->first()});
                }
                for(int j=0;j<separated->getCylinderCount();++j) separated->getChamber(j)->flowCylinderPorts(dt);
                if(!deviceCylinders.empty()) {
                    gpu_chamber::advance(deviceCylinders.data(),static_cast<int>(deviceCylinders.size()),dt);
                    for(int j=0;j<separated->getCylinderCount();++j) {
                        auto *chamber=separated->getChamber(j);
                        const auto state=chamber->cylinderFlowState();
                        const auto &device=deviceCylinders[j];
                        for(auto pair : {std::make_pair(&device.state.system,&state.system),
                                std::make_pair(&device.intake,static_cast<const GasSystem *>(&chamber->intakePipe()->last())),
                                std::make_pair(&device.exhaust,static_cast<const GasSystem *>(&chamber->exhaustPipe()->first()))}) {
                            EXPECT_NEAR(pair.first->n(),pair.second->n(),1e-9*(std::max)(1e-10,pair.second->n()));
                            EXPECT_NEAR(pair.first->totalEnergy(),pair.second->totalEnergy(),1e-8*(std::max)(1.0,pair.second->totalEnergy()));
                        }
                        EXPECT_EQ(device.state.lit,state.lit);
                        EXPECT_NEAR(device.state.thermal.wallTemperature(),state.thermal.wallTemperature(),1e-7);
                        EXPECT_NEAR(device.state.thermal.coolantEnergy(),state.thermal.coolantEnergy(),1e-9);
                        EXPECT_NEAR(device.state.burntFuel,state.burntFuel,1e-12);
                        EXPECT_NEAR(device.state.totalIntakeFlow,state.totalIntakeFlow,1e-12);
                        EXPECT_NEAR(device.state.totalExhaustFlow,state.totalExhaustFlow,1e-12);
                        // Exercise importing a packet without perturbing the exact CPU reference.
                        chamber->applyCylinderFlowState(state);
                    }
                }
                for(int variant=0;variant<2;++variant)
                    GasPipe::advanceBatch(pipes[variant].data(),static_cast<int>(pipes[variant].size()),dt);
                for(int j=0;j<reference->getCylinderCount();++j) {
                    auto *a=reference->getChamber(j),*b=separated->getChamber(j);
                    compareGas(a->m_system,b->m_system);
                    EXPECT_EQ(a->getWallTemperature(),b->getWallTemperature());
                    EXPECT_EQ(a->getCoolantEnergy(),b->getCoolantEnergy());
                    EXPECT_EQ(a->getLastTimestepIntakeFlow(),b->getLastTimestepIntakeFlow());
                    EXPECT_EQ(a->getLastTimestepExhaustFlow(),b->getLastTimestepExhaustFlow());
                    EXPECT_EQ(a->isLit(),b->isLit());
                    EXPECT_EQ(a->m_flameEvent.lit_n,b->m_flameEvent.lit_n);
                    EXPECT_EQ(a->m_flameEvent.percentageLit,b->m_flameEvent.percentageLit);
                    // Reservoir array order can differ between separately
                    // compiled engines; compare the actual port connections.
                    compareGas(a->getCylinderHead()->getIntake(a->getPiston()->getCylinderIndex())->m_system,
                        b->getCylinderHead()->getIntake(b->getPiston()->getCylinderIndex())->m_system);
                    compareGas(*a->getCylinderHead()->getExhaustSystem(a->getPiston()->getCylinderIndex())->getSystem(),
                        *b->getCylinderHead()->getExhaustSystem(b->getPiston()->getCylinderIndex())->getSystem());
                    for(int k=0;k<cells;++k) {
                        compareGas(a->intakePipe()->cell(k),b->intakePipe()->cell(k));
                        compareGas(a->exhaustPipe()->cell(k),b->exhaustPipe()->cell(k));
                    }
                }
            }
        }
    }
}

TEST(SimulatorRegression, ExecutionDoesNotReturnPreviousOutput) {
    es_script::Compiler compiler;
    compiler.initialize(std::string(ENGINE_SIM_TEST_SOURCE_DIR) + "/es");
    const bool compiled = compiler.compile(std::string(ENGINE_SIM_TEST_SOURCE_DIR) + "/test/scripts/no_engine.mr");
    if (!compiled) { compiler.destroy(); FAIL() << "Could not compile empty script"; }
    Engine previous;
    es_script::Compiler::output()->engine = &previous;
    const auto output = compiler.execute();
    EXPECT_FALSE(output.success);
    EXPECT_EQ(output.engine, nullptr);
    compiler.destroy();
}

TEST(SimulatorRegression, InvalidCurveStopsScriptBeforeEngineCreation) {
    es_script::Compiler compiler;
    compiler.initialize(std::string(ENGINE_SIM_TEST_SOURCE_DIR) + "/es");
    const bool compiled = compiler.compile(std::string(ENGINE_SIM_TEST_SOURCE_DIR)
        + "/test/scripts/invalid_function_sample.mr");
    if (!compiled) { compiler.destroy(); FAIL() << "Could not compile invalid curve fixture"; }
    EngineOwner owner;
    owner.output = compiler.execute();
    EXPECT_FALSE(owner.output.success);
    EXPECT_EQ(owner.output.engine, nullptr);
    compiler.destroy();
    std::ifstream log("error_log.log");
    const std::string message((std::istreambuf_iterator<char>(log)), {});
    EXPECT_NE(message.find("Function samples must have finite coordinates and values"), std::string::npos);
    EXPECT_NE(message.find("invalid_function_sample.mr(5)"), std::string::npos);
}

TEST(SimulatorRegression, IncompleteEngineReportsScriptError) {
    es_script::Compiler compiler;
    compiler.initialize(std::string(ENGINE_SIM_TEST_SOURCE_DIR) + "/es");
    const bool compiled = compiler.compile(std::string(ENGINE_SIM_TEST_SOURCE_DIR)
        + "/test/scripts/incomplete_engine.mr");
    if (!compiled) { compiler.destroy(); FAIL() << "Could not compile incomplete engine fixture"; }
    EngineOwner owner;
    owner.output = compiler.execute();
    EXPECT_FALSE(owner.output.success);
    EXPECT_EQ(owner.output.engine, nullptr);
    compiler.destroy();
    std::ifstream log("error_log.log");
    const std::string message((std::istreambuf_iterator<char>(log)), {});
    EXPECT_NE(message.find("Engine requires at least one crankshaft"), std::string::npos);
    EXPECT_NE(message.find("incomplete_engine.mr(4)"), std::string::npos);
}

TEST(SimulatorRegression, HayabusaCamDurationMatchesFiftyThouLift) {
    es_script::Compiler compiler;
    compiler.initialize(std::string(ENGINE_SIM_TEST_SOURCE_DIR) + "/es");
    const bool compiled = compiler.compile(std::string(ENGINE_SIM_TEST_SOURCE_DIR) + "/test/scripts/hayabusa.mr");
    if (!compiled) { compiler.destroy(); FAIL() << "Could not compile Hayabusa"; }
    EngineOwner owner;
    owner.output = compiler.execute();
    compiler.destroy();
    ASSERT_TRUE(owner.output.success);
    CylinderHead *head = owner.output.engine->getHead(0);
    // The script specifies 240/220 crank degrees at 0.050-inch lift,
    // with gamma 1.2. Each cam turns at half crank speed.
    for (const auto &lobe : {std::make_pair(head->getIntakeCamshaft(), 240.0),
            std::make_pair(head->getExhaustCamshaft(), 220.0)}) {
        const double angle = units::angle(lobe.second / 4, units::deg);
        for (double side : {-1.0, 1.0}) {
            EXPECT_NEAR(lobe.first->sampleLobe(side * angle) / units::thou, 50.0, 0.05);
        }
    }
}

TEST(SimulatorRegression, RadialDisplacementMatchesPistonTravel) {
    es_script::Compiler compiler;
    compiler.initialize(std::string(ENGINE_SIM_TEST_SOURCE_DIR) + "/es");
    const bool compiled = compiler.compile(std::string(ENGINE_SIM_TEST_SOURCE_DIR) + "/test/scripts/radial_9.mr");
    if (!compiled) { compiler.destroy(); FAIL() << "Radial script compilation failed"; }
    EngineOwner owner;
    owner.output = compiler.execute();
    compiler.destroy();
    ASSERT_TRUE(owner.output.success);
    Engine *engine = owner.output.engine;
    struct PlacementProbe : PistonEngineSimulator { using PistonEngineSimulator::placeCylinder; } simulator;
    Simulator::Parameters params;
    params.systemType = Simulator::SystemType::NsvOptimized;
    simulator.initialize(params);
    simulator.setSimulationFrequency(static_cast<int>(engine->getSimulationFrequency()));
    simulator.loadSimulation(engine, owner.output.vehicle, owner.output.transmission);
    engine->calculateDisplacement();

    const int count = engine->getCylinderCount();
    std::vector<double> low(count, std::numeric_limits<double>::infinity());
    std::vector<double> high(count, -std::numeric_limits<double>::infinity());
    for (int step = 0; step < 1000; ++step) {
        // Placement uses the journal's local angle (normally only at startup).
        engine->getCrankshaft(0)->setRodJournalAngle(0, 2 * constants::pi * step / 1000.0);
        // This radial has one master rod and one level of articulated rods.
        for (int i = 0; i < count; ++i)
            if (!engine->getConnectingRod(i)->getMasterRod()) simulator.placeCylinder(i);
        for (int i = 0; i < count; ++i)
            if (engine->getConnectingRod(i)->getMasterRod()) simulator.placeCylinder(i);
        for (int i = 0; i < count; ++i) {
            const double volume = engine->getChamber(i)->getVolume();
            low[i] = (std::min)(low[i], volume);
            high[i] = (std::max)(high[i], volume);
        }
    }
    double sweptVolume = 0;
    for (int i = 0; i < count; ++i) sweptVolume += high[i] - low[i];
    EXPECT_NEAR(engine->getDisplacement(), sweptVolume, 1e-8);
}

static void compareCoupledFluidLoop(bool resetEachFluidStep, bool cpuControl=false, int refinement=1,
    double perturb=0.0, bool cpuCandidate=false, std::vector<double> *exceedance=nullptr,
    bool assertResults=true) {
    if(!cpuCandidate && !gpu_pipe::enabled()) GTEST_SKIP() << "Set ENGINE_SIM_GPU=1 in a CUDA build";
    for(const char *path : {"test/scripts/hayabusa.mr","test/scripts/ferrari_v12.mr"})
        for(int cells : {2,8,64}) {
            SCOPED_TRACE(std::string(path)+" cells="+std::to_string(cells));
            EngineOwner owners[2];
            std::unique_ptr<Simulator> simulators[2];
            std::vector<GasPipe *> pipes[2];
            for(int variant=0;variant<2;++variant) {
                es_script::Compiler compiler;
                compiler.initialize(std::string(ENGINE_SIM_TEST_SOURCE_DIR)+"/es");
                ASSERT_TRUE(compiler.compile(std::string(ENGINE_SIM_TEST_SOURCE_DIR)+"/"+path));
                owners[variant].output=compiler.execute(); compiler.destroy();
                auto &out=owners[variant].output; ASSERT_TRUE(out.success);
                simulators[variant].reset(out.engine->createSimulator(out.vehicle,out.transmission));
                std::srand(17);
                for(int j=0;j<out.engine->getCylinderCount();++j) {
                    auto *chamber=out.engine->getChamber(j); chamber->update(1e-5);
                    chamber->m_system.initialize(2e5+j*10000,chamber->getVolume(),900,{.015,.795,.19});
                    for(auto *pipe : {chamber->intakePipe(),chamber->exhaustPipe()}) {
                        auto prototype=pipe->cell(0); prototype.setVolume(.001);
                        pipe->initialize(prototype,1,.001,cells);
                        for(int k=0;k<cells;++k)
                            pipe->cell(k).initialize(1e5+(j+k)%3*8e4,.001/cells,300+10*k,{.015,.795,.19});
                        pipes[variant].push_back(pipe);
                    }
                    if(perturb!=0.0 && variant==1) {
                        // Machine-epsilon probe of the free-running fixture. With
                        // cpuCandidate both sides then run the identical CPU schedule.
                        chamber->m_system.changeEnergy(perturb*chamber->m_system.kineticEnergy());
                        for(auto *pipe : {chamber->intakePipe(),chamber->exhaustPipe()})
                            for(int k=0;k<cells;++k)
                                pipe->cell(k).changeEnergy(perturb*pipe->cell(k).kineticEnergy());
                    }
                    chamber->ignite();
                }
            }
            auto *reference=owners[0].output.engine,*candidate=owners[1].output.engine;
            // Script compilation may enumerate reservoirs in a different order.
            // Identify matching reservoirs by their connected cylinders.
            std::vector<int> intakeMap(reference->getIntakeCount(),-1),exhaustMap(reference->getExhaustSystemCount(),-1);
            for(int j=0;j<reference->getCylinderCount();++j) {
                SCOPED_TRACE("input cylinder="+std::to_string(j));
                const auto *ca=reference->getChamber(j),*cb=candidate->getChamber(j);
                ASSERT_EQ(ca->manifoldCouplingK(),cb->manifoldCouplingK());
                ASSERT_EQ(ca->collectorCouplingK(),cb->collectorCouplingK());
                const auto *ia=ca->getCylinderHead()->getIntake(ca->getPiston()->getCylinderIndex());
                const auto *ib=cb->getCylinderHead()->getIntake(cb->getPiston()->getCylinderIndex());
                const auto *ea=ca->getCylinderHead()->getExhaustSystem(ca->getPiston()->getCylinderIndex());
                const auto *eb=cb->getCylinderHead()->getExhaustSystem(cb->getPiston()->getCylinderIndex());
                for(int a=0;a<reference->getIntakeCount();++a) if(reference->getIntake(a)==ia)
                    for(int b=0;b<candidate->getIntakeCount();++b) if(candidate->getIntake(b)==ib) {
                        ASSERT_TRUE(intakeMap[a]==-1 || intakeMap[a]==b); intakeMap[a]=b;
                    }
                for(int a=0;a<reference->getExhaustSystemCount();++a) if(reference->getExhaustSystem(a)==ea)
                    for(int b=0;b<candidate->getExhaustSystemCount();++b) if(candidate->getExhaustSystem(b)==eb) {
                        ASSERT_TRUE(exhaustMap[a]==-1 || exhaustMap[a]==b); exhaustMap[a]=b;
                    }
                const auto a=reference->getChamber(j)->cylinderFlowParameters();
                const auto b=candidate->getChamber(j)->cylinderFlowParameters();
                for(auto member : {&chamber_flow::Parameters::volume,&chamber_flow::Parameters::cylinderHeight,
                    &chamber_flow::Parameters::surfaceArea,&chamber_flow::Parameters::bore,
                    &chamber_flow::Parameters::boreArea,&chamber_flow::Parameters::meanPistonSpeed,
                    &chamber_flow::Parameters::blowbyK,&chamber_flow::Parameters::crankcasePressure,
                    &chamber_flow::Parameters::intakeK,&chamber_flow::Parameters::exhaustK,
                    &chamber_flow::Parameters::intakeArea,&chamber_flow::Parameters::exhaustArea,
                    &chamber_flow::Parameters::fuelMass,&chamber_flow::Parameters::fuelEnergyDensity})
                    ASSERT_EQ(a.*member,b.*member);
            }
            for(int j=0;j<reference->getIntakeCount();++j) {
                ASSERT_GE(intakeMap[j],0);
                const auto a=reference->getIntake(j)->flowParameters(),b=candidate->getIntake(intakeMap[j])->flowParameters();
                for(auto member : {&reservoir_flow::IntakeParameters::molecularAfr,&reservoir_flow::IntakeParameters::fuelMass,
                    &reservoir_flow::IntakeParameters::oxygenPerFuel,&reservoir_flow::IntakeParameters::throttlePlatePosition,
                    &reservoir_flow::IntakeParameters::crossSectionArea,&reservoir_flow::IntakeParameters::inputFlowK,
                    &reservoir_flow::IntakeParameters::idleFlowK,&reservoir_flow::IntakeParameters::velocityDecay})
                    ASSERT_EQ(a.*member,b.*member);
            }
            for(int j=0;j<reference->getExhaustSystemCount();++j) {
                ASSERT_GE(exhaustMap[j],0);
                const auto a=reference->getExhaustSystem(j)->flowParameters(),b=candidate->getExhaustSystem(exhaustMap[j])->flowParameters();
                for(auto member : {&reservoir_flow::ExhaustParameters::collectorArea,
                    &reservoir_flow::ExhaustParameters::outletFlowRate,&reservoir_flow::ExhaustParameters::velocityDecay})
                    ASSERT_EQ(a.*member,b.*member);
            }
            auto *gpu=dynamic_cast<PistonEngineSimulator *>(simulators[1].get()); ASSERT_NE(gpu,nullptr);
            const int fluidSteps=resetEachFluidStep?1:8*refinement;
            gpu->setFluidSimulationSteps(fluidSteps);
            // Measurement mode (exceedance != null) records value/tolerance for
            // every comparison instead of asserting; used to place the GPU
            // trajectory inside the CPU conditioning envelope.
            const auto near=[&](double a,double b,double floor=1e-8) {
                const double T=1e-7*(std::max)(floor,std::abs(b));
                if(exceedance) exceedance->push_back(std::abs(a-b)/T);
                if(assertResults) EXPECT_NEAR(a,b,T);
            };
            const auto nearAbs=[&](double a,double b,double T) {
                if(exceedance) exceedance->push_back(std::abs(a-b)/T);
                if(assertResults) EXPECT_NEAR(a,b,T);
            };
            const auto compareGas=[&](const GasSystem &a,const GasSystem &b) {
                near(a.n(),b.n()); near(a.totalEnergy(),b.totalEnergy()); near(a.temperature(),b.temperature());
                near(a.pressure(),b.pressure()); nearAbs(a.velocity_x(),b.velocity_x(),1e-5);
                nearAbs(a.velocity_y(),b.velocity_y(),1e-5);
                near(a.mix().p_fuel,b.mix().p_fuel); near(a.mix().p_o2,b.mix().p_o2);
                near(a.mix().p_co2,b.mix().p_co2); near(a.mix().p_h2o,b.mix().p_h2o);
                near(a.mix().residualFraction,b.mix().residualFraction);
            };
            for(int step=0;step<(resetEachFluidStep?32*refinement:4);++step) {
                SCOPED_TRACE(step);
                // Isolate local CPU/device error using identical evolving inputs.
                // The separate trajectory test never resets either simulation.
                if(resetEachFluidStep) {
                    for(int j=0;j<reference->getCylinderCount();++j) {
                        candidate->getChamber(j)->applyCylinderFlowState(reference->getChamber(j)->cylinderFlowState());
                    }
                    for(int j=0;j<reference->getIntakeCount();++j)
                        candidate->getIntake(intakeMap[j])->applyFlowState(reference->getIntake(j)->flowState());
                    for(int j=0;j<reference->getExhaustSystemCount();++j)
                        candidate->getExhaustSystem(exhaustMap[j])->applyFlowState(reference->getExhaustSystem(j)->flowState());
                    for(size_t j=0;j<pipes[0].size();++j) {
                        gpu_pipe::Pipe description{};
                        std::vector<GasSystem> state(pipes[0][j]->count());
                        pipes[0][j]->exportCoupled(description,state.data());
                        pipes[1][j]->importCoupled(state.data(),description.stableTimestep);
                        ASSERT_DOUBLE_EQ(pipes[0][j]->stableTimestep(),pipes[1][j]->stableTimestep());
                    }
                }
                const double fluidDt=1e-4/(8*refinement);
                const double timestep=fluidDt*fluidSteps;
                const auto advanceCpu=[&](Engine *engine,const std::vector<GasPipe *> &activePipes) {
                  for(int fluid=0;fluid<fluidSteps;++fluid) {
                    for(int j=0;j<engine->getExhaustSystemCount();++j) engine->getExhaustSystem(j)->process(fluidDt);
                    for(int j=0;j<engine->getIntakeCount();++j) {
                        auto *intake=engine->getIntake(j); intake->process(fluidDt); intake->m_flowRate+=intake->m_flow;
                    }
                    if(cpuControl) {
                        SCOPED_TRACE("CPU control atmosphere");
                        for(int j=0;j<candidate->getExhaustSystemCount();++j) {
                            candidate->getExhaustSystem(exhaustMap[j])->process(fluidDt);
                            compareGas(*engine->getExhaustSystem(j)->getSystem(),*candidate->getExhaustSystem(exhaustMap[j])->getSystem());
                        }
                        for(int j=0;j<candidate->getIntakeCount();++j) {
                            auto *intake=candidate->getIntake(intakeMap[j]); intake->process(fluidDt); intake->m_flowRate+=intake->m_flow;
                            compareGas(engine->getIntake(j)->m_system,intake->m_system);
                        }
                        if(::testing::Test::HasFailure()) return;
                    }
                    double remaining=fluidDt;
                    int substeps=0;
                    while(remaining>0) {
                        ASSERT_LT(++substeps,10000);
                        double h=remaining;
                        for(auto *pipe:activePipes) h=(std::min)(h,pipe->stableTimestep());
                        for(int j=0;j<engine->getCylinderCount();++j) engine->getChamber(j)->flowReservoirPorts(h);
                        if(cpuControl) {
                            SCOPED_TRACE("CPU control reservoir ports");
                            for(int j=0;j<candidate->getCylinderCount();++j) candidate->getChamber(j)->flowReservoirPorts(h);
                            for(size_t j=0;j<activePipes.size();++j) for(int k=0;k<cells;++k)
                                compareGas(activePipes[j]->cell(k),pipes[1][j]->cell(k));
                            if(::testing::Test::HasFailure()) return;
                        }
                        for(int j=0;j<engine->getCylinderCount();++j) engine->getChamber(j)->flowCylinderPorts(h);
                        if(cpuControl) {
                            SCOPED_TRACE("CPU control cylinder ports");
                            for(int j=0;j<candidate->getCylinderCount();++j) {
                                candidate->getChamber(j)->flowCylinderPorts(h);
                                compareGas(engine->getChamber(j)->m_system,candidate->getChamber(j)->m_system);
                            }
                            if(::testing::Test::HasFailure()) return;
                        }
                        GasPipe::advanceBatch(activePipes.data(),static_cast<int>(activePipes.size()),h);
                        if(cpuControl) GasPipe::advanceBatch(pipes[1].data(),static_cast<int>(pipes[1].size()),h);
                        remaining-=h;
                    }
                  }
                };
                advanceCpu(reference,pipes[0]);
                if(::testing::Test::HasFailure() && cpuControl) return;
                if(cpuCandidate) advanceCpu(candidate,pipes[1]);
                else if(!cpuControl) ASSERT_TRUE(gpu->advanceCoupledFluids(timestep));
                for(int j=0;j<reference->getCylinderCount();++j) {
                    SCOPED_TRACE("cylinder="+std::to_string(j));
                    auto *a=reference->getChamber(j),*b=candidate->getChamber(j);
                    compareGas(a->m_system,b->m_system);
                    near(a->getWallTemperature(),b->getWallTemperature()); near(a->getCoolantEnergy(),b->getCoolantEnergy());
                    near(a->getLastTimestepIntakeFlow(),b->getLastTimestepIntakeFlow());
                    near(a->getLastTimestepExhaustFlow(),b->getLastTimestepExhaustFlow());
                    if(assertResults) EXPECT_EQ(a->isLit(),b->isLit());
                near(a->m_nBurntFuel,b->m_nBurntFuel,1e-12);
                    near(a->m_flameEvent.lit_n,b->m_flameEvent.lit_n);
                    near(a->m_flameEvent.percentageLit,b->m_flameEvent.percentageLit);
                    auto *ai=a->getCylinderHead()->getIntake(a->getPiston()->getCylinderIndex());
                    auto *bi=b->getCylinderHead()->getIntake(b->getPiston()->getCylinderIndex());
                    compareGas(ai->m_system,bi->m_system); near(ai->m_totalFuelInjected,bi->m_totalFuelInjected);
                    near(ai->m_flowRate,bi->m_flowRate); near(ai->m_flow,bi->m_flow);
                    auto *ae=a->getCylinderHead()->getExhaustSystem(a->getPiston()->getCylinderIndex());
                    auto *be=b->getCylinderHead()->getExhaustSystem(b->getPiston()->getCylinderIndex());
                    compareGas(*ae->getSystem(),*be->getSystem()); near(ae->getFlow(),be->getFlow());
                    for(int k=0;k<cells;++k) {
                        SCOPED_TRACE("cell="+std::to_string(k));
                        { SCOPED_TRACE("intake"); compareGas(a->intakePipe()->cell(k),b->intakePipe()->cell(k)); }
                        { SCOPED_TRACE("exhaust"); compareGas(a->exhaustPipe()->cell(k),b->exhaustPipe()->cell(k)); }
                    }
                }
            }
        }
}

// STRICT FREE-RUNNING DIAGNOSTIC (opt in with --gtest_also_run_disabled_tests).
// The absolute 1e-7 threshold is below this fixture's own noise floor: a pure
// CPU-vs-CPU run with a 1e-16 initial probe already fails it (45 assertions,
// p50 1.6x tolerance), and the GPU's accumulated divergence sits inside the
// CPU 1e-14..1e-12 probe envelope while growing exponentially from zero
// (failures 0,0,60,272 across the four mechanical steps). Evidence and
// reproduce steps: build/phase1-h1-findings.md. Implementation agreement is
// gated by CoupledGpuMatchesIdenticalFluidStepInputs,
// CoupledSnapshotCpuControl, CoupledTrajectoryCpuBaseline and
// CoupledTrajectoryWithinConditioningEnvelope instead.
TEST(SimulatorRegression, DISABLED_CoupledGpuPreservesAdaptiveFluidLoop) {
    compareCoupledFluidLoop(false);
}

TEST(SimulatorRegression, CoupledGpuMatchesIdenticalFluidStepInputs) {
    compareCoupledFluidLoop(true);
}

TEST(SimulatorRegression, CoupledSnapshotCpuControl) {
    compareCoupledFluidLoop(true,true);
}

TEST(SimulatorRegression, DISABLED_CoupledFluidRefinementDiagnostic) {
    for(int refinement : {2,4,8}) {
        SCOPED_TRACE("refinement="+std::to_string(refinement));
        compareCoupledFluidLoop(false,false,refinement);
    }
}

// Conditioning envelope for the free-running fixture. Both sides run the
// identical CPU schedule (fluidSteps=8, four 1e-4 mechanical steps) and differ
// only by a controlled machine-epsilon probe of the initial state. The GPU
// trajectory diagnostic is judged against this CPU-vs-CPU envelope, because the
// strict 1e-7 tolerance is only meaningful if the fixture itself can hold it.
//
// Free-running CPU vs CPU with no probe must be exact; this guards the
// comparison harness itself against nondeterminism.
TEST(SimulatorRegression, CoupledTrajectoryCpuBaseline) {
    compareCoupledFluidLoop(false,false,1,0.0,true);
}

TEST(SimulatorRegression, DISABLED_CoupledTrajectoryConditioningEnvelope) {
    for(double probe : {1e-16,1e-15,1e-14,1e-12}) {
        char label[64];
        std::snprintf(label,sizeof(label),"probe=%.0e",probe);
        SCOPED_TRACE(label);
        std::vector<double> exceedance;
        compareCoupledFluidLoop(false,false,1,probe,true,&exceedance,true);
        std::sort(exceedance.begin(),exceedance.end());
        size_t overTol=0;
        for(double v : exceedance) if(!(v<=1.0)) ++overTol;
        const auto at=[&](double q)->double {
            return exceedance.empty()?0.0:exceedance[static_cast<size_t>(q*(exceedance.size()-1))];
        };
        std::fprintf(stderr,"PROBE %s n=%zu over-tol=%zu p99=%.3f max=%.1f\n",
            label,exceedance.size(),overTol,at(0.99),exceedance.empty()?0.0:exceedance.back());
    }
}

// Chaos-aware gate for the coupled GPU trajectory. Both runs are measurement
// runs: every comparison records value/tolerance instead of asserting. The
// GPU's free-running divergence must stay inside the envelope the CPU itself
// produces from the identical schedule when only the initial state is
// displaced by 1e-12. Both runs are deterministic, so the bound is stable. A
// systematic GPU error (bad conservation, wrong port direction, dropped
// stage) grows past it; floating-point reassociation rounding does not - the
// measured GPU result sits between the CPU 1e-14 and 1e-12 probes.
TEST(SimulatorRegression, CoupledTrajectoryWithinConditioningEnvelope) {
    if(!gpu_pipe::enabled()) GTEST_SKIP() << "Set ENGINE_SIM_GPU=1 in a CUDA build";
    std::vector<double> gpuExceedance, cpuEnvelope;
    compareCoupledFluidLoop(false,false,1,0.0,false,&gpuExceedance,false);
    compareCoupledFluidLoop(false,false,1,1e-12,true,&cpuEnvelope,false);
    const auto summarize=[](std::vector<double> values) {
        std::sort(values.begin(),values.end());
        const auto at=[&](double q)->double {
            if(values.empty()) return 0.0;
            return values[static_cast<size_t>(q*(values.size()-1))];
        };
        const auto failing=[&]()->size_t {
            size_t n=0;
            for(double v : values) if(!(v<=1.0)) ++n;
            return n;
        };
        return std::make_tuple(values.size(),failing(),at(0.99),values.empty()?0.0:values.back());
    };
    const auto nanCount=[](const std::vector<double> &v) {
        size_t n=0; for(double x : v) if(std::isnan(x)) ++n; return n;
    };
    const auto gpuNan=nanCount(gpuExceedance), cpuNan=nanCount(cpuEnvelope);
    const auto gpu=summarize(std::move(gpuExceedance));
    const auto cpu=summarize(std::move(cpuEnvelope));
    std::fprintf(stderr,"ENVELOPE gpu n=%zu over-tol=%zu p99=%.3f max=%.1f\n",
        std::get<0>(gpu),std::get<1>(gpu),std::get<2>(gpu),std::get<3>(gpu));
    std::fprintf(stderr,"ENVELOPE cpu(1e-12) n=%zu over-tol=%zu p99=%.3f max=%.1f\n",
        std::get<0>(cpu),std::get<1>(cpu),std::get<2>(cpu),std::get<3>(cpu));
    std::fprintf(stderr,"ENVELOPE raw gpu-nan=%zu cpu-nan=%zu\n",gpuNan,cpuNan);
    if(std::get<0>(gpu)==0)
        std::fprintf(stderr,"ENVELOPE note: fixture produced no GPU divergence; envelope gate is vacuous\n");
    EXPECT_LE(static_cast<double>(std::get<1>(gpu)),static_cast<double>(std::get<1>(cpu))*8.0)
        << "GPU over-tolerance comparison count exceeds the CPU 1e-12 conditioning envelope";
    EXPECT_LE(std::get<2>(gpu),std::max(1.0,std::get<2>(cpu))*8.0)
        << "GPU p99 divergence exceeds the CPU 1e-12 conditioning envelope";
    EXPECT_LE(std::get<3>(gpu),std::get<3>(cpu)*8.0)
        << "GPU max divergence exceeds the CPU 1e-12 conditioning envelope";
}
