#include <gtest/gtest.h>
#include "../scripting/include/compiler.h"
#include "../include/simulator.h"
#include "../include/piston_engine_simulator.h"
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
                for(int j=0;j<separated->getCylinderCount();++j) separated->getChamber(j)->flowCylinderPorts(dt);
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
