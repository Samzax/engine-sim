#include <gtest/gtest.h>
#include "../scripting/include/compiler.h"
#include "../include/simulator.h"
#include "../include/piston_engine_simulator.h"
#include "../include/constants.h"
#include "../include/vehicle_drag_constraint.h"
#include <memory>
#include <limits>
#include <stdexcept>
#include <fstream>
#include <iterator>

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
        simulator->setSimulationFrequency(20000);
        simulator->destroy();
        simulator->destroy();
        EXPECT_EQ(simulator->getEngine(), nullptr);
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
