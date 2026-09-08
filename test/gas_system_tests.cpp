#include <gtest/gtest.h>

#include "../include/gas_system.h"
#include "../include/cylinder_thermal_model.h"
#include "../include/fuel.h"
#include "../include/lubrication_model.h"
#include "../include/gas_pipe.h"
#include "../include/units.h"
#include "../include/csv_io.h"

#include <sstream>

TEST(GasSystemTests, CylinderThermalEnergyBalance) {
    GasSystem gas;
    gas.initialize(5e5, 0.0005, 1000);
    CylinderThermalModel thermal;
    CylinderThermalModel::Parameters p;
    p.initialWallTemperature = 300;
    p.coolantTemperature = 290;
    p.wallHeatCapacity = 500;
    thermal.initialize(p);
    const double initial = gas.kineticEnergy() + p.wallHeatCapacity * thermal.wallTemperature();
    for (int i = 0; i < 1000; ++i) thermal.exchange(gas, 0.04, 10, 0.001);
    const double final = gas.kineticEnergy() + p.wallHeatCapacity * thermal.wallTemperature()
        + thermal.coolantEnergy();
    EXPECT_NEAR(final, initial, initial * 1e-11);
    EXPECT_LT(gas.temperature(), 1000);
    EXPECT_GT(gas.temperature(), p.coolantTemperature);
    EXPECT_GT(thermal.coolantEnergy(), 0);
}

TEST(GasSystemTests, CylinderThermalEquilibriumAndLargeStep) {
    GasSystem gas;
    gas.initialize(1e5, 0.0005, 300);
    CylinderThermalModel thermal;
    CylinderThermalModel::Parameters p;
    p.initialWallTemperature = p.coolantTemperature = 300;
    thermal.initialize(p);
    thermal.exchange(gas, 0.04, 10, 1);
    EXPECT_NEAR(gas.temperature(), 300, 1e-10);
    EXPECT_NEAR(thermal.wallTemperature(), 300, 1e-10);
    gas.changeTemperature(2000);
    thermal.exchange(gas, 0.04, 10, 1000);
    EXPECT_GE(gas.temperature(), 300);
    EXPECT_LE(gas.temperature(), 2300);
    EXPECT_GE(thermal.wallTemperature(), 300);
    EXPECT_LE(thermal.wallTemperature(), 2300);
}

TEST(GasSystemTests, GasSystemSanity) {
    GasSystem system;
    system.initialize(0.0, 0.0, 0.0);
}

TEST(GasSystemTests, DistributedPipeWaveTravelAndConservation) {
    GasSystem prototype;
    prototype.setVariableProperties(true);
    const GasSystem::Mix air{0,0.79,0.21};
    prototype.initialize(1e5,0.001,300,air);
    GasPipe pipe;
    pipe.initialize(prototype,1.0,0.001,64);
    double mass=0,energy=0;
    const double gamma=prototype.heatCapacityRatio();
    for (int i=0;i<pipe.count();++i) {
        const double p=1e5+100*std::cos(constants::pi*(i+0.5)/pipe.count());
        pipe.cell(i).initialize(p,0.001/pipe.count(),300*std::pow(p/1e5,(gamma-1)/gamma),air);
        mass+=pipe.cell(i).mass(); energy+=pipe.cell(i).totalEnergy();
    }
    // Half a period of the fundamental closed-pipe standing acoustic wave.
    pipe.advance(1.0/prototype.c());
    double finalMass=0,finalEnergy=0;
    for (int i=0;i<pipe.count();++i) {
        finalMass+=pipe.cell(i).mass(); finalEnergy+=pipe.cell(i).totalEnergy();
    }
    EXPECT_NEAR(finalMass,mass,1e-12);
    EXPECT_NEAR(finalEnergy,energy,1e-7);
    EXPECT_LT(pipe.first().pressure(),1e5-50);
    EXPECT_GT(pipe.last().pressure(),1e5+50);
}

TEST(GasSystemTests, OilViscosityAndFrictionHeatBalance) {
    CylinderThermalModel wall;
    CylinderThermalModel::Parameters wp;
    wp.initialWallTemperature=400;
    wp.coolantTemperature=300;
    wall.initialize(wp);
    LubricationModel oil;
    LubricationModel::Parameters p;
    p.initialTemperature=300;
    oil.initialize(p);
    EXPECT_NEAR(oil.viscosity(313.15),p.viscosity40,1e-8);
    EXPECT_NEAR(oil.viscosity(373.15),p.viscosity100,1e-8);
    EXPECT_GT(oil.viscousMultiplier(),1);
    const double initial=wp.wallHeatCapacity*wall.wallTemperature()+p.heatCapacity*oil.temperature();
    for (int i=0;i<100;++i) oil.advance(0.1,1000,wall);
    EXPECT_NEAR(oil.frictionEnergy(),10000,1e-8);
    EXPECT_NEAR(wp.wallHeatCapacity*wall.wallTemperature()+p.heatCapacity*oil.temperature()
        +oil.coolantEnergy()-oil.frictionEnergy(),initial,initial*1e-11);
    EXPECT_GT(oil.temperature(),300);
}

TEST(GasSystemTests, FlameSpeedRespondsToMixtureTemperatureAndDilution) {
    Function turbulence;
    turbulence.initialize(2, 1);
    turbulence.addSample(0,1); turbulence.addSample(100,1);
    Fuel::Parameters p;
    p.turbulenceToFlameSpeedRatio=&turbulence;
    Fuel fuel;
    fuel.initialize(p);
    const double rich=p.molecularAfr/1.21;
    const double reference=fuel.laminarBurningVelocity(rich,298,units::atm);
    EXPECT_NEAR(reference,0.305,1e-12);
    EXPECT_LT(fuel.laminarBurningVelocity(p.molecularAfr/0.8,298,units::atm),reference);
    EXPECT_GT(fuel.laminarBurningVelocity(rich,400,units::atm),reference);
    EXPECT_LT(fuel.laminarBurningVelocity(rich,298,2*units::atm),reference);
    EXPECT_LT(fuel.flameSpeed(0,rich,298,units::atm,0,0,0.1),reference);
    EXPECT_EQ(fuel.flameSpeed(0,rich,298,units::atm,0,0,0.6),0);
    turbulence.destroy();
}

TEST(GasSystemTests, VariablePropertiesTemperatureAndReactionMass) {
    GasSystem gas;
    gas.setVariableProperties(true);
    GasSystem::Mix mix{1.0/60.5, 47.0/60.5, 12.5/60.5};
    for (double t : {150.0, 300.0, 999.9, 1000.1, 2500.0, 6000.0, 7000.0}) {
        gas.initialize(1e5, 0.001, t, mix);
        EXPECT_NEAR(gas.temperature(), t, 1e-5);
        EXPECT_NEAR(gas.pressure(), 1e5, 0.01);
    }
    gas.initialize(1e5, 0.001, 300, mix);
    const double mass = gas.mass();
    const double fuel = gas.n_fuel();
    const double burned = gas.react(gas.n(), mix);
    EXPECT_NEAR(burned, fuel, 1e-12);
    EXPECT_NEAR(gas.mass(), mass, 1e-12);
    EXPECT_NEAR(gas.mix().p_co2 * gas.n(), 8 * fuel, 1e-12);
    EXPECT_NEAR(gas.mix().p_h2o * gas.n(), 9 * fuel, 1e-12);
    gas.changeEnergy(burned * mix.fuelMolecularMass * 44e6);
    EXPECT_GT(gas.temperature(), 1000);
    EXPECT_LT(gas.heatCapacityRatio(), 1.4);
}

TEST(GasSystemTests, VariableMixtureFlowConservesMassAndEnergy) {
    GasSystem a, b;
    a.setVariableProperties(true); b.setVariableProperties(true);
    GasSystem::Mix products{0, 0.9, 0.1};
    products.p_co2 = 0.15; products.p_h2o = 0.2;
    a.initialize(4e5, 0.001, 1200, products);
    b.initialize(1e5, 0.002, 300, {0,0.79,0.21});
    const double mass = a.mass()+b.mass(), energy = a.totalEnergy()+b.totalEnergy();
    const double co2 = a.n()*a.mix().p_co2;
    GasSystem::FlowParameters p{GasSystem::k_28inH2O(100), 1e-5, 1, 0, 0.002, 0.002, &a, &b};
    for (int i=0; i<100; ++i) GasSystem::flow(p);
    EXPECT_NEAR(a.mass()+b.mass(), mass, 1e-12);
    EXPECT_NEAR(a.totalEnergy()+b.totalEnergy(), energy, 1e-7);
    EXPECT_NEAR(a.n()*a.mix().p_co2+b.n()*b.mix().p_co2, co2, 1e-12);
}

TEST(GasSystemTests, VariableHeatCapacityWallEnergyBalance) {
    GasSystem gas;
    gas.setVariableProperties(true);
    GasSystem::Mix products{0, 1, 0};
    products.p_co2=0.2; products.p_h2o=0.3;
    gas.initialize(5e5, 0.0005, 2500, products);
    CylinderThermalModel thermal;
    CylinderThermalModel::Parameters p;
    thermal.initialize(p);
    const double initial=gas.kineticEnergy()+p.wallHeatCapacity*thermal.wallTemperature();
    for (int i=0; i<100; ++i) thermal.exchange(gas, 0.04, 10, 0.01);
    EXPECT_NEAR(gas.kineticEnergy()+p.wallHeatCapacity*thermal.wallTemperature()+thermal.coolantEnergy(), initial, initial*1e-11);
    EXPECT_GE(gas.temperature(), p.coolantTemperature);
    EXPECT_LE(gas.temperature(), 2500);
}

TEST(GasSystemTests, AdiabaticEnergyConservation) {
    constexpr double pistonArea = units::area(1.0, units::cm2);
    constexpr double vesselHeight = units::distance(1.0, units::cm);
    const double compression = vesselHeight * 0.5;
    const int steps = 10000;

    GasSystem system;
    system.initialize(
        units::pressure(1.0, units::atm),
        units::volume(1.0, units::cc),
        units::celcius(25.0)
    );

    const double initialSystemEnergy = system.kineticEnergy();
    const double initialMolecules = system.n();

    double W = 0.0;
    double currentPistonHeight = vesselHeight;
    for (int i = 1; i <= steps; ++i) {
        const double newPistonHeight = vesselHeight - (compression / steps) * i;
        const double dH = (currentPistonHeight - newPistonHeight);
        const double F = system.pressure() * pistonArea;
        W += F * dH;

        system.changeVolume((newPistonHeight - currentPistonHeight) * pistonArea);

        currentPistonHeight = newPistonHeight;
    }

    const double finalSystemEnergy = system.kineticEnergy();
    const double finalMolecules = system.n();

    EXPECT_NEAR(finalMolecules, initialMolecules, 1E-6);
    EXPECT_NEAR(finalSystemEnergy - initialSystemEnergy, W, 1E-4);
}

TEST(GasSystemTests, PressureEqualizationEnergyConservation) {
    GasSystem system1, system2;
    system1.initialize(
        units::pressure(1.0, units::atm),
        units::volume(1000.0, units::cc),
        units::celcius(25.0)
    );

    system2.initialize(
        units::pressure(2.0, units::atm),
        units::volume(1000.0, units::cc),
        units::celcius(25.0)
    );

    const double initialSystemEnergy = system1.totalEnergy() + system2.totalEnergy();
    const double initialMolecules = system1.n() + system2.n();

    GasSystem::FlowParameters params;
    params.k_flow = 0.000001;
    params.crossSectionArea_0 = 1.0;
    params.crossSectionArea_1 = 1.0;
    params.direction_x = 1.0;
    params.direction_y = 0.0;
    params.dt = 1.0;
    params.system_0 = &system1;
    params.system_1 = &system2;

    const double dt = 1 / 100.0;
    const int steps = 1000;
    for (int i = 1; i <= steps; ++i) {
        GasSystem::flow(params);
    }

    const double finalSystemEnergy = system1.totalEnergy() + system2.totalEnergy();
    const double finalMolecules = system1.n() + system2.n();

    const double p0 = system1.pressure();
    const double p1 = system2.pressure();

    EXPECT_NEAR(finalMolecules, initialMolecules, 1E-6);
    EXPECT_NEAR(finalSystemEnergy, initialSystemEnergy, 1E-4);
}

TEST(GasSystemTests, PressureEquilibriumMaxFlow) {
    GasSystem system1, system2;
    system1.initialize(
        units::pressure(1.0, units::atm),
        units::volume(1.0, units::cc),
        units::celcius(2500.0)
    );

    system2.initialize(
        units::pressure(2.0, units::atm),
        units::volume(1.0, units::cc),
        units::celcius(25.0)
    );

    const double maxFlowIn = system1.pressureEquilibriumMaxFlow(&system2);

    // Negative flow enters system1 and carries the donor's energy/mix.
    const double incomingEnergy = system2.kineticEnergyPerMol();
    const auto incomingMix = system2.mix();
    system2.loseN(-maxFlowIn, incomingEnergy);
    system1.gainN(-maxFlowIn, incomingEnergy, incomingMix);

    EXPECT_NEAR(system1.pressure(), system2.pressure(), 1E-6);

    system1.changePressure(units::pressure(100.0, units::atm));

    const double maxFlowOut = system1.pressureEquilibriumMaxFlow(&system2);

    const double outgoingEnergy = system1.kineticEnergyPerMol();
    const auto outgoingMix = system1.mix();
    system1.loseN(maxFlowOut, outgoingEnergy);
    system2.gainN(maxFlowOut, outgoingEnergy, outgoingMix);

    EXPECT_NEAR(system1.pressure(), system2.pressure(), 1E-6);
}

TEST(GasSystemTests, PressureEquilibriumMaxFlowInfinite) {
    GasSystem system1;
    system1.initialize(
        units::pressure(1.0, units::atm),
        units::volume(1.0, units::cc),
        units::celcius(25.0)
    );

    constexpr double P_env = units::pressure(2.0, units::atm);
    constexpr double T_env = units::celcius(25.0);

    const double maxFlow = system1.pressureEquilibriumMaxFlow(P_env, T_env);
    const double E_k_per_mol = GasSystem::kineticEnergyPerMol(T_env, system1.degreesOfFreedom());

    system1.gainN(-maxFlow, E_k_per_mol);

    EXPECT_NEAR(system1.pressure(), P_env, 1E-6);
}

TEST(GasSystemTests, PressureEquilibriumMaxFlowInfiniteOverpressure) {
    GasSystem system1;
    system1.initialize(
        units::pressure(100.0, units::atm),
        units::volume(1.0, units::m3),
        units::celcius(2500.0)
    );

    constexpr double P_env = units::pressure(2.0, units::atm);
    constexpr double T_env = units::celcius(25.0);

    const double maxFlow = system1.pressureEquilibriumMaxFlow(P_env, T_env);

    system1.loseN(maxFlow, system1.kineticEnergyPerMol());

    EXPECT_NEAR(system1.pressure(), P_env, 1E-6);
}

TEST(GasSystemTests, FlowVariableVolume) {
    GasSystem system1;
    system1.initialize(
        units::pressure(100.0, units::atm),
        units::volume(1.0, units::m3),
        units::celcius(25.0)
    );

    constexpr double P_env = units::pressure(2.0, units::atm);
    constexpr double T_env = units::celcius(25.0);

    const double maxFlow = system1.pressureEquilibriumMaxFlow(P_env, T_env);

    constexpr double dV = units::volume(1000000.0, units::cc) / 100;
    for (int i = 0; i < 100; ++i) {
        const double flowRate0 = system1.flow(0.01, 1/60.0, units::pressure(0.1, units::atm), units::celcius(25.0));
        const double flowRate1 = system1.flow(0.01, 1 / 60.0, units::pressure(0.2, units::atm), units::celcius(25.0));
        system1.changeVolume(-dV);
        system1.changeTemperature(100);

        std::cerr << flowRate0 << ", " << flowRate1 << "\n";
    }
}

TEST(GasSystemTests, PowerStrokeTest) {
    GasSystem system1;
    system1.initialize(
        units::pressure(100.0, units::atm),
        units::volume(1.0, units::m3),
        units::celcius(2000.0)
    );

    constexpr double dV = units::volume(1000000.0, units::cc) / 100;
    for (int i = 0; i < 100; ++i) {
        const double flowRate0 = system1.flow(1.0, 1 / 60.0, units::pressure(1.0, units::atm), units::celcius(25.0));
        std::cerr << i << ", " << flowRate0 << ", " << system1.pressure() << "\n";
    }
}

TEST(GasSystemTests, IntakeStrokeTest) {
    atg_csv::CsvData csv;
    csv.initialize();
    csv.m_columns = 3;

    GasSystem system1, system2;
    system1.initialize(
        units::pressure(1.0, units::atm),
        units::volume(1000.0, units::m3),
        units::celcius(25.0)
    );
    system2.initialize(
        units::pressure(1.0, units::atm),
        units::volume(1.0, units::m3),
        units::celcius(25.0)
    );

    csv.write("t");
    csv.write("n");
    csv.write("theoretical");

    constexpr double dV = units::volume(4.0, units::m3);
    constexpr double dt = 0.01;
    system2.changeVolume(dV * dt * 100);
    for (int i = 0; i < 100; ++i) {
        GasSystem::FlowParameters flowParams;
        flowParams.crossSectionArea_0 = 1.0;
        flowParams.crossSectionArea_1 = 1.0;
        flowParams.direction_x = 1.0;
        flowParams.direction_y = 0.0;
        flowParams.dt = dt;
        flowParams.k_flow = GasSystem::k_carb(100000.0);
        flowParams.system_0 = &system1;
        flowParams.system_1 = &system2;

        GasSystem::flow(flowParams);
        //system2.changeVolume(dV * dt);
        //system2.changeTemperature(units::celcius(25.0) - system2.temperature());

        csv.write(std::to_string(i * dt).c_str());
        csv.write(std::to_string(system2.n()).c_str());
        csv.write(std::to_string(
            units::pressure(1.0, units::atm) * system2.volume()
            / (constants::R * units::celcius(25.0))).c_str());
    }

    csv.m_rows = 100 + 1;
    csv.writeCsv("intake_stroke.csv");
    csv.destroy();
}

TEST(GasSystemTests, FlowLimit) {
    GasSystem system1;
    system1.initialize(
        units::pressure(100.0, units::atm),
        units::volume(1.0, units::m3),
        units::celcius(2000.0)
    );

    constexpr double P_env = units::pressure(1.0, units::atm);
    constexpr double T_env = units::celcius(25.0);
    const double maxFlow = system1.pressureEquilibriumMaxFlow(P_env, T_env);

    system1.flow(15.0, 10.0, P_env, T_env);

    EXPECT_NEAR(system1.pressure(), P_env, 1E-6);
}

TEST(GasSystemTests, IdealGasLaw) {
    GasSystem system1;
    system1.initialize(
        units::pressure(100.0, units::atm),
        units::volume(1.0, units::m3),
        units::celcius(2000.0)
    );

    const double PV = system1.pressure() * system1.volume();
    const double nRT = system1.n() * constants::R * system1.temperature();

    EXPECT_NEAR(PV, nRT, 1E-6);
}

TEST(GasSystemTests, CompositionSanityCheck) {
    GasSystem::Mix a, b;
    a.p_fuel = 1.0;
    a.p_inert = 1.0;
    a.p_o2 = 1.0;

    b.p_fuel = 0.0;
    b.p_inert = 0.0;
    b.p_o2 = 0.0;

    GasSystem system1;
    system1.initialize(
        units::pressure(100.0, units::atm),
        units::volume(100.0, units::m3),
        units::celcius(2000.0),
        a
    );

    GasSystem system2;
    system2.initialize(
        units::pressure(1.0, units::atm),
        units::volume(1.0, units::m3),
        units::celcius(2000.0),
        b
    );

    const double PV = system1.pressure() * system1.volume();
    const double nRT = system1.n() * constants::R * system1.temperature();

    GasSystem::FlowParameters params;
    params.k_flow = 1.0;
    params.crossSectionArea_0 = 1.0;
    params.crossSectionArea_1 = 1.0;
    params.direction_x = 1.0;
    params.direction_y = 0.0;
    params.dt = 1 / 60.0;
    params.system_0 = &system1;
    params.system_1 = &system2;

    for (int i = 0; i < 200; ++i) {
        const double flowRate = GasSystem::flow(params);
        std::cerr << i << ", " << flowRate << ", " << system1.pressure() << "\n";
    }

    EXPECT_NEAR(system2.mix().p_fuel, 1.0, 2E-2);
    EXPECT_NEAR(system2.mix().p_inert, 1.0, 2E-2);
    EXPECT_NEAR(system2.mix().p_o2, 1.0, 2E-2);
}

TEST(GasSystemTests, ChokedFlowTest) {
    GasSystem system1;
    system1.initialize(
        units::pressure(2.5, units::atm),
        units::volume(1.0, units::m3),
        units::celcius(2000.0)
    );

    const double flow_k = GasSystem::flowConstant(
        units::flow(400, units::scfm),
        units::pressure(2.5, units::atm),
        units::pressure(1.5, units::atm),
        units::celcius(2000.0),
        GasSystem::heatCapacityRatio(5)
    );

    const double chokedFlow =
        system1.flowRate(
            flow_k,
            system1.pressure(),
            units::pressure(1.0, units::atm),
            system1.temperature(),
            units::celcius(25),
            GasSystem::heatCapacityRatio(5),
            GasSystem::chokedFlowLimit(5),
            GasSystem::chokedFlowRate(5));
    const double noncriticalFlow =
        system1.flowRate(
            flow_k,
            system1.pressure(),
            units::pressure(2.0, units::atm),
            system1.temperature(),
            units::celcius(25),
            GasSystem::heatCapacityRatio(5),
            GasSystem::chokedFlowLimit(5),
            GasSystem::chokedFlowRate(5));

    const double chokedFlowScfm = units::convert(chokedFlow, units::scfm);
    const double noncriticalFlowScfm = units::convert(noncriticalFlow, units::scfm);
}

TEST(GasSystemTests, CfmConversions) {
    constexpr double standardPressure = units::pressure(1.0, units::atm);
    constexpr double standardTemp = units::celcius(25.0);
    constexpr double airDensity =
        units::AirMolecularMass * (standardPressure * units::volume(1.0, units::m3))
        / (constants::R * standardTemp);

    const double flow_28 = GasSystem::k_28inH2O(300);
    
    const double flowRate = GasSystem::flowRate(
        flow_28,
        units::pressure(1.0, units::atm),
        units::pressure(1.0, units::atm) - units::pressure(41.0, units::inH2O),
        units::celcius(25.0),
        units::celcius(25.0),
        GasSystem::heatCapacityRatio(5),
        GasSystem::chokedFlowLimit(5),
        GasSystem::chokedFlowRate(5));

    const double flowRateCfm = units::convert(flowRate, units::scfm);
}

TEST(GasSystemTests, FlowRateConstant) {
    const double flow_k = GasSystem::flowConstant(
        units::flow(400, units::scfm),
        units::pressure(2.5, units::atm),
        units::pressure(0.5, units::atm),
        units::celcius(2000.0),
        GasSystem::heatCapacityRatio(5)
    );

    const double flowRate =
        GasSystem::flowRate(
            flow_k,
            units::pressure(2.5, units::atm),
            units::pressure(2.5 - 0.5, units::atm),
            units::celcius(2000.0),
            units::celcius(25),
            GasSystem::heatCapacityRatio(5),
            GasSystem::chokedFlowLimit(5),
            GasSystem::chokedFlowRate(5));

    EXPECT_NEAR(flowRate, units::flow(400, units::scfm), 1E-6);
}

TEST(GasSystemTests, GasVelocityReducesStaticPressure) {
    atg_csv::CsvData csv;
    csv.initialize();
    csv.m_columns = 6;

    GasSystem system1, system2;
    system1.initialize(
        units::pressure(15, units::psi),
        units::volume(300, units::cc),
        units::celcius(25.0)
    );
    system1.setGeometry(units::distance(10, units::cm), units::distance(10, units::cm), 1.0, 0.0);

    system2.initialize(
        units::pressure(2, units::psi),
        units::volume(1.0, units::L),
        units::celcius(25.0)
    );
    system2.setGeometry(units::distance(10, units::cm), units::distance(2, units::cm), 1.0, 0.0);

    const double initialSystemEnergy = system1.totalEnergy() + system2.totalEnergy();
    const double initialMolecules = system1.n() + system2.n();

    GasSystem::FlowParameters params;
    params.k_flow = GasSystem::k_28inH2O(500.0) * 1.0;
    params.crossSectionArea_0 = 50.0 * units::cm * units::cm;
    params.crossSectionArea_1 = 4.0 * units::cm * units::cm;
    params.direction_x = 1.0;
    params.direction_y = 0.0;
    params.dt = 1 / 10000.0;
    params.system_0 = &system1;
    params.system_1 = &system2;

    csv.write("time");
    csv.write("P_0");
    csv.write("P_1");
    csv.write("v_0");
    csv.write("v_1");
    csv.write("total_energy");

    const int steps = 1000;
    for (int i = 1; i <= steps; ++i) {
        const double staticPressure =
            system2.pressure() + system2.dynamicPressure(0.0, 1.0);
        const double totalPressure =
            system2.pressure() + system2.dynamicPressure(1.0, 0.0);

        const double systemEnergy = system1.totalEnergy() + system2.totalEnergy();
        const double velocity_x_0 = system1.velocity_x();
        const double velocity_x_1 = system2.velocity_x();

        const double P_0 = system1.pressure();
        const double P_1 = system2.pressure() + system2.dynamicPressure(-1.0, 0.0);

        GasSystem::flow(params);
        system1.updateVelocity(params.dt);
        system2.updateVelocity(params.dt);

        //system1.dissipateVelocity(params.dt, 0.01);
        //system2.dissipateVelocity(params.dt, 0.01);

        ++csv.m_rows;
        csv.write(std::to_string(i * params.dt).c_str());
        csv.write(std::to_string(P_0).c_str());
        csv.write(std::to_string(P_1).c_str());
        csv.write(std::to_string(velocity_x_0).c_str());
        csv.write(std::to_string(velocity_x_1).c_str());
        csv.write(std::to_string(systemEnergy).c_str());
    }

    const double finalSystemEnergy = system1.totalEnergy() + system2.totalEnergy();
    const double finalMolecules = system1.n() + system2.n();

    const double p0 = system1.pressure();
    const double p1 = system2.pressure();

    EXPECT_NEAR(finalMolecules, initialMolecules, 1E-6);
    EXPECT_NEAR(finalSystemEnergy, initialSystemEnergy, 1E-4);

    csv.writeCsv("gas_system_test_output.csv", nullptr, '\t');
    csv.destroy();
}

TEST(GasSystemTests, GasVelocityProducesScavengingEffect) {
    atg_csv::CsvData csv;
    csv.initialize();
    csv.m_columns = 7;

    constexpr double cylinderArea =
        constants::pi * units::distance(2.0, units::inch) * units::distance(2.0, units::inch);
    constexpr double tubeArea =
        constants::pi * units::distance(1.75 / 2, units::inch) * units::distance(1.75 / 2, units::inch);

    GasSystem system1, system2, atmosphere;
    system1.initialize(
        units::pressure(1000, units::psi),
        units::volume(1000, units::cc),
        units::celcius(1000.0)
    );
    system1.setGeometry(
        units::distance(10.0, units::cm),
        units::distance(1.0, units::cm),
        1.0,
        0.0);

    system2.initialize(
        units::pressure(15, units::psi),
        tubeArea * units::distance(50.0, units::inch),
        units::celcius(25.0)
    );
    system2.setGeometry(
        units::distance(50.0, units::inch),
        std::sqrt(tubeArea),
        1.0,
        0.0);

    atmosphere.initialize(
        units::pressure(15, units::psi),
        units::volume(10000, units::m3),
        units::celcius(25.0)
    );

    const double initialSystemEnergy =
        system1.totalEnergy()
        + system2.totalEnergy()
        + atmosphere.totalEnergy();
    const double initialMolecules = system1.n() + system2.n();

    const double atmosphereArea =
        1000.0;

    GasSystem::FlowParameters params;
    params.k_flow = GasSystem::k_28inH2O(230.0) * 1.0;        
    params.direction_x = 1.0;
    params.direction_y = 0.0;
    params.dt = 1 / (16 * 4000.0);
    params.system_0 = &system1;
    params.system_1 = &system2;

    csv.write("iteration");
    csv.write("time");
    csv.write("static_cylinder_pressure");
    csv.write("exhaust_pressure");
    csv.write("v_0");
    csv.write("v_1");
    csv.write("exhaust_static_pressure");

    const int steps = 10000;
    for (int i = 1; i <= steps; ++i) {
        const double staticPressure =
            system2.pressure() + system2.dynamicPressure(0.0, 1.0);
        const double totalPressure =
            system2.pressure() + system2.dynamicPressure(1.0, 0.0);

        const double systemEnergy0 =
            system1.totalEnergy()
            + system2.totalEnergy()
            + atmosphere.totalEnergy();
        const double velocity_x_0 = system1.velocity_x();
        const double velocity_x_1 = system2.velocity_x();

        const double P_0 = system1.pressure();
        const double P_1 = system2.pressure() + system2.dynamicPressure(1.0, 0.0);
        const double exhaustStaticPressure = system2.pressure();

        if (system1.volume() > units::volume(118.0, units::cc)) {
            system1.changeVolume(-units::volume(200000.0, units::cc) * params.dt);
        }
        else {
            system1.setVolume(units::volume(118.0, units::cc));
        }

        params.system_0 = &system1;
        params.system_1 = &system2;
        params.crossSectionArea_0 = cylinderArea;
        params.crossSectionArea_1 = tubeArea;
        params.k_flow = GasSystem::k_28inH2O(230.0);
        GasSystem::flow(params);

        const double systemEnergy1 =
            system1.totalEnergy()
            + system2.totalEnergy()
            + atmosphere.totalEnergy();

        params.system_0 = &system2;
        params.system_1 = &atmosphere;
        params.crossSectionArea_0 = tubeArea;
        params.crossSectionArea_1 = atmosphereArea;
        params.k_flow = GasSystem::k_carb(1000);
        GasSystem::flow(params);

        const double systemEnergy2 =
            system1.totalEnergy()
            + system2.totalEnergy()
            + atmosphere.totalEnergy();

        system1.updateVelocity(params.dt);
        system2.updateVelocity(params.dt);
        system1.dissipateExcessVelocity();
        system2.dissipateExcessVelocity();

        ++csv.m_rows;
        csv.write(std::to_string(i).c_str());
        csv.write(std::to_string(i * params.dt).c_str());
        csv.write(std::to_string(P_0).c_str());
        csv.write(std::to_string(P_1).c_str());
        csv.write(std::to_string(velocity_x_0).c_str());
        csv.write(std::to_string(velocity_x_1).c_str());
        csv.write(std::to_string(exhaustStaticPressure).c_str());
    }

    csv.writeCsv("gas_system_test_output.csv", nullptr, '\t');
    csv.destroy();
}

TEST(GasSystemTests, GasVelocityProducesRamEffect) {
    atg_csv::CsvData csv;
    csv.initialize();
    csv.m_columns = 9;

    constexpr double cylinderArea =
        constants::pi * units::distance(2.0, units::inch) * units::distance(2.0, units::inch);
    constexpr double runnerArea =
        constants::pi * units::distance(0.5, units::inch) * units::distance(0.5, units::inch);

    GasSystem cylinder, runner, atmosphere;
    cylinder.initialize(
        units::pressure(1.0, units::atm),
        units::volume(118, units::cc),
        units::celcius(25.0)
    );
    cylinder.setGeometry(
        units::distance(10.0, units::cm),
        units::distance(1.0, units::cm),
        1.0,
        0.0);

    runner.initialize(
        units::pressure(1.0, units::atm),
        units::volume(320, units::cc),
        units::celcius(25.0)
    );
    runner.setGeometry(
        units::distance(5.0, units::inch),
        std::sqrt(runnerArea),
        1.0,
        0.0);

    atmosphere.initialize(
        units::pressure(1.0, units::atm),
        units::volume(10000, units::m3),
        units::celcius(25.0)
    );

    const double initialSystemEnergy =
        cylinder.totalEnergy()
        + runner.totalEnergy()
        + atmosphere.totalEnergy();
    const double initialMolecules = cylinder.n() + runner.n();

    const double atmosphereArea =
        1000.0;

    GasSystem::FlowParameters params;
    params.dt = 1 / (16 * 4000.0);
    params.direction_x = 1.0;
    params.direction_y = 0.0;

    csv.write("iteration");
    csv.write("time");
    csv.write("crank_angle");
    csv.write("cylinder_volume");
    csv.write("n_mol");
    csv.write("cylinder_air_velocity");
    csv.write("runner_air_velocity");
    csv.write("cylinder_pressure");
    csv.write("runner_pressure");

    constexpr double speed = 3000; // rpm
    constexpr double stroke = units::distance(4.0, units::inch);

    double max_n = 0.0;
    double max_n_angle = 0.0;

    double max_v = 0.0;
    double max_v_angle = 0.0;

    double flow = 1.0;

    const int steps = 10000;
    for (int i = 1; i <= steps; ++i) {
        const double velocity_x_0 = cylinder.velocity_x();
        const double velocity_x_1 = runner.velocity_x();

        const double P_0 = cylinder.pressure();
        const double P_1 = runner.pressure() + runner.dynamicPressure(1.0, 0.0);
        const double exhaustStaticPressure = runner.pressure();

        const double t = i * params.dt;
        const double pistonHeight =
            units::distance(0.25, units::inch)
            + stroke / 2
            + (stroke / 2) * -std::cos(2 * constants::pi * (t * (speed / 60)));

        cylinder.setVolume(pistonHeight * cylinderArea);
        if (i == 1) {
            cylinder.changePressure(units::pressure(1.0, units::atm) - cylinder.pressure());
        }

        params.system_0 = &runner;
        params.system_1 = &cylinder;
        params.crossSectionArea_0 = runnerArea;
        params.crossSectionArea_1 = cylinderArea;
        params.k_flow = GasSystem::k_28inH2O(230.0) * flow;
        GasSystem::flow(params);

        params.system_0 = &atmosphere;
        params.system_1 = &runner;
        params.crossSectionArea_0 = atmosphereArea;
        params.crossSectionArea_1 = cylinderArea;
        params.k_flow = GasSystem::k_carb(500);
        GasSystem::flow(params);

        cylinder.updateVelocity(params.dt, 1.0);
        runner.updateVelocity(params.dt, 0.1);
        cylinder.dissipateExcessVelocity();
        runner.dissipateExcessVelocity();

        const double angle = 360 * (t * (speed / 60));

        if (cylinder.n() > max_n) {
            max_n = cylinder.n();
            max_n_angle = angle;
        }

        if (cylinder.volume() > max_v) {
            max_v = cylinder.volume();
            max_v_angle = angle;
        }

        ++csv.m_rows;
        csv.write(std::to_string(i).c_str());
        csv.write(std::to_string(i * params.dt).c_str());
        csv.write(std::to_string(angle).c_str());
        csv.write(std::to_string(cylinder.volume()).c_str());
        csv.write(std::to_string(cylinder.n()).c_str());
        csv.write(std::to_string(velocity_x_0).c_str());
        csv.write(std::to_string(velocity_x_1).c_str());
        csv.write(std::to_string(P_0).c_str());
        csv.write(std::to_string(P_1).c_str());
    }

    csv.writeCsv("gas_system_test_output.csv", nullptr, '\t');
    csv.destroy();

    const double delay = max_n_angle - max_v_angle;
    const double m_n = max_n;

    int a = 0;
}

TEST(GasSystemTests, GasVelocityStabilizesInClosedSystem) {
    atg_csv::CsvData csv;
    csv.initialize();
    csv.m_columns = 2;

    GasSystem system1;
    system1.initialize(
        units::pressure(100, units::psi),
        units::volume(1000, units::cc),
        units::celcius(1000.0)
    );
    system1.setGeometry(
        units::distance(10.0, units::cm),
        units::distance(10.0, units::cm),
        1.0,
        0.0);

    const double initialSystemEnergy = system1.totalEnergy();
    const double initialMolecules = system1.n();

    csv.write("time");
    csv.write("velocity");

    const int steps = 10000;
    const double dt = 1 / 10000.0;
    for (int i = 1; i <= steps; ++i) {
        system1.updateVelocity(dt);

        ++csv.m_rows;
        csv.write(std::to_string(i * dt).c_str());
        csv.write(std::to_string(system1.velocity_x()).c_str());
    }

    csv.writeCsv("gas_system_test_output.csv", nullptr, '\t');
    csv.destroy();
}
