#ifndef ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H
#define ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H

#include "scs.h"

#include "piston.h"
#include "gas_system.h"
#include "cylinder_head.h"
#include "units.h"
#include "fuel.h"
#include "cylinder_thermal_model.h"
#include "lubrication_model.h"
#include "gas_pipe.h"

class Engine;
class CombustionChamber : public atg_scs::ForceGenerator {
    public:
        struct Parameters {
            Piston *Piston;
            CylinderHead *Head;
            Fuel *Fuel;
            Function *MeanPistonSpeedToTurbulence;

            double StartingPressure;
            double StartingTemperature;
            double CrankcasePressure;
            CylinderThermalModel::Parameters thermal;
            bool dynamicCombustion = true;
            LubricationModel::Parameters lubrication;
            int pipeCells = 8;
            double pipeFrictionFactor = 0.02;
        };

        struct FlameEvent {
            double lit_n = 0;
            double total_n = 0;
            double percentageLit = 0;
            double efficiency = 1.0;
            double flameSpeed = 0.0;
            double ignitionTemperature = 300, ignitionPressure = 101325;
            double unburnedTemperature = 300;

            double lastVolume = 0.0;
            double travel_x = 0.0;
            double travel_y = 0.0;
            GasSystem::Mix globalMix;
        };

        struct FrictionModelParams {
            double frictionCoeff = 0.06;
            double breakawayFriction = units::force(50, units::N);
            double breakawayFrictionVelocity = units::distance(0.1, units::m);
            double viscousFrictionCoefficient = units::force(20, units::N);
        };

    public:
        CombustionChamber();
        virtual ~CombustionChamber();

        void initialize(const Parameters &params);
        void destroy();
        void setEngine(Engine *engine) { m_engine = engine; }
        virtual void apply(atg_scs::SystemState *system);

        CylinderHead *getCylinderHead() const { return m_head; }
        Piston *getPiston() const { return m_piston; }

        double getFrictionForce() const;
        double getVolume() const;
        double pistonSpeed() const;
        double calculateMeanPistonSpeed() const;
        double calculateFiringPressure() const;

        bool isLit() const { return m_lit; }
        bool popLitLastFrame();

        void ignite();
        void update(double dt);
        void flow(double dt);
        void flowPorts(double dt) { flowStep(dt,true); }
        // Run reservoir stages in cylinder order, then cylinder stages, then
        // pipe interiors. Distinct pipe endpoints make the stages independent.
        bool supportsSeparatedPorts() const { return m_intakePipe.active() && m_exhaustPipe.active(); }
        void flowReservoirPorts(double dt);
        void flowCylinderPorts(double dt);
        GasPipe *intakePipe() { return &m_intakePipe; }
        GasPipe *exhaustPipe() { return &m_exhaustPipe; }
        void aggregatePipes() {
            m_intakePipe.aggregate(m_intakeRunnerAndManifold);
            m_exhaustPipe.aggregate(m_exhaustRunnerAndPrimary);
        }
        void configureGas(bool enabled, double fuelMass, double oxygenPerFuel) {
            m_system.setVariableProperties(enabled);
            m_intakeRunnerAndManifold.setVariableProperties(enabled);
            m_exhaustRunnerAndPrimary.setVariableProperties(enabled);
            if (enabled) {
                m_intakeRunnerAndManifold.reset(m_intakeRunnerAndManifold.pressure(),m_intakeRunnerAndManifold.temperature(),{0,0.79,0.21});
                m_exhaustRunnerAndPrimary.reset(m_exhaustRunnerAndPrimary.pressure(),m_exhaustRunnerAndPrimary.temperature(),{0,0.79,0.21});
            }
            m_intakePipe.configure(enabled,fuelMass,oxygenPerFuel);
            m_exhaustPipe.configure(enabled,fuelMass,oxygenPerFuel);
        }

        double lastEventAfr() const;
        double getWallTemperature() const { return m_thermal.wallTemperature(); }
        double getCoolantEnergy() const { return m_thermal.coolantEnergy()+m_lubrication.coolantEnergy(); }
        double getOilTemperature() const { return m_lubrication.temperature(); }
        double getFrictionEnergy() const { return m_lubrication.frictionEnergy(); }

        double getLastIterationExhaustFlow() const { return m_exhaustFlow; }

        void resetLastTimestepExhaustFlow() { m_lastTimestepTotalExhaustFlow = 0; }
        double getLastTimestepExhaustFlow() const { return m_lastTimestepTotalExhaustFlow; }

        void resetLastTimestepIntakeFlow() { m_lastTimestepTotalIntakeFlow = 0; }
        double getLastTimestepIntakeFlow() const { return m_lastTimestepTotalIntakeFlow; }

        Function *m_meanPistonSpeedToTurbulence;
        GasSystem m_system;
        GasSystem m_intakeRunnerAndManifold;
        GasSystem m_exhaustRunnerAndPrimary;
        FlameEvent m_flameEvent;
        bool m_lit;

        FrictionModelParams m_frictionModel;

        double m_peakTemperature;
        double m_nBurntFuel;

    protected:
        double calculateFrictionForce(double v) const;
        void updateCycleStates();

        double m_intakeFlowRate;
        double m_exhaustFlowRate;

        double m_manifoldToRunnerFlowRate;
        double m_primaryToCollectorFlowRate;
        double m_cylinderCrossSectionSurfaceArea;
        double m_cylinderWidthApproximation;

        double m_lastTimestepTotalExhaustFlow;
        double m_lastTimestepTotalIntakeFlow;
        double m_exhaustFlow;

        double m_crankcasePressure;

        double *m_pressure;
        double *m_pistonSpeed;
        double m_pistonSpeedSum = 0;
        static constexpr int StateSamples = 256;

        bool m_litLastFrame;

        Piston *m_piston;
        CylinderHead *m_head;
        Engine *m_engine;
        Fuel *m_fuel;
        CylinderThermalModel m_thermal;
        bool m_dynamicCombustion = true;
        LubricationModel m_lubrication;
        GasPipe m_intakePipe, m_exhaustPipe;
        void flowStep(double dt, bool deferPipes=false, bool reservoirPortsDone=false);
        void flowIntakeReservoir(double dt);
        void flowExhaustReservoir(double dt);
};

#endif /* ATG_ENGINE_SIM_COMBUSTION_CHAMBER_H */
