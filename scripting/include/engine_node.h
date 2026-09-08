#ifndef ATG_ENGINE_SIM_ENGINE_NODE_H
#define ATG_ENGINE_SIM_ENGINE_NODE_H

#include "object_reference_node.h"

#include "crankshaft_node.h"
#include "cylinder_bank_node.h"
#include "ignition_module_node.h"
#include "engine_context.h"
#include "fuel_node.h"
#include "throttle_nodes.h"

#include "engine_sim.h"

#include <map>
#include <set>

namespace es_script {

    class EngineNode : public ObjectReferenceNode<EngineNode> {
    public:
        EngineNode() { /* void */ }
        virtual ~EngineNode() { /* void */ }

        std::string validateAssembly() const {
            if (m_crankshafts.empty()) return "Engine requires at least one crankshaft";
            if (m_cylinderBanks.empty()) return "Engine requires at least one cylinder bank";
            if (m_ignitionModule == nullptr) return "Engine requires an ignition module";
            if (m_fuel == nullptr || m_throttle == nullptr)
                return "Engine requires fuel and throttle definitions";
            const std::set<CrankshaftNode *> crankshafts(m_crankshafts.begin(), m_crankshafts.end());
            if (crankshafts.size() != m_crankshafts.size())
                return "Engine cannot attach the same crankshaft instance more than once";
            std::map<ConnectingRodNode *, const RodJournalNode *> rods;
            for (const CylinderBankNode *bank : m_cylinderBanks) {
                if (bank->getCylinderCount() == 0) return "Cylinder bank requires at least one cylinder";
                if (bank->getCylinderHead() == nullptr) return "Cylinder bank requires a cylinder head";
                if (!bank->getCylinderHead()->hasLobesForCylinders(bank->getCylinderCount()))
                    return "Every intake and exhaust camshaft, including VTEC profiles, requires a lobe for each cylinder in its bank";
                for (int i = 0; i < bank->getCylinderCount(); ++i) {
                    const auto &cylinder = bank->getCylinder(i);
                    if (!rods.emplace(cylinder.rod, cylinder.rodJournal).second)
                        return "Each cylinder requires a separate connecting rod instance";
                }
            }
            for (const CylinderBankNode *bank : m_cylinderBanks) {
                for (int i = 0; i < bank->getCylinderCount(); ++i) {
                    const RodJournalNode *journal = bank->getCylinder(i).rodJournal;
                    const bool hasCrankshaft = journal->getCrankshaft() != nullptr;
                    const bool hasMasterRod = journal->getRod() != nullptr;
                    if (hasCrankshaft == hasMasterRod
                        || (hasCrankshaft && crankshafts.count(journal->getCrankshaft()) == 0)
                        || (hasMasterRod && rods.count(journal->getRod()) == 0)) {
                        return "Cylinder rod journal must belong to a crankshaft or master rod in this engine";
                    }
                }
            }
            // Every chain of master rods must terminate at a crankshaft. A loop
            // otherwise reaches the recursive runtime rod traversal intact.
            for (const auto &entry : rods) {
                std::set<ConnectingRodNode *> visited;
                ConnectingRodNode *rod = entry.first;
                while (rod != nullptr) {
                    if (!visited.insert(rod).second)
                        return "Connecting rod assembly contains a master rod cycle";
                    rod = rods.at(rod)->getRod();
                }
            }
            const std::set<CylinderBankNode *> banks(m_cylinderBanks.begin(), m_cylinderBanks.end());
            if (!m_ignitionModule->connectionsBelongTo(banks))
                return "Ignition wire connects to a cylinder bank outside this engine";
            return {};
        }

        void buildEngine(Engine *engine) {
            int cylinderCount = 0;
            for (const CylinderBankNode *bank : m_cylinderBanks) {
                cylinderCount += bank->getCylinderCount();
            }

            std::set<ExhaustSystemNode *> exhaustSystems;
            std::set<IntakeNode *> intakes;
            for (const CylinderBankNode *bank : m_cylinderBanks) {
                const int n = bank->getCylinderCount();
                for (int i = 0; i < n; ++i) {
                    exhaustSystems.insert(bank->getCylinder(i).exhaust);
                    intakes.insert(bank->getCylinder(i).intake);
                }
            }

            EngineContext context;
            context.setEngine(engine);

            Engine::Parameters parameters = m_parameters;
            parameters.crankshaftCount = (int)m_crankshafts.size();
            parameters.cylinderBanks = (int)m_cylinderBanks.size();
            parameters.cylinderCount = cylinderCount;
            parameters.exhaustSystemCount = (int)exhaustSystems.size();
            parameters.intakeCount = (int)intakes.size();
            parameters.throttle = m_throttle->generate();
            engine->initialize(parameters);

            {
                int i = 0;
                for (ExhaustSystemNode *exhaust : exhaustSystems) {
                    context.addExhaust(
                        exhaust, engine->getExhaustSystem(i++));
                    exhaust->generate(&context);
                }
            }

            {
                int i = 0;
                for (IntakeNode *intake : intakes) {
                    context.addIntake(
                        intake, engine->getIntake(i++));
                    intake->generate(&context);
                }
            }

            {
                int i = 0;
                for (const CylinderBankNode *bank : m_cylinderBanks) {
                    context.addHead(bank->getCylinderHead(), engine->getHead(i++));
                }
            }

            for (int i = 0; i < parameters.crankshaftCount; ++i) {
                m_crankshafts[i]->generate(engine->getCrankshaft(i), &context);
            }

            for (int i = 0; i < parameters.cylinderBanks; ++i) {
                m_cylinderBanks[i]->indexSlaveJournals(&context);
            }

            int cylinderIndex = 0;
            for (int i = 0; i < parameters.cylinderBanks; ++i) {
                m_cylinderBanks[i]->generate(
                    i,
                    cylinderIndex,
                    engine->getCylinderBank(i),
                    engine->getCrankshaft(0),
                    engine,
                    &context);
                cylinderIndex += m_cylinderBanks[i]->getCylinderCount();
            }

            for (int i = 0; i < parameters.cylinderBanks; ++i) {
                m_cylinderBanks[i]->connectRodAssemblies(&context);
            }

            // All master links must exist before resolving nested rods. The
            // assembly validation above has already rejected master cycles.
            for (int i = 0; i < engine->getCylinderCount(); ++i) {
                ConnectingRod *rod = engine->getConnectingRod(i);
                ConnectingRod *root = rod;
                while (root->getMasterRod() != nullptr) root = root->getMasterRod();
                rod->setCrankshaft(root->getCrankshaft());
            }

            m_ignitionModule->generate(engine, &context);
            
            Function *meanPistonSpeedToTurbulence = new Function;
            engine->ownFunction(meanPistonSpeedToTurbulence);
            meanPistonSpeedToTurbulence->initialize(30, 1);
            for (int i = 0; i < 30; ++i) {
                const double s = (double)i;
                meanPistonSpeedToTurbulence->addSample(s, s * 0.5);
            }

            Fuel *fuel = engine->getFuel();
            m_fuel->generate(fuel, &context);

            CombustionChamber::Parameters ccParams;
            ccParams.CrankcasePressure = units::pressure(1.0, units::atm);
            ccParams.Fuel = fuel;
            ccParams.StartingPressure = units::pressure(1.0, units::atm);
            ccParams.StartingTemperature = units::celcius(25.0);
            ccParams.MeanPistonSpeedToTurbulence = meanPistonSpeedToTurbulence;
            ccParams.thermal = m_parameters.thermal;

            for (int i = 0; i < engine->getCylinderCount(); ++i) {
                ccParams.Piston = engine->getPiston(i);
                ccParams.Head = engine->getHead(ccParams.Piston->getCylinderBank()->getIndex());
                engine->getChamber(i)->initialize(ccParams);
            }
        }

        void addCrankshaft(CrankshaftNode *crankshaft) {
            m_crankshafts.push_back(crankshaft);
        }

        void addCylinderBank(CylinderBankNode *bank) {
            m_cylinderBanks.push_back(bank);
        }

        int getIgnitionModuleCount() const {
            return m_ignitionModule == nullptr
                ? 0
                : 1;
        }

        void addIgnitionModule(IgnitionModuleNode *ignitionModule) {
            m_ignitionModule = ignitionModule;
        }

    protected:
        virtual void registerInputs() {
            addInput("name", &m_parameters.name);
            addInput("starter_torque", &m_parameters.starterTorque);
            addInput("starter_speed", &m_parameters.starterSpeed);
            addInput("dyno_min_speed", &m_parameters.dynoMinSpeed);
            addInput("dyno_max_speed", &m_parameters.dynoMaxSpeed);
            addInput("dyno_hold_step", &m_parameters.dynoHoldStep);
            addInput("thermal_model", &m_parameters.thermal.enabled);
            addInput("wall_temperature", &m_parameters.thermal.initialWallTemperature);
            addInput("coolant_temperature", &m_parameters.thermal.coolantTemperature);
            addInput("wall_heat_capacity", &m_parameters.thermal.wallHeatCapacity);
            addInput("coolant_conductance", &m_parameters.thermal.coolantConductance);
            addInput("heat_transfer_scale", &m_parameters.thermal.heatTransferScale);
            addInput("redline", &m_parameters.redline);
            addInput("fuel", &m_fuel, InputTarget::Type::Object);
            addInput("throttle", &m_throttle, InputTarget::Type::Object);
            addInput("simulation_frequency", &m_parameters.initialSimulationFrequency);
            addInput("hf_gain", &m_parameters.initialHighFrequencyGain);
            addInput("jitter", &m_parameters.initialJitter);
            addInput("noise", &m_parameters.initialNoise);

            ObjectReferenceNode<EngineNode>::registerInputs();
        }

        virtual void _evaluate() {
            setOutput(this);

            // Read inputs
            readAllInputs();
        }

        ThrottleNode *m_throttle = nullptr;
        IgnitionModuleNode *m_ignitionModule = nullptr;
        FuelNode *m_fuel = nullptr;

        Engine::Parameters m_parameters;
        std::vector<CrankshaftNode *> m_crankshafts;
        std::vector<CylinderBankNode *> m_cylinderBanks;
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_ENGINE_NODE_H */
