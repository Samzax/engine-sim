#ifndef ATG_ENGINE_SIM_COMPILER_H
#define ATG_ENGINE_SIM_COMPILER_H

#include "language_rules.h"

#include "engine_sim.h"
#include "piranha.h"

#include <vector>

namespace es_script {

    class Compiler {
    public:
        struct Output {
            bool success = false;
            Engine *engine = nullptr;
            Vehicle *vehicle = nullptr;
            Transmission *transmission = nullptr;
            Simulator::Parameters simulatorParameters;
            ApplicationSettings applicationSettings;

            std::vector<Function *> functions;
        };

    private:
        static Output *s_output;

    public:
        Compiler();
        ~Compiler();

        static Output *output();

        void initialize(const std::string &libraryPath = "");
        bool compile(const piranha::IrPath &path);
        Output execute();
        void destroy();

    private:
        void printError(const piranha::CompilationError *err, std::ofstream &file) const;

    private:
        class Program : public piranha::NodeProgram {
        public:
            piranha::Node *getErrorNode() const { return m_errorNode; }
        };

        LanguageRules m_rules;
        piranha::Compiler *m_compiler;
        Program m_program;
    };

} /* namespace es_script */

#endif /* ATG_ENGINE_SIM_COMPILER_H */
