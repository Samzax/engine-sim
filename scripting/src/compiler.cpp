#include "../include/compiler.h"

es_script::Compiler::Output *es_script::Compiler::s_output = nullptr;

es_script::Compiler::Compiler() {
    m_compiler = nullptr;
}

es_script::Compiler::~Compiler() {
    assert(m_compiler == nullptr);
}

es_script::Compiler::Output *es_script::Compiler::output() {
    if (s_output == nullptr) {
        s_output = new Output;
    }

    return s_output;
}

void es_script::Compiler::initialize(const std::string &libraryPath) {
    m_compiler = new piranha::Compiler(&m_rules);
    m_compiler->setFileExtension(".mr");

    if (!libraryPath.empty()) m_compiler->addSearchPath(libraryPath);
    m_compiler->addSearchPath("../../es/");
    m_compiler->addSearchPath("../es/");
    m_compiler->addSearchPath("es/");

    m_rules.initialize();
}

bool es_script::Compiler::compile(const piranha::IrPath &path) {
    bool successful = false;

    std::ofstream file("error_log.log", std::ios::out);
    piranha::IrCompilationUnit *unit = m_compiler->compile(path);
    if (unit == nullptr) {
        file << "Can't find file: " << path.toString() << "\n";
    }
    else {
        const piranha::ErrorList *errors = m_compiler->getErrorList();
        if (errors->getErrorCount() == 0) {
            unit->build(&m_program);

            m_program.initialize();

            successful = true;
        }
        else {
            for (int i = 0; i < errors->getErrorCount(); ++i) {
                printError(errors->getCompilationError(i), file);
            }
        }
    }

    file.close();

    return successful;
}

es_script::Compiler::Output es_script::Compiler::execute() {
    // Previous outputs belong to the application. Never return stale pointers
    // if this execution fails or does not set an engine.
    *output() = Output{};
    const bool result = m_program.execute();

    if (!result || output()->engine == nullptr) {
        std::ofstream file("error_log.log", std::ios::app);
        if (!result) {
            file << "Script runtime error: " << m_program.getRuntimeError() << '\n';
            const piranha::Node *node = m_program.getErrorNode();
            const auto printLocation = [&file](const piranha::IrParserStructure *source) {
                if (source != nullptr && source->getParentUnit() != nullptr) {
                    file << "       At " << source->getParentUnit()->getPath().toString()
                        << '(' << source->getSummaryToken()->lineStart << ")\n";
                }
            };
            if (node != nullptr) {
                printLocation(node->getIrStructure());
                for (auto *context = node->getContext(); context != nullptr; context = context->getParent()) {
                    printLocation(context->getContext());
                }
            }
        }
        file << "Script execution failed or did not produce an engine. The current engine was not replaced.\n";
    }
    output()->success = result && output()->engine != nullptr;
    return *output();
}

void es_script::Compiler::destroy() {
    m_program.free();
    m_compiler->free();

    delete m_compiler;
    m_compiler = nullptr;
    delete s_output;
    s_output = nullptr;
}

void es_script::Compiler::printError(
    const piranha::CompilationError *err,
    std::ofstream &file) const
{
    const piranha::ErrorCode_struct &errorCode = err->getErrorCode();
    file << err->getCompilationUnit()->getPath().toString()
        << "(" << err->getErrorLocation()->lineStart << "): error "
        << errorCode.stage << errorCode.code << ": " << errorCode.info << std::endl;

    piranha::IrContextTree *context = err->getInstantiation();
    while (context != nullptr) {
        piranha::IrNode *instance = context->getContext();
        if (instance != nullptr) {
            const std::string instanceName = instance->getName();
            const std::string definitionName = (instance->getDefinition() != nullptr)
                ? instance->getDefinition()->getName()
                : "<Type Error>";
            const std::string formattedName = (instanceName.empty())
                ? "<unnamed> " + definitionName
                : instanceName + " " + definitionName;

            file
                << "       While instantiating: "
                << instance->getParentUnit()->getPath().toString()
                << "(" << instance->getSummaryToken()->lineStart << "): "
                << formattedName << std::endl;
        }

        context = context->getParent();
    }
}
