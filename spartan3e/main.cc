#include "backend.h"
#include "command.h"
#include "log.h"
#include "ucf.h"
#include <filesystem>

USING_NEXTPNR_NAMESPACE

class Spartan3eCommandHandler : public CommandHandler
{
  public:
    Spartan3eCommandHandler(int argc, char **argv) : CommandHandler(argc, argv) {}
    void setupArchContext(Context *ctx) override {}
    void customBitstream(Context *ctx) override {}
    void customAfterLoad(Context *ctx) override
    {
        if (vm.count("ucf"))
            read_ucf(ctx, vm["ucf"].as<std::string>());
    }
    po::options_description getArchOptions() override
    {
        po::options_description options("Spartan-3E options");
        options.add_options()("chipdb", po::value<std::string>(), "device database (required)")(
                "output", po::value<std::string>(), "output routed physical design")("ucf", po::value<std::string>(),
                                                                                     "input supported UCF constraints")(
                "annotated-netlist", po::value<std::string>(), "optional diagnostic netlist export");
        return options;
    }
    void validateInputs()
    {
        if (vm.count("write"))
            log_error("Use --annotated-netlist for diagnostic output, not --write.\n");
        if (!vm.count("chipdb"))
            log_error("The --chipdb option is required.\n");
        if (vm.count("test"))
            return;
        if (!vm.count("json"))
            log_error("The --json option is required.\n");
        bool partial = vm.count("pack-only") || vm.count("no-place") || vm.count("no-route");
        if (partial && vm.count("output"))
            log_error("--output requires complete placement and routing.\n");
        if (!partial && !vm.count("output"))
            log_error("The --output option is required for routed physical design.\n");
        for (const auto &output : {"output", "annotated-netlist"}) {
            if (!vm.count(output))
                continue;
            namespace fs = std::filesystem;
            auto path = vm[output].as<std::string>();
            std::error_code ec;
            auto status = fs::symlink_status(path, ec);
            if (ec && ec != std::errc::no_such_file_or_directory)
                log_error("Cannot inspect --%s destination %s: %s\n", output, path.c_str(), ec.message().c_str());
            if (fs::exists(status) && !fs::is_regular_file(status))
                log_error("--%s destination must be a regular file, not a directory or symlink.\n", output);
            auto target = fs::weakly_canonical(path, ec);
            if (ec)
                log_error("Cannot resolve --%s destination %s: %s\n", output, path.c_str(), ec.message().c_str());
            for (const auto &input : {"chipdb", "json", "ucf"}) {
                if (!vm.count(input))
                    continue;
                auto source = fs::weakly_canonical(vm[input].as<std::string>(), ec);
                if (ec)
                    log_error("Cannot resolve --%s input: %s\n", input, ec.message().c_str());
                if (target == source)
                    log_error("--%s would overwrite --%s input.\n", output, input);
            }
            if (std::string(output) == "annotated-netlist" && vm.count("output")) {
                auto physical = fs::weakly_canonical(vm["output"].as<std::string>(), ec);
                if (ec)
                    log_error("Cannot resolve physical output: %s\n", ec.message().c_str());
                if (target == physical)
                    log_error("Physical and annotated-netlist outputs must be distinct.\n");
            }
        }
        if (vm.count("annotated-netlist"))
            vm.emplace("write", vm.at("annotated-netlist"));
    }
    std::unique_ptr<Context> createContext(dict<std::string, Property> &values) override
    {
        validateInputs();
        auto architecture = values.find("arch.name");
        if (architecture != values.end() && architecture->second.as_string() != "spartan3e")
            log_error("Unsupported architecture '%s'.\n", architecture->second.as_string().c_str());
        ArchArgs args;
        auto ctx = std::make_unique<Context>(args);
        ctx->backend =
                make_backend(vm["chipdb"].as<std::string>(), vm.count("output") ? vm["output"].as<std::string>() : "");
        ctx->backend->with_gui = vm.count("gui") != 0;
        ctx->backend->init(ctx.get());
        return ctx;
    }
};

int main(int argc, char **argv) { return Spartan3eCommandHandler(argc, argv).exec(); }
