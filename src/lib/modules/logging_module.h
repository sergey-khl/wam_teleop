#pragma once

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "module.h"
#include "../utils/config_loader.h"

// print to screen entries in state defined by logging config file. press g to start logging
template <size_t DOF>
class LoggingModule : public Module<DOF> {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    LoggingModule(ModuleRole role, const Config& config)
        : role_(role), config_(config) {}

    char key() const override { return 'g'; }
    const char* name() const override { return "logging"; }

    void onLoad() override { std::cout << "logging module loaded" << std::endl; }
    void onUnload() override { std::cout << "logging module unloaded" << std::endl; }

    void update(const ControlContext<DOF>& ctx) override {
        if (!this->isLoaded()) return;
        const int period = config_.logging.every_n_loops > 0 ? config_.logging.every_n_loops : 1;
        if (++counter_ % static_cast<uint64_t>(period) != 0) return;
        if (ctx.st == nullptr) return;

        const std::vector<std::string>& fields =
            (role_ == ModuleRole::Leader) ? config_.logging.leader : config_.logging.follower;
        if (fields.empty()) return;

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "[LOG " << (role_ == ModuleRole::Leader ? "LEADER" : "FOLLOWER") << "]\n";
        for (const std::string& field : fields) {
            std::cout << "  -> " << field << ": ";
            if (!formatField(std::cout, *ctx.st, field)) {
                std::cout << "<unknown field>";
            }
            std::cout << "\n";
        }
        std::cout << std::flush;
    }

  private:
    ModuleRole role_;
    Config config_;
    uint64_t counter_ = 0;
};
