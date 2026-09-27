#include "utils.h"
#include <barrett/systems.h>
#include <barrett/units.h>
#include <barrett/systems/pid_controller.h>

using namespace barrett;
namespace fs = boost::filesystem;

void print_leader_banner(const TeleopConfig& config) {
    const char* barrett_cfg = std::getenv("BARRETT_CONFIG_FILE");
    int barrett_port = read_can_port();

    std::vector<double> sync_pos  = config.leader.sync_pos;
    bool vertical = config.leader.vertical;
    bool on_leader = config.policy.on_leader;

    printf("\n========================================\n");
    printf("  LEADER \n");
    printf("========================================\n");
    printf("  Sync pos      : [");
    for (int i = 0; i < 7; ++i)
        printf("%s%.4f", i ? ", " : "", sync_pos[i]);
    printf("]\n");
    printf("  Vertical      : %s\n",   vertical ? "yes" : "no");
    printf("]\n");
    printf("  config    : %s\n", barrett_cfg ? barrett_cfg : "(BARRETT_CONFIG_FILE not set)");
    printf("  CAN port  : %d\n",   barrett_port);
    printf("  policy on leader  : %s\n",   on_leader ? "yes" : "no");
    printf("========================================\n\n");
}

void print_follower_banner(const TeleopConfig& config) {
    const char* barrett_cfg = std::getenv("BARRETT_CONFIG_FILE");
    int barrett_port = read_can_port();

    std::vector<double> sync_pos  = config.follower.sync_pos;
    bool vertical = config.follower.vertical;
    bool on_follower = config.policy.on_follower;
  
    printf("\n========================================\n");
    printf("  FOLLOWER \n");
    printf("========================================\n");
    printf("  Sync pos      : [");
    for (int i = 0; i < 7; ++i)
        printf("%s%.4f", i ? ", " : "", sync_pos[i]);
    printf("]\n");
    printf("  Vertical      : %s\n",   vertical ? "yes" : "no");
    printf("  Scales        : [");
    for (size_t i = 0; i < config.sync_mapping.scales.size(); ++i)
        printf("%s%.3f", i ? ", " : "", config.sync_mapping.scales[i]);
    printf("]\n");
    printf("  Offsets       : [");
    for (size_t i = 0; i < config.sync_mapping.offsets.size(); ++i)
        printf("%s%.3f", i ? ", " : "", config.sync_mapping.offsets[i]);
    printf("]\n");
    printf("  config    : %s\n", barrett_cfg ? barrett_cfg : "(BARRETT_CONFIG_FILE not set)");
    printf("  CAN port  : %d\n",   barrett_port);
    printf("  policy on follower  : %s\n",   on_follower ? "yes" : "no");
    printf("========================================\n\n");
}

int read_can_port() {
    const char* cfg_path = std::getenv("BARRETT_CONFIG_FILE");
    std::ifstream f(cfg_path);
    std::string line;
    while (std::getline(f, line)) {
        int port;
        if (sscanf(line.c_str(), " port = %d", &port) == 1)
            return port;
    }
    return -1;
}

std::string get_teleop_config_directory() {
    //try in order of priority
    // try environment variable TELEOP_CONFIG_DIR
    const char* config_dir_env = std::getenv("TELEOP_CONFIG_DIR");
    if (config_dir_env) {
        fs::path config_path(config_dir_env);
        if (fs::exists(config_path) && fs::is_directory(config_path)) {
            return config_path.string();
        } else {
            std::cerr << "Environment variable TELEOP_CONFIG_DIR is set, but the directory doesn't exist or is "
                         "invalid.\n";
        }
    }

    // try ~/.config/teleop
    fs::path user_config_dir = fs::path(getenv("HOME")) / ".config" / "teleop";
    if (fs::exists(user_config_dir) && fs::is_directory(user_config_dir)) {
        return user_config_dir.string();
    }

    // try /etc/teleop
    fs::path system_config_dir = "/etc/teleop";
    if (fs::exists(system_config_dir) && fs::is_directory(system_config_dir)) {
        return system_config_dir.string();
    }

    std::cerr << "No valid configuration directory found.\n";
    return "";
}

template <size_t DOF, typename Controller>
void apply_gains(Controller& controller, const PolicyGains& gains) {
    typename Controller::unitless_type kp, ki, kd, int_limit, cs_limit;
    
    for (int i = 0; i < DOF; ++i) {
        kp[i] = gains.kp[i];
        ki[i] = gains.ki[i];
        kd[i] = gains.kd[i];
        int_limit[i] = gains.integrator_limit[i];
        cs_limit[i] = gains.control_signal_limit[i];
    }
    
    controller.setKp(kp);
    controller.setKi(ki);
    controller.setKd(kd);
    controller.setIntegratorLimit(int_limit);
    controller.setControlSignalLimit(cs_limit);
}

BARRETT_UNITS_TEMPLATE_TYPEDEFS(7);
template void apply_gains<7, systems::PIDController<jp_type, jt_type>>(
    systems::PIDController<jp_type, jt_type>&, const PolicyGains&);
template void apply_gains<7, systems::PIDController<jt_type, jt_type>>(
    systems::PIDController<jt_type, jt_type>&, const PolicyGains&);
