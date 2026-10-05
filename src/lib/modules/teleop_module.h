#pragma once

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include <boost/optional.hpp>

#include "module.h"
#include "../utils/config_loader.h"

// pass on other robot information and sync with jp
// The link itself is always running so both sides can compare positions before linking
template <size_t DOF, typename Link>
class TeleopModule : public Module<DOF> {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    TeleopModule(ModuleRole role, const Config& config, TeleopState<DOF>* state,
                 std::unique_ptr<Link> link)
        : role_(role), config_(config), state_(state), link_(std::move(link)) {
    }

    char key() const override { return 'l'; }
    const char* name() const override { return "teleop"; }

    bool isLinked() const { return this->isLoaded(); }
    void tryLink() { this->load(); }
    void unlink() { this->unload(); }

    // both robots must be within radian tolerance when linking
    bool theirIsNear(double tolerance) {
        if (!have_their_ || state_ == nullptr) return false;
        return state_->with_lock([this, tolerance](TeleopData<DOF>& st) {
            const jp_teleop_type& ours = (role_ == ModuleRole::Leader) ? st.leader_jp : st.follower_jp;
            const jp_teleop_type& theirs = (role_ == ModuleRole::Leader) ? st.follower_jp : st.leader_jp;
            return (ours - theirs).cwiseAbs().maxCoeff() <= tolerance;
        });
    }

    void receive(ControlContext<DOF>& ctx, TeleopData<DOF>& st) override {
        (void)ctx;

        const uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::steady_clock::now().time_since_epoch())
                                    .count();
        const uint64_t timeout_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(TIMEOUT_DURATION).count();

        boost::optional<std::vector<uint8_t>> received = link_->getLatestTeleopReceived();
        const bool got_packet = received && setStateFromLatestPacket(st, *received);
        if (got_packet && now_ns >= st.timestamp && now_ns - st.timestamp <= timeout_ns) {
            mirrorAndOffset(st);
            have_their_ = true;
            return;
        }

        // teleop defaults
        if (role_ == ModuleRole::Leader) {
            setLocalStateValue(st.leader_jp, st.follower_jp);
            setLocalStateValue(st.leader_jv, st.follower_jv);
            setLocalStateValue(st.leader_cart_pos, st.follower_cart_pos);
            setLocalStateValue(st.leader_quat, st.follower_quat);
            setLocalStateValue(jt_teleop_type::Zero(), st.follower_dyngravcomp_torque);
            setLocalStateValue(jt_teleop_type::Zero(), st.environment_torque);
            setLocalStateValue(jt_teleop_type::Zero(), st.filtered_environment_torque);
        } else {
            setLocalStateValue(st.follower_jp, st.leader_jp);
            setLocalStateValue(st.follower_jv, st.leader_jv);
            setLocalStateValue(st.follower_cart_pos, st.leader_cart_pos);
            setLocalStateValue(st.follower_quat, st.leader_quat);
            setLocalStateValue(jt_teleop_type::Zero(), st.leader_dyngravcomp_torque);
            setLocalStateValue(jt_teleop_type::Zero(), st.human_torque);
            setLocalStateValue(jt_teleop_type::Zero(), st.filtered_human_torque);
        }

        have_their_ = false;

        if (isLinked()) {
            const double age_ms =
                got_packet ? static_cast<double>(now_ns - st.timestamp) / 1e6 : -1.0;
            std::cout << "lost link with age " << age_ms << " ms" << std::endl;
            this->unload(); // linking is module loading
        }
    }

    void send(TeleopData<DOF>& st) override { link_->send(st); }

  protected:
    void onLoad() override { std::cout << "teleop module linked" << std::endl; }
    void onUnload() override { std::cout << "teleop module unlinked" << std::endl; }

  private:
    // decode the raw packet into the shared state using the recv routing
    bool setStateFromLatestPacket(TeleopData<DOF>& st, const std::vector<uint8_t>& packet) {
        return decode(link_->recvFields(), packet.data(), packet.size(), st);
    }

    // Mirror and offset some of the WAM joints.
    // NOTE: the follower does the exact opposite of the leader.
    void mirrorAndOffset(TeleopData<DOF>& st) {
        if (role_ == ModuleRole::Leader) {
            for (size_t i = 0; i < TELEOP_DOF; ++i) {
                st.follower_jp[i] = (st.follower_jp[i] - config_.sync_mapping.offsets[i]) / config_.sync_mapping.scales[i];
                st.follower_jv[i] = st.follower_jv[i] / config_.sync_mapping.scales[i];
                st.follower_dyngravcomp_torque[i] = st.follower_dyngravcomp_torque[i] / config_.sync_mapping.scales[i];
                st.environment_torque[i] = st.environment_torque[i] / config_.sync_mapping.scales[i];
                st.filtered_environment_torque[i] = st.filtered_environment_torque[i] / config_.sync_mapping.scales[i];
            }
        } else {
            for (size_t i = 0; i < TELEOP_DOF; ++i) {
                st.leader_jp[i] = st.leader_jp[i] * config_.sync_mapping.scales[i] + config_.sync_mapping.offsets[i];
                st.leader_jv[i] = st.leader_jv[i] * config_.sync_mapping.scales[i];
                st.leader_dyngravcomp_torque[i] = st.leader_dyngravcomp_torque[i] * config_.sync_mapping.scales[i];
                st.human_torque[i] = st.human_torque[i] * config_.sync_mapping.scales[i];
                st.filtered_human_torque[i] = st.filtered_human_torque[i] * config_.sync_mapping.scales[i];
            }
        }
    }

    ModuleRole role_;
    Config config_;
    TeleopState<DOF>* state_;
    std::unique_ptr<Link> link_;

    bool have_their_ = false;

    const std::chrono::milliseconds TIMEOUT_DURATION{20};
};
