#pragma once

#include <atomic>
#include <cstddef>
#include <vector>

#include <barrett/units.h>

#include "../utils/data_packets.h"

enum class ModuleRole { Leader, Follower };

// Everything a module may need for one control cycle.
template <size_t DOF>
struct ControlContext {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

    TeleopData<DOF>* st = nullptr;

    const jt_teleop_type* ref_ext_torque = nullptr; // their external torque (mapped into our frame)
    const jt_teleop_type* cur_ext_torque = nullptr; // our external torque
    const jt_teleop_type* cur_dyn = nullptr;        // estimated dynamics feed-forward
    const jt_teleop_type* cur_grav = nullptr;       // gravity feed-forward
    const jp_teleop_type* cur_pos = nullptr;        // our joint positions

    bool cancel_policy = false;
};

template <size_t DOF>
class Module {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    virtual ~Module() = default;

    virtual char key() const = 0;
    virtual const char* name() const = 0;

    bool isLoaded() const { return loaded_.load(); }
    void load() {
        if (!loaded_.load()) {
            loaded_.store(true);
            onLoad();
        }
    }
    void unload() {
        if (loaded_.load()) {
            onUnload();
            loaded_.store(false);
        }
    }
    void toggle() {
        if (isLoaded()) {
            unload();
        } else {
            load();
        }
    }

    // Torque-producing modules override these.
    virtual bool producesTorque() const { return false; }
    virtual jt_teleop_type torque(const ControlContext<DOF>&) { return jt_teleop_type::Zero(); }

    // Control-loop hooks, only called while the module is loaded.
    virtual void receive(ControlContext<DOF>&, TeleopData<DOF>&) {}
    virtual void send(TeleopData<DOF>&) {}
    virtual void update(const ControlContext<DOF>&) {}

  protected:
    virtual void onLoad() {}
    virtual void onUnload() {}

    std::atomic<bool> loaded_{false};
};

template <size_t DOF>
class ModuleManager {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  public:
    void add(Module<DOF>* module) { modules_.push_back(module); }

    Module<DOF>* find(char key) const {
        for (Module<DOF>* m : modules_) {
            if (m->key() == key) return m;
        }
        return nullptr;
    }

    bool toggle(char key) {
        Module<DOF>* m = find(key);
        if (m == nullptr) return false;
        m->toggle();
        return true;
    }

    // The whole of the control law: sum the torques of every loaded module.
    jt_teleop_type sumTorque(const ControlContext<DOF>& ctx) const {
        jt_teleop_type u = jt_teleop_type::Zero();
        for (Module<DOF>* m : modules_) {
            if (m->isLoaded() && m->producesTorque()) {
                u += m->torque(ctx);
            }
        }
        return u;
    }

  private:
    std::vector<Module<DOF>*> modules_;
};
