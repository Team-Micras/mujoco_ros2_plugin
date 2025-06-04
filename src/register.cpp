#include <mujoco/mjplugin.h>

#include "ros2_plugin.hpp"

namespace mujoco::plugin::ros2 {
mjPLUGIN_LIB_INIT {
    Ros2Plugin::register_plugin();
}

}  // namespace mujoco::plugin::ros2
