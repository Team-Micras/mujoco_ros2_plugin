<!-- markdownlint-disable -->
<div align="center">

![micras_mujoco_ros2_plugin](https://github.com/user-attachments/assets/bc836e9b-fbb2-450f-8c56-2a33d2e1c9a2)

MuJoCo Plugin for integrating with ROS 2

</div>

<p align="center">
<a href="https://cplusplus.com/"><img alt="Made with C++" src="https://img.shields.io/badge/made_with-c%2B%2B-blue?style=for-the-badge&labelColor=ef4041&color=c1282d" height="30"></a>
<a href="https://docs.ros.org/en/jazzy/index.html"><img alt="ROS Jazzy" src="https://img.shields.io/badge/ROS_version-jazzy-green?style=for-the-badge&labelColor=2e8b57&color=228b22" height="30"></a>
<a href="https://mujoco.org/"><img alt="MuJoCo" src="https://img.shields.io/badge/built_for-MuJoCo-blue?style=for-the-badge&labelColor=4169e1&color=1e90ff" height="30"></a>

<!-- markdownlint-restore -->

## 📑 Summary

- [📑 Summary](#-summary)
- [🔌 Plugin Overview](#-plugin-overview)
- [🚀 Features](#-features)
- [🛠 Prerequisites](#-prerequisites)
- [⚙️ Installation](#️-installation)
  - [Building from Source](#building-from-source)
  - [Registering the plugin](#registering-the-plugin)
- [▶️ Usage](#️-usage)
  - [Loading the Plugin in MJCF](#loading-the-plugin-in-mjcf)
  - [Plugin Configuration](#plugin-configuration)
- [🔄 ROS 2 Interface](#-ros-2-interface)
  - [⬅️ Published Topics](#️-published-topics)
    - [Sensor Data](#sensor-data)
    - [Simulation Clock](#simulation-clock)
  - [➡️ Subscribed Topics (Actuators)](#️-subscribed-topics-actuators)
- [💡 Example](#-example)
- [👥 Contributing](#-contributing)
  - [💬 Git commit messages](#-git-commit-messages)
  - [🔀 Git workflow](#-git-workflow)
- [✨ Contributors](#-contributors)

## 🔌 Plugin Overview

**A ROS 2 Plugin for the MuJoCo Simulator, enabling seamless integration for robotics simulation and control.**

This plugin acts as a bridge, allowing sensor data to flow from MuJoCo to ROS 2, and actuator commands from ROS 2 back into your MuJoCo simulation. It's designed to be straightforward to use and configure, facilitating rapid development and testing of robotic systems.

<!-- markdownlint-disable -->
<div align="center">

![mujoco_ros2_plugin](https://github.com/user-attachments/assets/b4f3b565-e612-43b9-949b-438f973df0df)

Mujoco ROS 2 topics plotted on [PlotJuggler](https://github.com/facontidavide/PlotJuggler)

</div>
<!-- markdownlint-restore -->

## 🚀 Features

- **ROS 2 Node Integration**: Automatically initializes and manages a dedicated ROS 2 node.
- **Sensor Data Publication**:
  - Publishes data from all MuJoCo sensors defined in your model.
  - Supports single-value sensors (`example_interfaces::msg::Float64`).
  - Supports multi-dimensional sensors (`example_interfaces::msg::Float64MultiArray`).
- **Simulation Clock Publication**:
  - Publishes MuJoCo simulation time as `builtin_interfaces::msg::Time` messages.
  - Enables time synchronization between MuJoCo simulation and ROS 2 nodes.
- **Actuator Command Subscription**:
  - Subscribes to ROS 2 topics for controlling MuJoCo actuators.
  - Accepts `example_interfaces::msg::Float64` for actuator inputs.
- **Configurable**:
  - Set a custom ROS 2 namespace for all topics (default: `"mujoco/"`).
  - Adjust the reliability param of the QoS for the actuators subscribers (default: `best_effort`).
- **Passive MuJoCo Plugin**: Operates within MuJoCo's simulation loop for timely data exchange.
- **Easy to Build & Install**: Standard CMake build process.

## 🛠 Prerequisites

Ensure the following are installed and configured on your system:

- **MuJoCo Physics Simulator**:
  - The build system tries to find MuJoCo via the `simulate` program in your `PATH`.
  - Alternatively, set the `MUJOCO_PATH` environment variable to your MuJoCo `bin` directory (e.g., `~/.mujoco/mujoco-X.Y.Z/bin`).
- **ROS 2**: A C++20 compatible ROS 2 distribution (e.g., Jazzy, Humble).
  - Source your ROS 2 environment: `source /opt/ros/<your_ros_distro>/setup.bash`
  - Required ROS 2 packages: `rclcpp`, `example_interfaces`, `ament_cmake`.
- **Build Tools**:
  - `cmake` (version 3.16+)
  - C++20 compliant compiler (e.g., GCC, Clang)

## ⚙️ Installation

### Building from Source

1. **Set `MUJOCO_PATH` (if needed)**:
   If MuJoCo isn't found automatically at your `PATH`, export the `MUJOCO_PATH`:

   ```bash
   export MUJOCO_PATH="/path/to/your/mujoco/bin" # e.g., $HOME/.mujoco/mujoco-3.1.3/bin
   ```

2. **Compile the Plugin**:

   ```bash
   cmake -B build
   cmake --build build
   ```

3. **Install the Plugin Library**:

   ```bash
   cmake --install build
   ```

   This command installs the plugin (e.g., `libros2_plugin.so`) to `${MUJOCO_PATH}/mujoco_plugin/`. This path is typically searched by MuJoCo by default.

### Registering the plugin

MuJoCo needs to register the plugin for every simulation, the installation step usually places it in a standard location so that it can be automatically registered. If you install it elsewhere, or MuJoCo cannot find it, you might need to:

- **Option 1:** Move the library output file manually to the `mujoco_plugin` folder located at the same directory as the mujoco executable.
- **Option 2:** Register the plugin manually by calling the `Ros2Plugin::register_plugin()` function before the mujoco simulate loop.

## ▶️ Usage

### Loading the Plugin in MJCF

To activate the plugin, add an `<extension>` block to your MJCF model file. The plugin is identified by `mujoco.ros2`.

```xml
<mujoco>
  <extension>
    <plugin plugin="mujoco.ros2">
      <instance name="ros2_plugin">
        <!-- Optional: Configuration parameters -->
        <!-- <config key="ros_namespace" value="mujoco/"/> -->
        <!-- <config key="subscribers_reliability" value="best_effort"/> -->
      </instance>
    </plugin>
  </extension>

  <!-- ... rest of your MJCF model ... -->
</mujoco>
```

### Plugin Configuration

Configure the plugin directly within the MJCF `<plugin>` tag:

- **`ros_namespace`** (string, default: `"mujoco/"`):
  Sets the ROS 2 namespace for all topics. *Ensure it ends with a `/` for correct namespacing.*

  ```xml
  <config key="ros_namespace" value="custom_namespace/"/>
  ```

- **`subscribers_reliability`** (string, default: `best_effort`):
  Sets the reliability of the ROS subscribers QoS.

  ```xml
  <config key="subscribers_reliability" value="reliable"/>
  ```

## 🔄 ROS 2 Interface

The plugin establishes the following ROS 2 communication channels:

### ⬅️ Published Topics

#### Sensor Data

Data from sensors defined in the `<sensor>` section of your MJCF will be published.

- **Scalar Sensors**:
  - Topic: `<ros_namespace>sensors/<sensor_name>`
  - Type: `example_interfaces::msg::Float64`
- **Array Sensors**:
  - Topic: `<ros_namespace>sensors/<sensor_name>`
  - Type: `example_interfaces::msg::Float64MultiArray`

Where `<sensor_name>` is the `name` attribute from the MJCF.

#### Simulation Clock

The plugin publishes the MuJoCo simulation time for synchronization with ROS 2 nodes.

- **Clock Topic**:
  - Topic: `<ros_namespace>clock`
  - Type: `builtin_interfaces::msg::Time`
  - Contains the current MuJoCo simulation time in seconds and nanoseconds.

### ➡️ Subscribed Topics (Actuators)

Commands for actuators defined in the `<actuator>` section of your MJCF can be sent via these topics.

- Topic: `<ros_namespace>actuators/<actuator_name>/command`
- Type: `example_interfaces::msg::Float64`
  - The `data` field of the message is applied to `mjData->ctrl[]` for the respective actuator.

Where `<actuator_name>` is the `name` attribute from the MJCF.

## 💡 Example

Consider this MJCF snippet for a simple car:

```xml
<mujoco>
  <extension>
    <plugin plugin="mujoco.ros2">
    <instance name="ros2_plugin">
      <config key="ros_namespace" value="simple_car/"/>
    </instance>
    </plugin>
  </extension>

  <asset>
    <mesh name="chasis" scale=".01 .006 .0015"
      vertex=" 9   2   0
              -10  10  10
               9  -2   0
               10  3  -10
               10 -3  -10
              -8   10 -10
              -10 -10  10
              -8  -10 -10
              -5   0   20"/>
  </asset>

  <default>
    <joint damping=".03" actuatorfrcrange="-0.5 0.5"/>
    <default class="wheel">
      <geom type="cylinder" size=".03 .01" rgba=".5 .5 1 1"/>
    </default>
  </default>

  <worldbody>
    <geom type="plane" size="3 3 .01"/>

    <body name="car" pos="0 0 .03">
      <freejoint/>
      <geom name="chasis" type="mesh" mesh="chasis"/>
      <geom name="front wheel" pos=".08 0 -.015" type="sphere" size=".015" condim="1" priority="1"/>
      <body name="left wheel" pos="-.07 .06 0" zaxis="0 1 0">
        <joint name="left_wheel"/>
        <geom class="wheel"/>
      </body>
      <body name="right_wheel" pos="-.07 -.06 0" zaxis="0 1 0">
        <joint name="right_wheel"/>
        <geom class="wheel"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="left_motor" joint="left_wheel" ctrlrange="-1 1"/>
    <motor name="right_motor" joint="right_wheel" ctrlrange="-1 1"/>
  </actuator>

  <sensor>
    <jointactuatorfrc name="left_torque" joint="left_wheel"/>
    <jointactuatorfrc name="right_torque" joint="right_wheel"/>
  </sensor>
</mujoco>

```

**This would result in:**

- **Publishers**:
  - `/simple_car/clock` (`Time`)
  - `/simple_car/sensors/left_torque` (`Float64`)
  - `/simple_car/sensors/right_torque` (`Float64`)
- **Subscribers**:
  - `/simple_car/actuators/left_motor/command` (`Float64`)
  - `/simple_car/actuators/right_motor/command` (`Float64`)

Interact using ROS 2 tools:

```bash
# Listen to simulation clock
ros2 topic echo /simple_car/clock

# Listen to a sensor
ros2 topic echo /simple_car/sensors/left_torque

# Send a command to an actuator
ros2 topic pub /simple_car/actuators/right_motor/command example_interfaces/msg/Float64 '{data: 0.5}' --once
```

## 👥 Contributing

To learn how to contribute to the project, see the following contribution guidelines.

### 💬 Git commit messages

- Use the present tense ("Add feature" not "Added feature").
- Use the imperative mood ("Move cursor to..." not "Moves cursor to...").
- It is strongly recommended to start a commit message with a related emoji:
  - 📝 `:memo:` for documentation.
  - 🐛 `:bug:` for bug issues.
  - 🚑 `:ambulance:` for critical fixes.
  - 🎨 `:art:` for formatting code.
  - ✨ `:sparkles:` for new features.

  For more examples, see [this reference](https://gitmoji.dev/).

### 🔀 Git workflow

The project workflow is based on [GitHub Flow](https://docs.github.com/en/get-started/using-github/github-flow).

## ✨ Contributors

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tr>
    <td align="center"><a href="https://github.com/GabrielCosme"><img src="https://avatars.githubusercontent.com/u/62270066?v=4?s=100" width="100px;" alt="Gabriel Cosme Barbosa"/><br/><sub><b>Gabriel Cosme Barbosa</b></sub></a><br/><a href="https://github.com/Team-Micras/mujoco_ros2_plugin/commits?author=GabrielCosme" title="Code">💻</a> <a href="https://github.com/Team-Micras/mujoco_ros2_plugin/commits?author=GabrielCosme" title="Documentation">📖</a> <a href="#research-GabrielCosme" title="Research">🔬</a> <a href="https://github.com/Team-Micras/mujoco_ros2_plugin/pulls?q=is%3Apr+reviewed-by%3AGabrielCosme" title="Reviewed Pull Requests">👀</a></td>
    <td align="center"><a href="https://github.com/PedroDeSanti"><img src="https://avatars.githubusercontent.com/u/62271285?v=4" width="100px;" alt="Pedro de Santi"/><br/><sub><b>Pedro de Santi</b></sub></a><br/><a href="https://github.com/Team-Micras/mujoco_ros2_plugin/commits?author=PedroDeSanti" title="Code">💻</a> <a href="https://github.com/Team-Micras/mujoco_ros2_plugin/commits?author=PedroDeSanti" title="Documentation">📖</a> <a href="#research-PedroDeSanti" title="Research">🔬</a> <a href="https://github.com/Team-Micras/mujoco_ros2_plugin/pulls?q=is%3Apr+reviewed-by%3APedroDeSanti" title="Reviewed Pull Requests">👀</a></td>
  </tr>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!
