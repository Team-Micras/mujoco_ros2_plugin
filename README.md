<!-- markdownlint-disable -->
<div align="center">

![micras_mujoco_ros_plugin](https://github.com/user-attachments/assets/bc836e9b-fbb2-450f-8c56-2a33d2e1c9a2)

MuJoCo Plugin for integrating with ROS 2

</div>

<p align="center">
<a href="https://docs.ros.org/en/jazzy/index.html"><img alt="ROS Jazzy" src="https://img.shields.io/badge/ROS_version-jazzy-green?style=for-the-badge&labelColor=2e8b57&color=228b22" height="30"></a>
<a href="https://gazebosim.org/docs/harmonic"><img alt="MuJoCo" src="https://img.shields.io/badge/MuJoCo_version-3.3.2-blue?style=for-the-badge&labelColor=4169e1&color=1e90ff" height="30"></a>
<a href="https://cplusplus.com/"><img alt="Made with C++" src="https://img.shields.io/badge/made_with-c%2B%2B-blue?style=for-the-badge&labelColor=ef4041&color=c1282d" height="30"></a>

<!-- markdownlint-restore -->

## 📑 Summary

- [📑 Summary](#-summary)
- [🔌 Plugin Overview](#-plugin-overview)
- [🚀 Features](#-features)
- [🛠 Prerequisites](#-prerequisites)
- [⚙️ Installation](#️-installation)
  - [Building from Source](#building-from-source)
  - [Setting up MuJoCo Plugin Path](#setting-up-mujoco-plugin-path)
- [▶️ Usage](#️-usage)
  - [Loading the Plugin in MJCF](#loading-the-plugin-in-mjcf)
  - [Plugin Configuration](#plugin-configuration)
- [🔄 ROS 2 Interface](#-ros-2-interface)
  - [⬅️ Published Topics (Sensors)](#️-published-topics-sensors)
  - [➡️ Subscribed Topics (Actuators)](#️-subscribed-topics-actuators)
- [💡 Example](#-example)
- [👥 Contributing](#-contributing)
  - [💬 Git commit messages](#-git-commit-messages)
  - [🔀 Git workflow](#-git-workflow)
- [✨ Contributors](#-contributors)

## 🔌 Plugin Overview

**A ROS 2 Plugin for the MuJoCo Simulator, enabling seamless integration for robotics simulation and control.**

This plugin acts as a bridge, allowing sensor data to flow from MuJoCo to ROS 2, and actuator commands from ROS 2 back into your MuJoCo simulation. It's designed to be straightforward to use and configure, facilitating rapid development and testing of robotic systems.


## 🚀 Features

- **ROS 2 Node Integration**: Automatically initializes and manages a dedicated ROS 2 node (`mujoco_ros2_plugin`).
- **Sensor Data Publication**:
  - Publishes data from all MuJoCo sensors defined in your model.
  - Supports single-value sensors (`example_interfaces::msg::Float64`).
  - Supports multi-dimensional sensors (`example_interfaces::msg::Float64MultiArray`).
- **Actuator Command Subscription**:
  - Subscribes to ROS 2 topics for controlling MuJoCo actuators.
  - Accepts `example_interfaces::msg::Float64` for actuator inputs.
- **Configurable**:
  - Set a custom ROS 2 namespace for all topics (default: `"mujoco/"`).
  - Adjust the queue size for ROS 2 publishers and subscribers (default: `1`).
- **Passive MuJoCo Plugin**: Operates within MuJoCo's simulation loop for timely data exchange.
- **Easy to Build & Install**: Standard CMake build process.


## 🛠 Prerequisites

Ensure the following are installed and configured on your system:

- **MuJoCo Physics Simulator**:
  - The build system tries to find MuJoCo via the `simulate` program in your `PATH`.
  - Alternatively, set the `MUJOCO_PATH` environment variable to your MuJoCo `bin` directory (e.g., `~/.mujoco/mujoco-X.Y.Z/bin`).
- **ROS 2**: A C++20 compatible ROS 2 distribution (e.g., Humble, Iron).
  - Source your ROS 2 environment: `source /opt/ros/<your_ros_distro>/setup.bash`
  - Required ROS 2 packages: `rclcpp`, `example_interfaces`, `ament_cmake`.
- **Build Tools**:
  - `cmake` (version 3.16+)
  - C++20 compliant compiler (e.g., GCC, Clang)

---

## ⚙️ Installation

### Building from Source

1. **Set `MUJOCO_PATH` (if needed)**:
   If MuJoCo isn't found automatically, export the `MUJOCO_PATH`:

   ```bash
   export MUJOCO_PATH="/path/to/your/mujoco/bin" # e.g., $HOME/.mujoco/mujoco-3.1.3/bin
   ```

2. **Compile the Plugin**:

   ```bash
   mkdir build && cd build
   cmake ..
   make -j$(nproc)
   ```

3. **Install the Plugin Library**:

   ```bash
   sudo make install
   ```

   This command installs the plugin (e.g., `libros_plugin.so`) to `${MUJOCO_PATH}/mujoco_plugin/`. This path is typically searched by MuJoCo by default.

Start the build process by creating a build folder inside the project root:

```bash
mkdir build && cd build
```

And then generating the build commands:

```bash
cmake ..
```

The project can then be compiled by running:

```bash
make -j
```

### Setting up MuJoCo Plugin Path

MuJoCo needs to locate the compiled plugin. The `make install` step usually places it in a standard location. If you install it elsewhere, or MuJoCo cannot find it, you might need to:

- **Option 1 (Recommended if installed to standard path):** The default installation path `${MUJOCO_PATH}/mujoco_plugin/` should be automatically discoverable by MuJoCo.
- **Option 2: Use `MUJOCO_PLUGIN_PATH`**:
  Add the directory containing `libros_plugin.so` to this environment variable:

  ```bash
  export MUJOCO_PLUGIN_PATH="/path/to/plugin_directory:${MUJOCO_PLUGIN_PATH}"
  ```

- **Option 3: Copy to Model Directory**: Place the plugin library in the same directory as your MJCF model file.
- **Option 4: Copy to User Plugin Directory**: `~/.mujoco/plugin/` (Linux/macOS) or `C:\Users\<UserName>\.mujoco\plugin\` (Windows).

---

## ▶️ Usage

### Loading the Plugin in MJCF

To activate the plugin, add an `<extension>` block to your MJCF model file. The plugin is identified by `mujoco.ros2`.

```xml
<mujoco>
    <extension>
        <plugin plugin="mujoco.ros2">
            <!-- Optional: Configuration parameters -->
            <!-- <config key="ros_namespace" value="my_robot/"/> -->
            <!-- <config key="topic_queue_size" value="10"/> -->
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

- **`topic_queue_size`** (integer, default: `1`):
  Defines the queue size for ROS 2 publishers and subscribers.

  ```xml
  <config key="topic_queue_size" value="5"/>
  ```

---

## 🔄 ROS 2 Interface

The plugin establishes the following ROS 2 communication channels:

### ⬅️ Published Topics (Sensors)

Data from sensors defined in the `<sensor>` section of your MJCF will be published.

- **Single-Dimension Sensors**:
  - Topic: `<ros_namespace>sensor/<sensor_name>`
  - Type: `example_interfaces::msg::Float64`
- **Multi-Dimension Sensors**:
  - Topic: `<ros_namespace>sensor/<sensor_name>`
  - Type: `example_interfaces::msg::Float64MultiArray`

Where `<sensor_name>` is the `name` attribute from the MJCF.

### ➡️ Subscribed Topics (Actuators)

Commands for actuators defined in the `<actuator>` section of your MJCF can be sent via these topics.

- Topic: `<ros_namespace>actuators/<actuator_name>/command`
- Type: `example_interfaces::msg::Float64`
  - The `data` field of the message is applied to `mjData->ctrl[]` for the respective actuator.

Where `<actuator_name>` is the `name` attribute from the MJCF.

---

## 💡 Example

Consider this MJCF snippet for a simple arm:

```xml
<mujoco model="simple_arm_example">
    <compiler angle="radian"/>

    <extension>
        <plugin plugin="mujoco.ros2">
            <config key="ros_namespace" value="simple_arm/"/>
            <config key="topic_queue_size" value="10"/>
        </plugin>
    </extension>

    <worldbody>
        <!-- ... robot definition ... -->
        <body name="arm_base">
            <joint name="joint1" type="hinge" axis="0 0 1"/>
            <!-- ... more links and joints ... -->
        </body>
    </worldbody>

    <sensor>
        <jointpos name="joint1_pos" joint="joint1"/>
        <user name="my_custom_sensor" dim="3" /> <!-- Example user sensor -->
    </sensor>

    <actuator>
        <motor name="motor1" joint="joint1" gear="100"/>
    </actuator>
</mujoco>
```

**This would result in:**

- **Publishers**:
  - `/simple_arm/sensor/joint1_pos` (`Float64`)
  - `/simple_arm/sensor/my_custom_sensor` (`Float64MultiArray`)
- **Subscribers**:
  - `/simple_arm/actuators/motor1/command` (`Float64`)

Interact using ROS 2 tools:

```bash
# Listen to a sensor
ros2 topic echo /simple_arm/sensor/joint1_pos

# Send a command to an actuator
ros2 topic pub /simple_arm/actuators/motor1/command example_interfaces/msg/Float64 '{data: 0.5}' --once
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
    <td align="center"><a href="https://github.com/GabrielCosme"><img src="https://avatars.githubusercontent.com/u/62270066?v=4?s=100" width="100px;" alt="Gabriel Cosme Barbosa"/><br/><sub><b>Gabriel Cosme Barbosa</b></sub></a><br/><a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=GabrielCosme" title="Code">💻</a> <a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=GabrielCosme" title="Documentation">📖</a> <a href="#research-GabrielCosme" title="Research">🔬</a> <a href="https://github.com/Team-Micras/MicrasFirmware/pulls?q=is%3Apr+reviewed-by%3AGabrielCosme" title="Reviewed Pull Requests">👀</a></td>
    <td align="center"><a href="https://github.com/PedroDeSanti"><img src="https://avatars.githubusercontent.com/u/62271285?v=4" width="100px;" alt="Pedro de Santi"/><br/><sub><b>Pedro de Santi</b></sub></a><br/><a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=PedroDeSanti" title="Code">💻</a> <a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=PedroDeSanti" title="Documentation">📖</a> <a href="#research-PedroDeSanti" title="Research">🔬</a> <a href="https://github.com/Team-Micras/MicrasFirmware/pulls?q=is%3Apr+reviewed-by%3APedroDeSanti" title="Reviewed Pull Requests">👀</a></td>
  </tr>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!
