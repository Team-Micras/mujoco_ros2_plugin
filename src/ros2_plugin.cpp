#include <bit>
#include <cmath>
#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>

#include "ros2_plugin.hpp"

namespace mujoco::plugin::ros2 {
static constexpr unsigned int sec_to_nsec = 1e9;

int read_int_attr(const char* value, int default_value) {
    if (value == nullptr || value[0] == '\0') {
        return default_value;
    }

    return static_cast<int>(std::strtol(value, nullptr, 10));
}

std::string read_string_attr(const char* value, const std::string& default_value) {
    if (value == nullptr) {
        return default_value;
    }

    return {value};
}

Ros2Plugin::Config Ros2Plugin::get_config_from_model(const mjModel* model, int instance) {
    Config config;
    config.node_name = "mujoco_" + std::to_string(instance);

    config.ros_namespace =
        read_string_attr(mj_getPluginConfig(model, instance, attr_key_ros_namespace), attr_default_ros_namespace);

    config.subscribers_reliability = read_string_attr(
        mj_getPluginConfig(model, instance, attr_key_subscribers_reliability), attr_default_subscribers_reliability
    );

    return config;
}

Ros2Plugin::Ros2Plugin(const Config& config) :
    Node{config.node_name},
    ros_namespace{config.ros_namespace},
    subscribers_qos{rclcpp::KeepLast(1)},
    publishers_qos{rclcpp::KeepLast(1)} {
    if (config.subscribers_reliability != "reliable") {
        this->subscribers_qos = this->subscribers_qos.best_effort();
    }

    RCLCPP_INFO(this->get_logger(), "ROS2 Mujoco Plugin node started");
}

void Ros2Plugin::create_sensor_publishers(const mjModel* model) {
    RCLCPP_INFO(this->get_logger(), "Creating sensor publishers for %d sensors", model->nsensor);

    this->clock_msg.sec = 0;
    this->clock_msg.nanosec = 0;
    this->clock_publisher =
        this->create_publisher<builtin_interfaces::msg::Time>(this->ros_namespace + "clock", this->publishers_qos);

    RCLCPP_INFO(this->get_logger(), "Created clock publisher on topic '%sclock'", this->ros_namespace.c_str());

    for (int i = 0; i < model->nsensor; i++) {
        const char*       sensor_name = model->names + model->name_sensoradr[i];
        const std::string topic_name = this->ros_namespace + "sensors/" + std::string(sensor_name);
        const int         num_dimensions = model->sensor_dim[i];

        if (num_dimensions == 1) {
            auto pub = this->create_publisher<example_interfaces::msg::Float64>(topic_name, this->publishers_qos);
            this->scalar_sensor_publishers.push_back(pub);
            this->subscribers_indexes.push_back(this->scalar_sensor_publishers.size() - 1);
        } else {
            auto pub =
                this->create_publisher<example_interfaces::msg::Float64MultiArray>(topic_name, this->publishers_qos);
            this->array_sensor_publishers.push_back(pub);
            this->subscribers_indexes.push_back(this->array_sensor_publishers.size() - 1);
        }

        RCLCPP_INFO(
            this->get_logger(), "Created publisher for sensor '%s' on topic '%s'", sensor_name, topic_name.c_str()
        );
    }
}

void Ros2Plugin::create_actuator_subscribers(const mjModel* model, mjData* data) {
    RCLCPP_INFO(this->get_logger(), "Creating actuator subscribers for %d actuators", model->nu);

    for (int i = 0; i < model->nu; i++) {
        const char* actuator_name = model->names + model->name_actuatoradr[i];
        std::string topic_name = this->ros_namespace + "actuators/" + actuator_name + "/command";

        auto sub = this->create_subscription<example_interfaces::msg::Float64>(
            topic_name, this->subscribers_qos,
            // NOLINTNEXTLINE(performance-unnecessary-value-param)
            [data, i](const example_interfaces::msg::Float64::SharedPtr msg) { data->ctrl[i] = msg->data; }
        );

        actuator_subscribers.push_back(sub);
        RCLCPP_INFO(
            this->get_logger(), "Created subscriber for actuator '%s' on topic '%s'", actuator_name, topic_name.c_str()
        );
    }
}

void Ros2Plugin::compute(const mjModel* model, mjData* data) {
    if (not initialized) {
        this->create_sensor_publishers(model);
        this->create_actuator_subscribers(model, data);
        initialized = true;
    }

    rclcpp::spin_some(this->get_node_base_interface());

    this->clock_msg.nanosec += std::lround(model->opt.timestep * sec_to_nsec);

    if (this->clock_msg.nanosec >= sec_to_nsec) {
        this->clock_msg.sec += static_cast<int32_t>(this->clock_msg.nanosec / sec_to_nsec);
        this->clock_msg.nanosec %= sec_to_nsec;
    }

    this->clock_publisher->publish(this->clock_msg);

    for (int i = 0; i < model->nsensor; i++) {
        const int num_dimensions = model->sensor_dim[i];
        const int sensor_address = model->sensor_adr[i];
        mjtNum*   sensor_data = &data->sensordata[sensor_address];

        if (num_dimensions == 1) {
            example_interfaces::msg::Float64 msg;
            msg.data = sensor_data[0];
            this->scalar_sensor_publishers[this->subscribers_indexes[i]]->publish(msg);
        } else {
            example_interfaces::msg::Float64MultiArray msg;
            msg.layout.dim.resize(1);
            msg.layout.dim[0].size = num_dimensions;
            msg.layout.dim[0].stride = num_dimensions;
            msg.data.assign(sensor_data, sensor_data + num_dimensions);
            this->array_sensor_publishers[this->subscribers_indexes[i]]->publish(msg);
        }
    }
}

void Ros2Plugin::reset() {
    RCLCPP_INFO(this->get_logger(), "Resetting ROS2 Plugin state");
    this->initialized = false;
    this->subscribers_indexes.clear();
    this->scalar_sensor_publishers.clear();
    this->array_sensor_publishers.clear();
    this->actuator_subscribers.clear();
}

void Ros2Plugin::register_plugin() {
    mjpPlugin plugin;
    mjp_defaultPlugin(&plugin);
    plugin.name = "mujoco.ros2";
    plugin.capabilityflags |= mjPLUGIN_PASSIVE;

    std::vector<const char*> attributes = {
        attr_key_ros_namespace,
        attr_key_subscribers_reliability,
    };

    plugin.nattribute = static_cast<int>(attributes.size());
    plugin.attributes = attributes.data();
    plugin.nstate = +[](const mjModel* /*model*/, int /*instance*/) { return 0; };

    plugin.init = +[](const mjModel* model, mjData* data, int instance) {
        if (not rclcpp::ok()) {
            rclcpp::init(0, nullptr);
            rclcpp::uninstall_signal_handlers();
        }

        Config config = get_config_from_model(model, instance);
        auto*  ros2_plugin = new Ros2Plugin(config);

        if (ros2_plugin == nullptr) {
            return -1;
        }

        data->plugin_data[instance] = std::bit_cast<uintptr_t>(ros2_plugin);
        return 0;
    };

    plugin.reset = +[](const mjModel* /*model*/, mjtNum* /*plugin_state*/, void* plugin_data, int /*instance*/) {
        auto* ros2_plugin = std::bit_cast<Ros2Plugin*>(plugin_data);
        ros2_plugin->reset();
    };

    plugin.compute = +[](const mjModel* model, mjData* data, int instance, int /*capability_bit*/) {
        auto* ros2_plugin = std::bit_cast<Ros2Plugin*>(data->plugin_data[instance]);
        ros2_plugin->compute(model, data);
    };

    plugin.destroy = +[](mjData* data, int instance) {
        delete std::bit_cast<Ros2Plugin*>(data->plugin_data[instance]);
        data->plugin_data[instance] = 0;
    };

    mjp_registerPlugin(&plugin);
}
}  // namespace mujoco::plugin::ros2
