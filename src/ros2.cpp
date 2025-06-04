#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>

#include "ros2.hpp"

namespace mujoco::plugin::ros2 {

constexpr char kAttrROSNamespace[] = "ros_namespace";

ROS2Plugin::Config ROS2Plugin::get_config_from_model(const mjModel* m, int instance) {
    Config config;
    // config.ros_namespace = ReadOptionalStringAttr(m, instance, kAttrROSNamespace).value_or("mujoco/");
    return config;
}

ROS2Plugin::ROS2Plugin(const Config& config) {
    this->ros_namespace = config.ros_namespace;
    rclcpp::init(0, nullptr);

    this->node = rclcpp::Node::make_shared("mujoco_ros2_plugin");
    RCLCPP_INFO(this->node->get_logger(), "ROS2 Mujoco Plugin node started");
}

ROS2Plugin::~ROS2Plugin() {
    RCLCPP_INFO(this->node->get_logger(), "Destroying ROS2 Plugin instance");
    rclcpp::shutdown();
}

void ROS2Plugin::create_sensor_publishers(const mjModel* m) {
    RCLCPP_INFO(this->node->get_logger(), "Creating sensor publishers for %d sensors", m->nsensor);

    for (int i = 0; i < m->nsensor; i++) {
        const char*       sensor_name = m->names + m->name_sensoradr[i];
        const std::string topic_name = this->ros_namespace + "sensor/" + std::string(sensor_name);

        int         pub_index;
        int         num_dimensions = m->sensor_dim[i];
        mjtDataType sensor_datatype = static_cast<mjtDataType>(m->sensor_datatype[i]);

        if (num_dimensions == 1) {
            auto pub = this->node->create_publisher<example_interfaces::msg::Float64>(topic_name, 1);
            double_sensor_publishers.push_back(pub);
            pub_index = static_cast<int>(double_sensor_publishers.size() - 1);
        } else {
            auto pub = this->node->create_publisher<example_interfaces::msg::Float64MultiArray>(topic_name, 1);
            multiarray_sensor_publishers.push_back(pub);
            pub_index = static_cast<int>(multiarray_sensor_publishers.size() - 1);
        }

        RCLCPP_INFO(
            this->node->get_logger(), "Created publisher for sensor '%s' of type %d with data type %d on topic '%s'",
            sensor_name, m->sensor_type[i], sensor_datatype, topic_name.c_str()
        );

        this->sensors.push_back({i, pub_index});
    }
}

void ROS2Plugin::create_actuator_subscribers(const mjModel* m) {
    for (int i = 0; i < m->nu; i++) {
        const char* actuator_name = m->names + m->name_actuatoradr[i];
        std::string topic_name = this->ros_namespace + "actuators/" + std::string(actuator_name) + "/command";

        auto sub = this->node->create_subscription<example_interfaces::msg::Float64>(
            topic_name, 1, [](const example_interfaces::msg::Float64::SharedPtr msg) {}
        );

        actuator_subscribers.push_back(sub);
        this->actuators.push_back({i, static_cast<int>(actuator_subscribers.size() - 1)});

        RCLCPP_INFO(
            this->node->get_logger(), "Created subscriber for actuator '%s' on topic '%s'", actuator_name,
            topic_name.c_str()
        );
    }
}

void ROS2Plugin::compute(const mjModel* m, mjData* d, int instance) {
    if (not initialized) {
        this->create_sensor_publishers(m);
        this->create_actuator_subscribers(m);
        initialized = true;
    }

    for (const auto& sensor : this->sensors) {
        int     num_dimensions = m->sensor_dim[sensor.object_index];
        int     sensor_address = m->sensor_adr[sensor.object_index];
        mjtNum* sensor_data = &d->sensordata[sensor_address];

        if (num_dimensions == 1) {
            example_interfaces::msg::Float64 msg;
            msg.data = sensor_data[0];
            this->double_sensor_publishers[sensor.comm_index]->publish(msg);
        } else {
            example_interfaces::msg::Float64MultiArray msg;
            msg.data.resize(num_dimensions);

            for (int j = 0; j < num_dimensions; j++) {
                msg.data[j] = sensor_data[j];
            }

            this->multiarray_sensor_publishers[sensor.comm_index]->publish(msg);
        }
    }

    for (auto& actuator : this->actuators) {
        example_interfaces::msg::Float64 msg;
        rclcpp::MessageInfo              info;

        if (this->actuator_subscribers[actuator.comm_index]->take(msg, info)) {
            d->ctrl[actuator.object_index] = msg.data;
        }
    }
}

void ROS2Plugin::reset() {
    RCLCPP_INFO(this->node->get_logger(), "Resetting ROS2 Plugin state from instance");
    this->initialized = false;
    this->sensors.clear();
    this->actuators.clear();
    this->double_sensor_publishers.clear();
    this->multiarray_sensor_publishers.clear();
    this->actuator_subscribers.clear();
}

void ROS2Plugin::register_plugin() {
    mjpPlugin plugin;
    mjp_defaultPlugin(&plugin);
    plugin.name = "mujoco.ros2";

    plugin.capabilityflags |= mjPLUGIN_PASSIVE;

    std::vector<const char*> attributes = {kAttrROSNamespace};

    plugin.nattribute = attributes.size();
    plugin.attributes = attributes.data();

    plugin.nstate = +[](const mjModel* m, int instance) { return 0; };

    plugin.init = +[](const mjModel* m, mjData* d, int instance) {
        std::fprintf(stderr, "Initializing ROS2 plugin for instance %d\n", instance);
        if (d->plugin_data[instance] != 0) {
            std::fprintf(stderr, "ROS2 plugin already initialized for instance %d\n", instance);
        }

        Config config = get_config_from_model(m, instance);
        auto*  ros2 = new ROS2Plugin(config);

        if (ros2 == nullptr) {
            return -1;
        }

        d->plugin_data[instance] = reinterpret_cast<uintptr_t>(ros2);
        return 0;
    };

    plugin.copy = +[](mjData* dest, const mjModel* m, const mjData* src, int instance) {
        fprintf(stderr, "Copying plugin data is not implemented for ROS2 plugin.\n");
    };

    plugin.reset = +[](const mjModel* m, mjtNum* plugin_state, void* plugin_data, int instance) {
        auto* ros2 = reinterpret_cast<ROS2Plugin*>(plugin_data);
        ros2->reset();
    };

    plugin.compute = +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
        auto* ros2 = reinterpret_cast<ROS2Plugin*>(d->plugin_data[instance]);
        ros2->compute(m, d, instance);
    };

    plugin.destroy = +[](mjData* d, int instance) {
        delete reinterpret_cast<ROS2Plugin*>(d->plugin_data[instance]);
        d->plugin_data[instance] = 0;
    };

    mjp_registerPlugin(&plugin);
}
}  // namespace mujoco::plugin::ros2
