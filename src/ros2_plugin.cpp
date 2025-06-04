#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>

#include "ros2_plugin.hpp"

namespace mujoco::plugin::ros2 {

static constexpr char attr_key_ros_namespace[] = "ros_namespace";
static constexpr char attr_key_topic_queue_size[] = "topic_queue_size";

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

ROS2Plugin::Config ROS2Plugin::get_config_from_model(const mjModel* model, int instance) {
    Config config;
    config.ros_namespace = read_string_attr(mj_getPluginConfig(model, instance, attr_key_ros_namespace), "mujoco/");
    config.topic_queue_size = read_int_attr(mj_getPluginConfig(model, instance, attr_key_topic_queue_size), 1);
    return config;
}

ROS2Plugin::ROS2Plugin(const Config& config) :
    ros_namespace(config.ros_namespace), topic_queue_size(config.topic_queue_size) {
    if (rclcpp::ok()) {
        RCLCPP_INFO(rclcpp::get_logger("mujoco"), "ROS2 already initialized");
        rclcpp::shutdown();
    }

    rclcpp::init(0, nullptr);
    this->node = rclcpp::Node::make_shared("mujoco_ros2_plugin");
    RCLCPP_INFO(this->node->get_logger(), "ROS2 Mujoco Plugin node started");
}

ROS2Plugin::~ROS2Plugin() {
    RCLCPP_INFO(this->node->get_logger(), "Destroying ROS2 Plugin instance");
    rclcpp::shutdown();
}

void ROS2Plugin::create_sensor_publishers(const mjModel* model) {
    RCLCPP_INFO(this->node->get_logger(), "Creating sensor publishers for %d sensors", model->nsensor);

    for (int i = 0; i < model->nsensor; i++) {
        const char*       sensor_name = model->names + model->name_sensoradr[i];
        const std::string topic_name = this->ros_namespace + "sensor/" + std::string(sensor_name);

        int  pub_index = 0;
        int  num_dimensions = model->sensor_dim[i];
        auto sensor_datatype = static_cast<mjtDataType>(model->sensor_datatype[i]);

        if (num_dimensions == 1) {
            auto pub =
                this->node->create_publisher<example_interfaces::msg::Float64>(topic_name, this->topic_queue_size);
            double_sensor_publishers.push_back(pub);
            pub_index = static_cast<int>(double_sensor_publishers.size() - 1);
        } else {
            auto pub = this->node->create_publisher<example_interfaces::msg::Float64MultiArray>(
                topic_name, this->topic_queue_size
            );
            multiarray_sensor_publishers.push_back(pub);
            pub_index = static_cast<int>(multiarray_sensor_publishers.size() - 1);
        }

        RCLCPP_INFO(
            this->node->get_logger(), "Created publisher for sensor '%s' of type %d with data type %d on topic '%s'",
            sensor_name, model->sensor_type[i], sensor_datatype, topic_name.c_str()
        );

        this->sensors.push_back({i, pub_index});
    }
}

void ROS2Plugin::create_actuator_subscribers(const mjModel* model) {
    for (int i = 0; i < model->nu; i++) {
        const char* actuator_name = model->names + model->name_actuatoradr[i];
        std::string topic_name = this->ros_namespace + "actuators/" + std::string(actuator_name) + "/command";

        auto sub = this->node->create_subscription<example_interfaces::msg::Float64>(
            topic_name, this->topic_queue_size, [](const example_interfaces::msg::Float64::SharedPtr msg) {}
        );

        actuator_subscribers.push_back(sub);
        this->actuators.push_back({i, static_cast<int>(actuator_subscribers.size() - 1)});

        RCLCPP_INFO(
            this->node->get_logger(), "Created subscriber for actuator '%s' on topic '%s'", actuator_name,
            topic_name.c_str()
        );
    }
}

void ROS2Plugin::compute(const mjModel* model, mjData* data) {
    if (not initialized) {
        this->create_sensor_publishers(model);
        this->create_actuator_subscribers(model);
        initialized = true;
    }

    for (const auto& sensor : this->sensors) {
        int     num_dimensions = model->sensor_dim[sensor.model_index];
        int     sensor_address = model->sensor_adr[sensor.model_index];
        mjtNum* sensor_data = &data->sensordata[sensor_address];

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
            data->ctrl[actuator.model_index] = msg.data;
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

    std::vector<const char*> attributes = {
        attr_key_ros_namespace,
        attr_key_topic_queue_size,
    };

    plugin.nattribute = static_cast<int>(attributes.size());
    plugin.attributes = attributes.data();
    plugin.nstate = +[](const mjModel* /*model*/, int /*instance*/) { return 0; };

    plugin.init = +[](const mjModel* model, mjData* data, int instance) {
        Config config = get_config_from_model(model, instance);
        auto*  ros2 = new ROS2Plugin(config);

        if (ros2 == nullptr) {
            return -1;
        }

        data->plugin_data[instance] = reinterpret_cast<uintptr_t>(ros2);
        return 0;
    };

    plugin.reset = +[](const mjModel* /*model*/, mjtNum* /*plugin_state*/, void* plugin_data, int /*instance*/) {
        auto* ros2 = reinterpret_cast<ROS2Plugin*>(plugin_data);
        ros2->reset();
    };

    plugin.compute = +[](const mjModel* model, mjData* data, int instance, int /*capability_bit*/) {
        auto* ros2 = reinterpret_cast<ROS2Plugin*>(data->plugin_data[instance]);
        ros2->compute(model, data);
    };

    plugin.destroy = +[](mjData* data, int instance) {
        delete reinterpret_cast<ROS2Plugin*>(data->plugin_data[instance]);
        data->plugin_data[instance] = 0;
    };

    mjp_registerPlugin(&plugin);
}
}  // namespace mujoco::plugin::ros2
