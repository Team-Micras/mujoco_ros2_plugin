#ifndef MUJOCO_PLUGIN_ROS2_HPP
#define MUJOCO_PLUGIN_ROS2_HPP

#include <example_interfaces/msg/float64.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <memory>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <rclcpp/rclcpp.hpp>

namespace mujoco::plugin::ros2 {

class ROS2Plugin {
public:
    struct Config {
        std::string ros_namespace = "mujoco/";
        int topic_queue_size = 1;
    };

    explicit ROS2Plugin(const Config& config);

    ~ROS2Plugin();

    void reset();

    void compute(const mjModel* m, mjData* d, int instance);

    static void register_plugin();

    static Config get_config_from_model(const mjModel* m, int instance);

private:
    struct IndexData {
        int object_index;
        int comm_index;
    };

    /**
     * @brief Create ROS2 publishers for sensor data.
     */
    void create_sensor_publishers(const mjModel* m);

    /**
     * @brief Create ROS2 subscribers for actuator commands.
     */
    void create_actuator_subscribers(const mjModel* m);

    std::string ros_namespace;
    int topic_queue_size;

    rclcpp::Node::SharedPtr node;

    std::vector<IndexData> sensors;
    std::vector<IndexData> actuators;

    /**
     * @brief Publishers for sensor data.
     */
    ///@{
    std::vector<rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr>           double_sensor_publishers;
    std::vector<rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr> multiarray_sensor_publishers;
    ///@}

    /**
     * @brief Subscribers for actuator commands.
     */
    std::vector<rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr> actuator_subscribers;

    bool initialized = false;
};
}  // namespace mujoco::plugin::ros2

#endif  // MUJOCO_PLUGIN_ROS2_HPP
