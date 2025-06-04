#ifndef MUJOCO_PLUGIN_ROS2_HPP
#define MUJOCO_PLUGIN_ROS2_HPP

#include <example_interfaces/msg/float64.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <rclcpp/rclcpp.hpp>

namespace mujoco::plugin::ros2 {

/**
 * @brief ROS2 Plugin for Mujoco to handle sensor data publishing and actuator command subscribing.
 *
 * This plugin allows Mujoco to communicate with ROS2 by publishing sensor data
 * and subscribing to actuator commands.
 */
class ROS2Plugin {
public:
    /**
     * @brief Configuration options for the ROS2 plugin.
     */
    struct Config {
        std::string ros_namespace = "mujoco/";  // Namespace for ROS2 topics
        int         topic_queue_size = 1;       // Queue size for ROS2 topics
    };

    /**
     * @brief Construct a new ROS2Plugin object
     *
     * @param config Configuration options for the plugin.
     */
    explicit ROS2Plugin(const Config& config);

    /**
     * @brief Destroy the ROS2Plugin object.
     */
    ~ROS2Plugin();

    /**
     * @brief Reset ROS 2 Plugin cleaning up all publishers and subscribers.
     */
    void reset();

    /**
     * @brief Sensor data and read actuator commands.
     *
     * @param model MuJoCo model pointer.
     * @param data MuJoCo data pointer.
     */
    void compute(const mjModel* model, mjData* data);

    /**
     * @brief Register the ROS2 plugin.
     */
    static void register_plugin();

    /**
     * @brief Get the configuration from the Mujoco model.
     *
     * This function extracts the ROS2 plugin configuration from the Mujoco model attributes.
     *
     * @param model MuJoCo model pointer.
     * @param instance Instance index of the plugin.
     * @return Configuration for the ROS2 plugin.
     */
    static Config get_config_from_model(const mjModel* model, int instance);

private:
    /**
     * @brief Data structure to hold index information for sensors and actuators.
     */
    struct IndexData {
        int object_index;  // Index of the object in the Mujoco model
        int comm_index;    // Index of the object in the publisher or subscriber array.
    };

    /**
     * @brief Create ROS2 publishers for sensor data.
     */
    void create_sensor_publishers(const mjModel* model);

    /**
     * @brief Create ROS2 subscribers for actuator commands.
     */
    void create_actuator_subscribers(const mjModel* model);

    /**
     * @brief Flag to check if the ROS 2 topics have been initialized.
     */
    bool initialized = false;

    /**
     * @brief ROS 2 namespace for the simulation topics.
     */
    std::string ros_namespace;

    /**
     * @brief Queue size for ROS 2 topics.
     */
    int topic_queue_size;

    /**
     * @brief ROS2 node handle.
     */
    rclcpp::Node::SharedPtr node;

    /**
     * @brief Arrays to hold sensor and actuator indexes.
     */
    ///@{
    std::vector<IndexData> sensors;
    std::vector<IndexData> actuators;
    ///@}

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
};
}  // namespace mujoco::plugin::ros2

#endif  // MUJOCO_PLUGIN_ROS2_HPP
