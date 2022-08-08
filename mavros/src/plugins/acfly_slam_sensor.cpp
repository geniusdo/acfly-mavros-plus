/**
 * @file acfly_slam_sensor.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief acfly slam sensor plugin.
 * @version 1.0
 * @date 2022-02-03
 *
 * @copyright Copyright (c) 2022 acfly
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <eigen_conversions/eigen_msg.h>
#include <mavros/acfly_position_sensor_base.h>
#include <mavros/plugin.hpp>
#include <mavros/setpoint_mixin.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Bool.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief acfly slam sensor send plugin
 * @brief acfly位置传感器发送插件，专门用来发送slam信息
 * @warning acfly自定义的信息全部直接以ENU-FLU系发送，与PX4不同
 */
class AcflySlamSensorPlugin : public plugin::AcflyPositionSensorBase,
                              private plugin::TF2ListenerMixin<AcflySlamSensorPlugin> {
public:
    explicit AcflySlamSensorPlugin(plugin::UASPtr uas_) : AcflyPositionSensorBase(uas_, "~acfly_slam_sensor"), tf_rate(10.0) {
        get_sensor_info(&ass_nh);

        // 传感器到飞控body系的三维变换
        Eigen::Affine3d     tr_sensor_body{};
        std::vector<double> rot{}, trans{};
        std::string         sensor_id, body_id;
        node->declare_parameter<std::string>("sensor_id", "camera");
        node->declare_parameter<std::string>("body_id", "base_link");
        node->get_parameter("sensor_id", sensor_id);
        node->get_parameter("body_id", body_id);
        // 获取sensor->body的变换
        if (node->get_parameter("sensor_body_rotation", rot)) {
            tr_sensor_body = Eigen::Affine3d(ftf::quaternion_from_rpy(rot[0], rot[1], rot[2]));
        } else {
            tr_sensor_body = Eigen::Affine3d(ftf::quaternion_from_rpy(0, 0, 0));
            RCLCPP_WARN(
                "acfly_slam_sensor",
                "ASS: No rotation parameter between sensor and body, set to default(0, 0, 0)");
        }
        if (node->get_parameter("sensor_body_translation", trans)) {
            tr_sensor_body.translation() = Eigen::Vector3d(trans[0], trans[1], trans[2]);
        } else {
            tr_sensor_body.translation() = Eigen::Vector3d(0, 0, 0);
            RCLCPP_WARN(
                "acfly_slam_sensor",
                "ASS: No translation parameter between sensor and body, set to default(0, 0, 0)");
        }

        // tf参数
        bool tf_listen;
        node->declare_parameter<std::string>("tf/frame_id", "map");
        node->declare_parameter<std::string>("tf/child_frame_id", "base_link");
        node->declare_parameter("tf/listen", true);
        node->declare_parameter("tf/rate_limit", 10.0);
        
        node->get_parameter<std::string>("tf/frame_id", tf_frame_id);
        node->get_parameter<std::string>("tf/child_frame_id", tf_child_frame_id);
        node->get_parameter("tf/listen", tf_listen);
        node->get_parameter("tf/rate_limit", tf_rate);

        // 添加静态tf(SLAM传感器与飞控body系之间的变换)
        std::vector<geometry_msgs::TransformStamped> transform_vector;
        uas->add_static_transform(sensor_id, body_id, tr_sensor_body, transform_vector);
        uas->tf2_static_broadcaster.sendTransform(transform_vector);

        // 启动tf监听线程(详细请看TF2ListenerMixin)或订阅
        if (tf_listen) {
            RCLCPP_INFO_STREAM("acfly_slam_sensor", "ASS: Listen to transform "
                                                           << tf_frame_id << " -> "
                                                           << tf_child_frame_id);
            tf2_start("acfly_slam_tf", &AcflySlamSensorPlugin::transform_cb);
        } else {
            RCLCPP_INFO_STREAM("acfly_slam_sensor", "ASS: Subscribe pose");
            pose_sub = node->create_subscription<geometry_msgs::PoseStamped>(
                    "pose", 10, std::bind(&AcflySlamSensorPlugin::pose_cb, this, _1));
            pose_cov_sub =node->create_subscription<geometry_msgs::PoseWithCovarianceStamped>(
                    "pose_cov", 10, std::bind(&AcflySlamSensorPlugin::pose_cov_cb, this, _1));
        }

        // 重置位置传感器
        reset_sub = node->create_subscription<std_msgs::Bool>(
                    "reset", 10, std::bind(&AcflySlamSensorPlugin::reset_cb, this, _1));
        // 设置位置传感器不可用
        block_sub = node->create_subscription<std_msgs::Bool>(
                    "block", 10, std::bind(&AcflySlamSensorPlugin::block_cb, this, _1));

        // 回环检测
        loop_sub = node->create_subscription<std_msgs::Bool>(
                    "loop", 10, std::bind(&AcflySlamSensorPlugin::loop_cb, this, _1));
    }

    Subscriptions get_subscriptions() override {
        return {/* 禁用接收 */};
    }

private:
    friend class TF2ListenerMixin;

    rclcpp::Subscription<geometry_msgs::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<geometry_msgs::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub;
    rclcpp::Subscription<std_msgs::Bool>::SharedPtr reset_sub;
    rclcpp::Subscription<std_msgs::Bool>::SharedPtr block_sub;
    rclcpp::Subscription<std_msgs::Bool>::SharedPtr loop_sub;

    std::string tf_frame_id;
    std::string tf_child_frame_id;
    double      tf_rate;

    /* ros callbacks */
    /* ROS回调函数 */

    void transform_cb(const geometry_msgs::TransformStamped &transform) {
        if (!uas->get_pos_sensor_connection_status(sensor_ind)) {
            register_position_sensor();
        } else {
            Eigen::Affine3d tr;
            tf::transformMsgToEigen(transform.transform, tr);

            update_position_sensor(transform.header.stamp, tr.translation(),
                                   Eigen::Vector3d::Zero(), Eigen::Quaterniond(tr.rotation()));
        }
    }

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &pose_stamp) {
        if (!uas->get_pos_sensor_connection_status(sensor_ind)) {
            register_position_sensor();
        } else {
            Eigen::Affine3d tr;
            tf::poseMsgToEigen(pose_stamp->pose, tr);

            update_position_sensor(pose_stamp->header.stamp, tr.translation(),
                                   Eigen::Vector3d::Zero(), Eigen::Quaterniond(tr.rotation()));
        }
    }

    void pose_cov_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_cov_stamp) {
        if (!uas->get_pos_sensor_connection_status(sensor_ind)) {
            register_position_sensor();
        } else {
            Eigen::Affine3d tr;
            tf::poseMsgToEigen(pose_cov_stamp->pose.pose, tr);

            update_position_sensor(pose_cov_stamp->header.stamp, tr.translation(),
                                   Eigen::Vector3d::Zero(), Eigen::Quaterniond(tr.rotation()));
        }
    }

    void reset_cb(const std_msgs::Bool::ConstPtr &reset) {
        if (reset->data)
            unregister_position_sensor();
    }

    void block_cb(const std_msgs::Bool::ConstPtr &block) {
        if (block->data)
            set_position_sensor_unavailable();
    }

    void loop_cb(const std_msgs::Bool::ConstPtr &loop) {
        // 可能会有多线程读写问题
        if (loop->data)
            reset_counter++;
    }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::AcflySlamSensorPlugin, mavros::plugin::Plugin)