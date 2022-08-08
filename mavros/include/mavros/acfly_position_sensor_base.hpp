/**
 * @file acfly_position_sensor_base.h
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief acfly position sensor template
 * @version 1.0
 * @date 2022-02-18
 *
 * @copyright Copyright (c) 2022 acfly
 * For commercial use, please contact acfly: https://www.acfly.cn
 *
 */

#include <mavros/plugin.hpp>
#include <mavros/utils.hpp>

namespace mavros {
namespace plugin {
/**
 * @brief acfly位置传感器基类
 * @note 具体使用方法请参考acfly_slam_sensor插件
 */
class AcflyPositionSensorBase : public plugin::Plugin {
public:
    explicit AcflyPositionSensorBase(plugin::UASPtr uas_) : Plugin(uas_, "AcflyPositionSensor"), reset_counter(0) {
        node->declare_parameter<std::string>("sensor/name", "ACFly_mavros_plus");
        node->declare_parameter("sensor/index", 15);
        node->declare_parameter("sensor/type", 1);
        node->declare_parameter("sensor/data_frame", 4);
        node->declare_parameter("sensor/type", 2);
        node->declare_parameter<float>("sensor/delay", 0.05);
        node->declare_parameter<float>("sensor/trust_xy", 0.01);
        node->declare_parameter<float>("sensor/trust_z", 0.01);
    }

    void get_sensor_info() {
        node->get_parameter("sensor/name", sensor_name);
        node->get_parameter("sensor/index", sensor_ind);                  // 最高16路，数组形式，0~15
        node->get_parameter("sensor/type", sensor_type);                   // 相对定位
        node->get_parameter("sensor/data_frame", sensor_data_frame);       // SLAM坐标系下的位置
        node->get_parameter("sensor/data_type", sensor_data_type);         // 三轴位置
        node->get_parameter("sensor/delay", sensor_delay);       // 延时(s)
        node->get_parameter("sensor/trust_xy", sensor_trust_xy); // xy方向方差(m^2)
        node->get_parameter("sensor/trust_z", sensor_trust_z);   // z方向方差(m^2)
    }

    void send_register_position_sensor_frame(uint8_t data_frame) {
        mavlink::ACFly::msg::ACFly_RegeisterPosSensor rp{};
        uas->msg_set_target(rp);
        mavlink::set_string(rp.sensor_name, sensor_name);
        rp.ind       = static_cast<int8_t>(sensor_ind);
        rp.type      = static_cast<uint8_t>(sensor_type);
        rp.DataFrame = data_frame;
        rp.DataType  = static_cast<uint8_t>(sensor_data_type);
        rp.delay     = sensor_delay;
        rp.trustXY   = sensor_trust_xy;
        rp.trustZ    = sensor_trust_z;

        uas->send_message_ignore_drop(rp);
    }

    void register_position_sensor() {
        send_register_position_sensor_frame(static_cast<uint8_t>(sensor_data_frame) | (1 << 7));
    }

    void unregister_position_sensor() {
        send_register_position_sensor_frame(static_cast<uint8_t>(sensor_data_frame) | (1 << 6));
    }

    void set_position_sensor_unavailable() {
        send_register_position_sensor_frame(static_cast<uint8_t>(sensor_data_frame));
    }

    void update_position_sensor(const builtin_interfaces::msg::Time   &stamp,
                                Eigen::Vector3d    pos,
                                Eigen::Vector3d    vel,
                                Eigen::Quaterniond att = Eigen::Quaterniond::Identity()) {
        // 未更新则退出
        if (last_transform_stamp == stamp)
            return;

        last_transform_stamp = stamp;

        // 消息标头和传感器基本信息
        mavlink::ACFly::msg::ACFly_UpdatePosSensor up{};
        uas->msg_set_target(up);
        up.ind      = static_cast<int8_t>(sensor_ind);
        up.DataType = static_cast<uint8_t>(sensor_data_type);

        // 定位信息
        up.posX       = pos[0];
        up.posY       = pos[1];
        up.posZ       = pos[2];
        up.velX       = vel[0];
        up.velY       = vel[1];
        up.velZ       = vel[2];
        up.AttQuat[0] = att.w();
        up.AttQuat[1] = att.x();
        up.AttQuat[2] = att.y();
        up.AttQuat[3] = att.z();

        // 传感器测量信息
        up.delay   = node->get_clock()->now().toSec() - stamp.toSec();
        up.trustXY = sensor_trust_xy;
        up.trustZ  = sensor_trust_z;
        up.reset   = reset_counter;

        uas->send_message_ignore_drop(up);
    }

protected:
    std::string sensor_name;
    int         sensor_ind, sensor_type;
    int         sensor_data_frame, sensor_data_type;
    float       sensor_delay;
    float       sensor_trust_xy, sensor_trust_z;
    uint8_t     reset_counter;

private:
    builtin_interfaces::msg::Time last_transform_stamp;
};
} // namespace plugin
} // namespace mavros