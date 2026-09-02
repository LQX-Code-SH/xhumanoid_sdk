/**
 * @file hand_gesture_control.cpp
 * @brief 因时灵巧手手势控制节点 - 实现OK手势和剪刀石头布功能
 *
 * 该节点通过 SetAngle (位置模式) 控制因时灵巧手实现预设手势:
 * - OK手势: 大拇指和食指捏合,其他手指伸直
 * - 石头: 所有手指弯曲握拳
 * - 剪刀: 食指和中指伸直,其他手指弯曲
 * - 布: 所有手指伸直张开
 *
 * 因时手 SetAngle 消息 (inspire_hand_msgs):
 * - hand_id: 手编号 (1=左手, 2=右手)
 * - joint_values[13]: 13 个关节的目标位置
 *
 * ⚠️ 13 个关节的真实布局以 GetAngleAct.joint_names 实测为准,
 *   代码中的手势位置为示例值,需按实际硬件校准。
 *
 * 通过ROS2服务触发手势:
 * - 服务名: /gesture_command
 * - 请求: gesture (ok/rock/scissors/paper)
 */

#include <rclcpp/rclcpp.hpp>
#include <inspire_hand_msgs/msg/set_angle.hpp>
#include <inspire_hand_msgs/msg/get_angle_act.hpp>
#include <inspire_hand_msgs/msg/touch_data.hpp>
#include <inspire_hand_gesture_interfaces/srv/gesture_command.hpp>
#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <array>
#include <cctype>

using GestureCommand = inspire_hand_gesture_interfaces::srv::GestureCommand;

class HandGestureControl : public rclcpp::Node
{
public:
    HandGestureControl() : Node("inspire_hand_gesture_control")
    {
        // 声明参数
        this->declare_parameter("hand_prefix", "right_hand");  // right_hand 或 left_hand
        this->declare_parameter("hand_id", 2);  // 手编号: 1=左手, 2=右手

        // 获取参数
        hand_prefix_ = this->get_parameter("hand_prefix").as_string();
        hand_id_ = this->get_parameter("hand_id").as_int();

        // 构建话题名称
        std::string angle_cmd_topic = hand_prefix_ + "/angle_set";
        std::string angle_actual_topic = hand_prefix_ + "/angle_actual";
        std::string touch_topic = hand_prefix_ + "/touch_data";

        // 创建角度控制发布者
        angle_pub_ = this->create_publisher<inspire_hand_msgs::msg::SetAngle>(
            angle_cmd_topic, 10);

        // 创建角度实际值订阅者
        status_sub_ = this->create_subscription<inspire_hand_msgs::msg::GetAngleAct>(
            angle_actual_topic, 10,
            std::bind(&HandGestureControl::statusCallback, this, std::placeholders::_1));

        // 创建触觉订阅者 (触觉为选配反馈)
        touch_sub_ = this->create_subscription<inspire_hand_msgs::msg::TouchData>(
            touch_topic, 10,
            std::bind(&HandGestureControl::touchCallback, this, std::placeholders::_1));

        // 创建手势控制服务
        gesture_service_ = this->create_service<GestureCommand>(
            "gesture_command",
            std::bind(&HandGestureControl::gestureCommandCallback,
                      this, std::placeholders::_1, std::placeholders::_2));

        // 初始化手势位置映射
        initGesturePositions();

        RCLCPP_INFO(this->get_logger(), "因时灵巧手手势控制节点已启动");
        RCLCPP_INFO(this->get_logger(), "手: %s (hand_id=%d)",
                    hand_prefix_.c_str(), hand_id_);
        RCLCPP_INFO(this->get_logger(), "控制话题: %s", angle_cmd_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "状态话题: %s", angle_actual_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "触觉话题: %s", touch_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "服务: /gesture_command");
        RCLCPP_INFO(this->get_logger(), "支持的手势: ok, rock(石头), scissors(剪刀), paper(布)");
    }

private:
    // 关节数量 (SetAngle.joint_values 固定长度)
    static constexpr int JOINT_COUNT = 13;

    // 位置范围: 0 (示例: 伸直) ~ 1000 (示例: 弯曲), 需按实际硬件校准
    static constexpr int32_t POS_MIN = 0;
    static constexpr int32_t POS_MAX = 1000;

    // 手编号 (vendor demo 07 约定)
    static constexpr int32_t HAND_ID_LEFT = 1;
    static constexpr int32_t HAND_ID_RIGHT = 2;

    // 手势位置数组 (13 个关节的位置值)
    using GesturePositions = std::array<int32_t, JOINT_COUNT>;

    // 发布者
    rclcpp::Publisher<inspire_hand_msgs::msg::SetAngle>::SharedPtr angle_pub_;

    // 订阅者
    rclcpp::Subscription<inspire_hand_msgs::msg::GetAngleAct>::SharedPtr status_sub_;
    rclcpp::Subscription<inspire_hand_msgs::msg::TouchData>::SharedPtr touch_sub_;

    // 服务
    rclcpp::Service<GestureCommand>::SharedPtr gesture_service_;

    // 手势位置映射
    std::map<std::string, GesturePositions> gesture_positions_;

    // 当前状态
    inspire_hand_msgs::msg::GetAngleAct::SharedPtr current_status_;
    inspire_hand_msgs::msg::TouchData::SharedPtr current_touch_;
    bool joint_names_logged_ = false;

    // 参数
    std::string hand_prefix_;
    int hand_id_;

    /**
     * @brief 初始化手势位置映射
     *
     * 位置范围: 0 (示例: 伸直) ~ 1000 (示例: 弯曲)
     * 前 6 个关节沿用强脑手 6 电机的手指映射 (拇指弯曲/拇指旋转/
     * 食指/中指/无名指/小指), 其余 7 个关节按伸直处理。
     * 真实布局以 angle_actual 话题的 joint_names 为准。
     */
    void initGesturePositions()
    {
        // OK手势: 大拇指和食指捏合,其他手指伸直
        gesture_positions_["ok"] = {{
            450,     // 拇指弯曲: 中等弯曲
            800,     // 拇指旋转: 适当旋转角度
            450,     // 食指: 弯曲与拇指捏合
            POS_MIN, // 中指: 伸直
            POS_MIN, // 无名指: 伸直
            POS_MIN, // 小指: 伸直
            POS_MIN, POS_MIN, POS_MIN, POS_MIN, POS_MIN, POS_MIN, POS_MIN  // 其余关节: 伸直
        }};

        // 石头: 所有手指弯曲握拳
        gesture_positions_["rock"] = {{
            800,     // 拇指弯曲: 完全弯曲
            500,     // 拇指旋转: 中间位置
            900,     // 食指: 完全弯曲
            900,     // 中指: 完全弯曲
            900,     // 无名指: 完全弯曲
            900,     // 小指: 完全弯曲
            900, 900, 900, 900, 900, 900, 900  // 其余关节: 弯曲
        }};

        // 剪刀: 食指和中指伸直,其他手指弯曲
        gesture_positions_["scissors"] = {{
            800,     // 拇指弯曲: 弯曲
            500,     // 拇指旋转: 中间位置
            POS_MIN, // 食指: 伸直
            POS_MIN, // 中指: 伸直
            900,     // 无名指: 弯曲
            900,     // 小指: 弯曲
            POS_MIN, POS_MIN, POS_MIN, POS_MIN, POS_MIN, POS_MIN, POS_MIN  // 其余关节: 伸直
        }};

        // 布: 所有手指伸直张开
        gesture_positions_["paper"] = {{
            POS_MIN, // 拇指弯曲: 伸直
            200,     // 拇指旋转: 张开角度
            POS_MIN, // 食指: 伸直
            POS_MIN, // 中指: 伸直
            POS_MIN, // 无名指: 伸直
            POS_MIN, // 小指: 伸直
            POS_MIN, POS_MIN, POS_MIN, POS_MIN, POS_MIN, POS_MIN, POS_MIN  // 其余关节: 伸直
        }};

        RCLCPP_INFO(this->get_logger(), "手势位置已初始化 (位置范围: 0-1000, 待校准)");
    }

    /**
     * @brief 手势控制服务回调函数
     */
    void gestureCommandCallback(
        const std::shared_ptr<GestureCommand::Request> request,
        std::shared_ptr<GestureCommand::Response> response)
    {
        std::string gesture = request->gesture;

        // 转换为小写
        std::transform(gesture.begin(), gesture.end(), gesture.begin(),
                       [](unsigned char c) { return std::tolower(c); });

        RCLCPP_INFO(this->get_logger(), "接收到手势命令: %s", gesture.c_str());

        // 检查手势是否存在
        if (gesture_positions_.find(gesture) == gesture_positions_.end())
        {
            response->success = false;
            response->message = "未知手势: '" + gesture + "'. 支持的手势: ok, rock, scissors, paper";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // 执行手势
        bool result = executeGesture(gesture);

        response->success = result;
        response->message = result ? "手势 '" + gesture + "' 执行成功" : "手势 '" + gesture + "' 执行失败";

        if (result)
        {
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    /**
     * @brief 角度实际值回调函数
     */
    void statusCallback(const inspire_hand_msgs::msg::GetAngleAct::SharedPtr msg)
    {
        current_status_ = msg;
        // 首次收到状态时打印真实关节布局, 辅助校准手势位置
        if (!joint_names_logged_)
        {
            joint_names_logged_ = true;
            std::string names;
            for (size_t i = 0; i < msg->joint_names.size(); i++)
            {
                if (i > 0) names += ", ";
                names += msg->joint_names[i];
            }
            RCLCPP_INFO(this->get_logger(), "关节布局 (joint_names[13]): %s", names.c_str());
        }
    }

    /**
     * @brief 触觉回调函数
     */
    void touchCallback(const inspire_hand_msgs::msg::TouchData::SharedPtr msg)
    {
        current_touch_ = msg;
    }

    /**
     * @brief 执行手势 - 通过Topic发布控制命令
     */
    bool executeGesture(const std::string &gesture)
    {
        // 创建消息
        auto msg = inspire_hand_msgs::msg::SetAngle();
        msg.hand_id = hand_id_;  // 手编号: 1=左手, 2=右手

        // 获取手势位置
        const GesturePositions &pos = gesture_positions_[gesture];

        // 设置 13 个关节位置
        msg.joint_values = pos;

        RCLCPP_INFO(this->get_logger(), "正在执行手势: %s (hand_id=%d)",
                    gesture.c_str(), hand_id_);
        RCLCPP_DEBUG(this->get_logger(),
                     "位置: 前6关节=%d,%d,%d,%d,%d,%d",
                     pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);

        // 发布控制命令
        angle_pub_->publish(msg);

        return true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HandGestureControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
