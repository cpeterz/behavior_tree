#include "base_interfaces/msg/game.hpp"
#include "base_interfaces/msg/navigation.hpp"
#include "base_interfaces/msg/armors.hpp"
#include "base_interfaces/msg/shooter.hpp"
#include "base_interfaces/msg/top.hpp"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"

using std::placeholders::_1;
namespace wmj
{
    // 装甲板信息
    struct Armor_msg
    {
        int armor_number = 0;       // 识别到的装甲板的数量
        double armor_distance = 5;  // 最近的装甲板的距离
        double armor_timestamp = 0; // 装甲板时间戳
    };

    // 导航信息
    struct Navigation_msg
    {
        double QUAT_1 = 0; // 四元数实数
        double QUAT_i = 0; // 四元数虚数部分
        double QUAT_j = 0;
        double QUAT_k = 0;
        double navigation_timestamp = 0; // 导航事件戳
        bool navigation_status = false;      // 当前导航状态，true表示正在移动
    };

    // 比赛状况信息
    struct Game_msg
    {
        double outpost_blood = 1000;  // 前哨站血量
        double sentry_blood = 1000;   // 哨兵血量
        int bullet_num = 700;         // 剩余子弹数目
        double time_left = 300;      // 剩余比赛时间
        double game_timestamp = 0; // 比赛状况事件戳
        bool manual_top = true;       // 是否手动开启小陀螺
    };

    // 共用ROS节点，负责收发消息
    class Topic_Node : public rclcpp::Node
    {
    public:
        Topic_Node(std::string name) : Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "Topic Node test start working");
        }
    };

    // 数据读取节点类
    class DataReadNode : public BT::SyncActionNode
    {
    public:
        DataReadNode(const std::string &name, const BT::NodeConfig &config,
                     rclcpp::Node::SharedPtr node);

        BT::NodeStatus tick() override; // tick函数，主运行函数
        static BT::PortsList providedPorts();

    private:
        std::shared_ptr<rclcpp::Node> node_; // 信息发布收取节点

        Armor_msg armor_msg;           // 装甲板信息
        Navigation_msg navigation_msg; // 导航信息
        Game_msg game_msg;             // 比赛状况信息

        double last_armor_timestamp = 0; // 记录上次消息的事件戳
        double last_navigation_timestamp = 0;
        double last_game_timestamp = 0;
        int armor_msg_count = 0; // 记录当前消息延误次数
        int game_msg_count = 0;
        int navigation_msg_count = 0;

        rclcpp::Subscription<base_interfaces::msg::Armors>::SharedPtr sub_armors;         // 装甲板信息订阅者
        rclcpp::Subscription<base_interfaces::msg::Game>::SharedPtr sub_game;             // 比赛状况订阅者
        rclcpp::Subscription<base_interfaces::msg::Navigation>::SharedPtr sub_navigation; // 导航信息订阅

        // 订阅者回调函数，用于接收消息并储存到类成员中
        void armor_call_back(const base_interfaces::msg::Armors::SharedPtr msg);
        void game_call_back(const base_interfaces::msg::Game::SharedPtr msg);
        void navigation_call_back(const base_interfaces::msg::Navigation::SharedPtr msg);
    };

    // 自瞄类
    class Detect_Node : public BT::SyncActionNode
    {
    public:
        Detect_Node(const std::string &name, const BT::NodeConfig &config,
                    rclcpp::Node::SharedPtr node);

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();

    private:
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Publisher<base_interfaces::msg::Shooter>::SharedPtr pub_shooter; // 子弹发射发布者

        int bullet_rate; // 弹速                                                    // 发弹频率
    };

    // 导航类
    class Navigation_Node : public BT::SyncActionNode
    {
    public:
        Navigation_Node(const std::string &name, const BT::NodeConfig &config,
                        rclcpp::Node::SharedPtr node);

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();

    private:
        rclcpp::Publisher<base_interfaces::msg::Navigation>::SharedPtr pub_navigation; // 导航信息发布者
        std::shared_ptr<rclcpp::Node> node_;
    };

    // 小陀螺类
    class Top_Node : public BT::SyncActionNode
    {
    public:
        Top_Node(const std::string &name, const BT::NodeConfig &config,
                 rclcpp::Node::SharedPtr node);

        BT::NodeStatus tick() override;
        static BT::PortsList providedPorts();

    private:
        rclcpp::Publisher<base_interfaces::msg::Top>::SharedPtr pub_top;
        std::shared_ptr<rclcpp::Node> node_;
        bool top_status;
    };
}