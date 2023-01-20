#include "base_interfaces/msg/game.hpp"
#include "base_interfaces/msg/navigation.hpp"
#include "base_interfaces/msg/armors.hpp"
#include "base_interfaces/msg/shooter.hpp"
#include "base_interfaces/msg/top.hpp"
#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h" //调试用头文件
// #include "behaviortree_cpp/loggers/bt_file_logger.h"
// #include "behaviortree_cpp/loggers/bt_minitrace_logger.h"
// #include "behaviortree_cpp/loggers/bt_zmq_publisher.h"

using std::placeholders::_1;

namespace wmj
{
    // 装甲板信息
    struct Armor_msg
    {
        int armor_number;       // 识别到的装甲板的数量
        double armor_distance;  // 最近的装甲板的距离
        double armor_timestamp; // 装甲板时间戳
    };

    // 导航信息
    struct Navigation_msg
    {
        double QUAT_1; // 四元数实数
        double QUAT_i; // 四元数虚数部分
        double QUAT_j;
        double QUAT_k;
        double navigation_timestamp; // 导航事件戳
        bool navigation_status;      // 当前导航状态，true表示正在移动
        bool navigation_back;        // 是否返回出发点
    };

    // 比赛状况信息
    struct Game_msg
    {
        double outpost_blood;  // 前哨站血量
        double sentry_blood;   // 哨兵血量
        int bullet_num;        // 剩余子弹数目
        double time_left;      // 剩余比赛时间
        double game_timestamp; // 比赛状况事件戳
        bool manual_top;       // 是否手动开启小陀螺
    };

    // 读取参数
    enum Read_Param
    {
        ALL,        // 所有参数
        ARMOR,      // 装甲板参数
        NAVIGATION, // 导航参数
        GAME        // 比赛状况参数
    };

    // 共用ROS节点，负责收发消息
    class Topic_Node : public rclcpp::Node
    {
    public:
        Topic_Node(std::string name) : Node(name)
        {
            RCLCPP_INFO(this->get_logger(), "Topic Node start working");
        }
    };

    // 数据读取节点
    class DataReadNode : public BT::SyncActionNode
    {
    public:
        DataReadNode(const std::string &name, const BT::NodeConfig &config,
                     rclcpp::Node::SharedPtr node);

        BT::NodeStatus tick() override; // tick函数，主运行函数
        static BT::PortsList providedPorts();
        void readParam(std::string path, Read_Param mode);

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

    // 发射基类
    class Shoot_Node : public BT::SyncActionNode
    {
    public:
        Shoot_Node(const std::string &name, const BT::NodeConfig &config,
                   rclcpp::Node::SharedPtr node);

        static BT::PortsList providedPorts();

    protected:
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Publisher<base_interfaces::msg::Shooter>::SharedPtr pub_shooter; // 子弹发射发布者
        base_interfaces::msg::Shooter msg;
    };

    // 无限制发射节点
    class No_Limit_Shoot_Node : public Shoot_Node
    {
    public:
        No_Limit_Shoot_Node(const std::string &name, const BT::NodeConfig &config,
                            rclcpp::Node::SharedPtr node)
            : Shoot_Node::Shoot_Node(name, config, node)
        {
            RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "No_Limit_Shoot_Node is working");
        }

    private:
        BT::NodeStatus tick() override;
    };

    // 停止发射节点
    class No_Shoot_Node : public Shoot_Node
    {
    public:
        No_Shoot_Node(const std::string &name, const BT::NodeConfig &config,
                      rclcpp::Node::SharedPtr node)
            : Shoot_Node::Shoot_Node(name, config, node)
        {
            RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "No_Shoot_Node is working");
        }

    private:
        BT::NodeStatus tick() override;
    };

    // 限制发射节点
    class Limit_Shoot_Node : public Shoot_Node
    {
    public:
        Limit_Shoot_Node(const std::string &name, const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr node)
            : Shoot_Node::Shoot_Node(name, config, node)
        {
            RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Limit_Shoot_Node is working");
        }

    private:
        BT::NodeStatus tick() override;
    };

    // 小陀螺基类
    class Top_Node : public BT::SyncActionNode
    {
    public:
        Top_Node(const std::string &name, const BT::NodeConfig &config,
                 rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts();

    protected:
        rclcpp::Publisher<base_interfaces::msg::Top>::SharedPtr pub_top;
        std::shared_ptr<rclcpp::Node> node_;
        bool top_status;
        base_interfaces::msg::Top msg;
    };

    // 关闭小陀螺节点
    class Top_off_Node : public Top_Node
    {
    public:
        Top_off_Node(const std::string &name, const BT::NodeConfig &config,
                     rclcpp::Node::SharedPtr node)
            : Top_Node::Top_Node(name, config, node)
        {
            RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Top_off_Node is working");
        }

    private:
        BT::NodeStatus tick() override;
    };

    // 开启小陀螺节点
    class Top_on_Node : public Top_Node
    {
    public:
        Top_on_Node(const std::string &name, const BT::NodeConfig &config,
                    rclcpp::Node::SharedPtr node)
            : Top_Node::Top_Node(name, config, node)
        {
            RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Top_on_Node is woring");
        }

    private:
        BT::NodeStatus tick() override;
    };

    // 导航基类
    class Navigation_Node : public BT::SyncActionNode
    {
    public:
        Navigation_Node(const std::string &name, const BT::NodeConfig &config,
                        rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts();

    protected:
        rclcpp::Publisher<base_interfaces::msg::Navigation>::SharedPtr pub_navigation; // 导航信息发布者
        std::shared_ptr<rclcpp::Node> node_;
        base_interfaces::msg::Navigation msg;
    };

    // 导航开启节点
    class Navigation_on_Node : public Navigation_Node
    {
    public:
        Navigation_on_Node(const std::string &name, const BT::NodeConfig &config,
                           rclcpp::Node::SharedPtr node)
            : Navigation_Node::Navigation_Node(name, config, node)
        {
            RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Navigation_on_Node is working");
        }

    private:
        BT::NodeStatus tick() override;
    };

    // 导航关闭节点
    class Navigation_off_Node : public Navigation_Node
    {
    public:
        Navigation_off_Node(const std::string &name, const BT::NodeConfig &config,
                            rclcpp::Node::SharedPtr node)
            : Navigation_Node::Navigation_Node(name, config, node)
        {
            RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Navigation_off_Node is working");
        }

    private:
        BT::NodeStatus tick() override;
    };

    // 导航返回节点
    class Navigation_back_Node : public Navigation_Node
    {
    public:
        Navigation_back_Node(const std::string &name, const BT::NodeConfig &config,
                             rclcpp::Node::SharedPtr node)
            : Navigation_Node::Navigation_Node(name, config, node)
        {
            RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Navigation_back_Node is working");
        }

    private:
        BT::NodeStatus tick() override;
    };

    // 导航条件节点
    class Navigation_Condition : public BT::ConditionNode
    {
    public:
        Navigation_Condition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts();

    private:
        int Get_Bullet_Rate(); // 获取当前子弹频率
        BT::NodeStatus tick() override;
    };

    // 小陀螺条件节点
    class Top_Condition : public BT::ConditionNode
    {
    public:
        Top_Condition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts();

    private:
        bool Get_Top_Status(); // 获取小陀螺状态,true则开启小陀螺
        BT::NodeStatus tick() override;
    };

    // 姿态条件节点
    class Pose_Condition : public BT::ConditionNode
    {
    public:
        Pose_Condition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts();

    private:
        bool Get_Current_Pose(); // 获取当前状态,false为进攻姿态，true为防御姿态
        BT::NodeStatus tick() override;
    };
}