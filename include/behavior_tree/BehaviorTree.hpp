#include "base_interfaces/msg/game.hpp"
#include "base_interfaces/msg/navigation.hpp"
#include "base_interfaces/msg/armors.hpp"
#include "base_interfaces/msg/aimer.hpp"
#include "base_interfaces/msg/bt_aimer.hpp"
#include "base_interfaces/msg/bt_top.hpp"
#include "base_interfaces/msg/bt_scan.hpp"
#include "base_interfaces/msg/scan_ctrl_info.hpp"
#include "base_interfaces/msg/bt_navigation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h" //调试用头文件
// #include "behaviortree_cpp/loggers/bt_file_logger.h"
// #include "behaviortree_cpp/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp/loggers/bt_zmq_publisher.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "libbase/common.h"

using std::placeholders::_1;

namespace wmj
{
    /**
     * @brief 装甲板信息结构体
    */
    struct Armor_msg
    {
        int armor_number;       // 识别到的装甲板的数量
        double armor_distance;  // 最近的装甲板的距离
        double armor_timestamp; // 装甲板时间戳
    };

    /**
     * @brief 自瞄权限信息
    */
   struct Aimer_msg
   {
        int aimer_if_track;     // 是否跟踪
        bool aimer_shootable;    // 是否发射
        double aimer_timestamp;  // 自瞄时间戳
   };

    /**
     * @brief 导航信息结构体
    */
    struct Navigation_msg
    {
        double navigation_timestamp;         // 导航事件戳
        int navigation_status;               // 导航状态信息，0表示没到目标点，1表示到达目标点，2表示导航崩了。
        int navigation_default_position;     // 默认位置
        double navigation_cur_position_x;    // 当前导航位置
        double navigation_cur_position_y;
        double navigation_cur_position_z;
        double navigation_cur_orientation_x;
        double navigation_cur_orientation_y;
        double navigation_cur_orientation_z;
        double navigation_cur_orientation_w;
    };

    /**
     * @brief 比赛信息结构体
    */
    struct Game_msg
    {
        int outpost_blood;        // 前哨站血量
        int sentry_blood;         // 哨兵血量
        int bullet_num;           // 剩余子弹数目
        int time_left;            // 剩余比赛时间
        double game_timestamp;    // 比赛状况事件戳
        bool manual_top;          // 是否手动开启小陀螺
        bool game_start;          // 比赛是否开始
        int m_alive;              // 我方存活机器人
        int enemy_alive;          // 敌方存活机器人
        double hityaw;            // 被打击角度
    };

    /**
     * @brief 参数读取
    */
    enum Read_Param
    {
        ALL,        // 所有参数
        ARMOR,      // 装甲板参数
        SHOOT,      // 自瞄参数
        GAME,       // 比赛状况参数
        NAV         // 导航参数
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

        BT::NodeStatus tick() override;         // tick函数，主运行函数
        static BT::PortsList providedPorts();
        void readParam(std::string path, Read_Param mode);

    private:
        std::shared_ptr<rclcpp::Node> node_;     // 信息发布收取节点

        Armor_msg armor_msg;                     // 装甲板信息
        Aimer_msg aimer_msg;
        Navigation_msg navigation_msg;           // 导航信息
        Game_msg game_msg;                       // 比赛状况信息

        double last_armor_timestamp = 0;         // 记录上次消息的事件戳
        double last_aimer_timestamp = 0;
        double last_navigation_timestamp = 0;
        double last_game_timestamp = 0;
        int armor_msg_count = 0;                 // 记录当前消息延误次数
        int aimer_msg_count = 0;
        int game_msg_count = 0;
        int navigation_msg_count = 0;
        // int m_waitNum;                        // 等待消息次数
        double m_waitGameMsgTime = 0;            // 等待比赛状况消息时间
        double m_waitNavigationMsgTime = 0;      // 等待导航信息消息时间
        double m_waitArmorMsgTime = 0;           // 等待装甲板消息时间
        double m_waitAimerMsgTime = 0;
        bool if_navigation = true;
        bool debug_game = false;
        bool debug_all = false;
        bool first_time = true;

        rclcpp::Subscription<base_interfaces::msg::Armors>::SharedPtr sub_armors;         // 装甲板信息订阅者
        rclcpp::Subscription<base_interfaces::msg::Aimer>::SharedPtr sub_aimer;           // 自瞄信息订阅者
        rclcpp::Subscription<base_interfaces::msg::Game>::SharedPtr sub_game;             // 比赛状况订阅者
        rclcpp::Subscription<base_interfaces::msg::Navigation>::SharedPtr sub_navigation; // 导航信息订阅

        // 订阅者回调函数，用于接收消息并储存到类成员中
        void armor_call_back(const base_interfaces::msg::Armors::SharedPtr msg);
        void aimer_call_back(const base_interfaces::msg::Aimer::SharedPtr msg);
        void game_call_back(const base_interfaces::msg::Game::SharedPtr msg);
        void navigation_call_back(const base_interfaces::msg::Navigation::SharedPtr msg);
    };

    /**
     * @brief 发射基类，子类包括 限制性发射节点 ， 无限制发射节点 ， 不发射节点
    */
    class Shoot_Node : public BT::SyncActionNode
    {
    public:
        Shoot_Node(const std::string &name, const BT::NodeConfig &config,
                   rclcpp::Node::SharedPtr node);

        static BT::PortsList providedPorts();

    protected:
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Publisher<base_interfaces::msg::BtAimer>::SharedPtr pub_shooter; // 子弹发射发布者
        base_interfaces::msg::BtAimer msg;
    };

    /**
     * @brief 只跟随不发射节点
     * 
    */
    class Track_Node : public Shoot_Node
    {
    public:
        Track_Node(const std::string &name, const BT::NodeConfig &config,
                            rclcpp::Node::SharedPtr node)
            : Shoot_Node::Shoot_Node(name, config, node)
        {
            RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Track_Node is working");
        }

    private:
        BT::NodeStatus tick() override;
    };

    /**
     * @brief 不发射节点
    */
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

    /**
     * @brief 限制性发射节点
     * 
     * @return bullet_rate 弹频
    */
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

    /**
     * @brief 小陀螺基类,子类包括开启小陀螺节点和关闭小陀螺节点
    */
    class Top_Node : public BT::SyncActionNode
    {
    public:
        Top_Node(const std::string &name, const BT::NodeConfig &config,
                 rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts();

    protected:
        rclcpp::Publisher<base_interfaces::msg::BtTop>::SharedPtr topPub;
        std::shared_ptr<rclcpp::Node> node_;
        bool top_status;                          // 小陀螺状态
        base_interfaces::msg::BtTop msg;            // 小陀螺消息
    };

    /**
     * @brief 关闭小陀螺节点
    */
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

    /**
     * @brief 开启小陀螺节点
    */
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

    /**
     * @brief 导航基类，子类包括 开启导航节点，关闭导航节点，自动返航节点
    */
    class Navigation_Node : public BT::SyncActionNode
    {
    public:
        Navigation_Node(const std::string &name, const BT::NodeConfig &config,
                        rclcpp::Node::SharedPtr node);
        static BT::PortsList providedPorts();

    protected:
        rclcpp::Publisher<base_interfaces::msg::BtNavigation>::SharedPtr navigationPub; // 导航信息发布者
        std::shared_ptr<rclcpp::Node> node_;
        base_interfaces::msg::BtNavigation msg;
        geometry_msgs::msg::PoseStamped startPosition;
        geometry_msgs::msg::PoseStamped resumePosition;
        geometry_msgs::msg::PoseStamped gainPosition;
        geometry_msgs::msg::PoseStamped viewPosition;                
    private:
        /**
         * @brief 获取位置信息
         * 
         * @param position 位置
         * 
         * @return PoseStamped 目标位置，四元数类型
        */
        geometry_msgs::msg::PoseStamped GetPositionInfo(int position);
    };

    /**
     * @brief 开启导航节点
     * 
     * @param position 目标位置
    */
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
        int final_last_position = -1;          
        int time_count = 0;
        int out_time = 0;
    };

    /**
     * @brief 导航关闭节点
    */
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

    // /**
    //  * @brief 返航节点
    // */
    // class Navigation_back_Node : public Navigation_Node
    // {
    // public:
    //     Navigation_back_Node(const std::string &name, const BT::NodeConfig &config,
    //                          rclcpp::Node::SharedPtr node)
    //         : Navigation_Node::Navigation_Node(name, config, node)
    //     {
    //         RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Navigation_back_Node is working");
    //     }

    // private:
    //     BT::NodeStatus tick() override;
    // };

    /**
     * @brief 扫描基类
    */
    class Scan_Node : public BT::SyncActionNode
    {
    public:
        Scan_Node(const std::string &name, const BT::NodeConfig &config,
                   rclcpp::Node::SharedPtr node);

        static BT::PortsList providedPorts();

    protected:
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Publisher<base_interfaces::msg::ScanCtrlInfo>::SharedPtr ScanPub;
        base_interfaces::msg::ScanCtrlInfo msg;
    };

    /**
     * @brief 开启360扫描节点
    */
    class Scan_on_Node : public Scan_Node
    {
    public:
        Scan_on_Node(const std::string &name, const BT::NodeConfig &config,
                            rclcpp::Node::SharedPtr node)
            : Scan_Node::Scan_Node(name, config, node)
        {
            RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Scan_on_Node is working");
        }

    private:
        BT::NodeStatus tick() override;
    };

    /**
     * @brief 开启90扫描节点
    */
    class Scan_90_Node : public Scan_Node
    {
    public:
        Scan_90_Node(const std::string &name, const BT::NodeConfig &config,
                            rclcpp::Node::SharedPtr node)
            : Scan_Node::Scan_Node(name, config, node)
        {
            RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Scan_90_Node is working");
        }

    private:
        BT::NodeStatus tick() override;
    };

    /**
     * @brief 关闭扫描节点
    */
    class Scan_off_Node : public Scan_Node
    {
    public:
        Scan_off_Node(const std::string &name, const BT::NodeConfig &config,
                      rclcpp::Node::SharedPtr node)
            : Scan_Node::Scan_Node(name, config, node)
        {
            RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Scan_off_Node is working");
        }

    private:
        BT::NodeStatus tick() override;
    };

    /**
     * @brief 比赛开始条件节点
    */
    class Start_Condition : public BT::ConditionNode
    {
    public:
        Start_Condition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts();

    private:

    /**
     * @brief 获取比赛状态
     * 
     * @param game_start 比赛是否开启
     * 
     * @return 是否开启主循环
    */
        bool GetGameStatus(); // 获取比赛状态,true则证明比赛开始，开启主循环
        BT::NodeStatus tick() override;
    };


    /**
     * @brief 小陀螺条件节点，根据手动命令或是前哨站血量判断是否开启小陀螺
    */
    class Top_Condition : public BT::ConditionNode
    {
    public:
        Top_Condition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts();

    private:

    /**
     * @brief 获取小陀螺状态
     * 
     * @param outpost_blood 前哨站血量
     * @param manual_top 是否手动开启小陀螺
     * 
     * @return 是否开启小陀螺，开启则返回 true
    */
        bool Get_Top_Status(); // 获取小陀螺状态,true则开启小陀螺
        BT::NodeStatus tick() override;
    };

    /**
     * @brief 姿态条件节点，根据小陀螺状态判断当前姿态
     * 
     * @param top_status 小陀螺状态
    */
    class Pose_Condition : public BT::ConditionNode
    {
    public:
        Pose_Condition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts();

    private:

        /**
         * @brief 获取当前姿态
         * 
         * @param top_status 小陀螺状态
         * 
         * @return false为进攻姿态，true为防御姿态
        */
        bool Get_Current_Pose(); 
        BT::NodeStatus tick() override;
    };

    /**
     * @brief 防御姿态下条件节点，综合各种情况进行行为选择
    */
    class Defend_Main_Condition : public BT::ConditionNode
    {
    public:
        Defend_Main_Condition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts();
        // /**
        //  * @brief 获取位置信息
        //  * 
        //  * @param position 位置
        //  * 
        //  * @return PoseStamped 目标位置，四元数类型
        // */
        // geometry_msgs::msg::PoseStamped GetPosition(double goal_position[3][2]);
        int m_goal_position = -1;       // 我的目标地点
        int m_last_position = -1;       // 我上一次的目标点，即本次出发点
        int total_blood = 0;            // 总共恢复血量
        int last_blood = 600;           // 上一时刻哨兵血量
        int if_arrive = 0;              // 是否到达的标志位，与防抖相适应,1为已到达，0为未到达
        int blood_time = 0;             // 累积补血时间
        double goal_position[4][2];     // 储存目的地x,y坐标
        int detect_time;                // 检测时间
        bool m_last_track = false;      // 上一帧是否识别到
        int m_lost_detect_count = 0;    // 多少帧内未识别

    private:
        /**
         * @brief 防御姿态下决定是否开启导航，将综合各种情况计算出当前弹频
         * 
         * @param armor_distance     最近装甲板距离
         * @param armor_number       装甲板数量
         * @param shootable          是否击打
         * @param if_track           是否跟踪
         * @param bullet_num         剩余子弹数量
         * @param navigation_status  当前导航状态
         * @param outpost_blood      前哨站血量
         * @param sentry_blood       哨兵血量 
         * 
         * @return bullet_rate弹频
        */
        int DefendCondition(); 

        /**
         * @brief Defend_Main_Condition判断函数，将弹频与阈值进行比较，小于阈值则返回SUCCESS，
         *        开始导航，否则返回FAILURE，不开启导航。
         * 
         * @param bullet_rate      弹频
        */
        BT::NodeStatus tick() override;
    };

    /**
     * @brief 进攻姿态下条件节点，综合各种情况进行行为选择
    */
    class Attack_Main_Condition : public BT::ConditionNode
    {
    public:
        Attack_Main_Condition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts();

    private:
        /**
         * @brief 进攻姿态下决定是否开启自瞄，将综合各种情况计算出当前弹频
         * 
         * @param armor_distance     最近装甲板距离
         * @param armor_number       装甲板数量
         * @param shootable          是否击打
         * @param if_track           是否跟踪
         * @param bullet_num         剩余子弹数量
         * @param navigation_status  当前导航状态
         * @param outpost_blood      前哨站血量
         * @param sentry_blood       哨兵血量 
         * 
         * @return bullet_rate弹频
        */
        int AttackCondition(); 

        /**
         * @brief Attack_Main_Condition判断函数，将弹频与阈值进行比较，大于阈值则返回SUCCESS，
         *        开启自瞄，否则返回FAILURE，不开启自瞄。
         * 
         * @param bullet_rate      弹频
        */
        BT::NodeStatus tick() override;
    };

    /**
     * @brief 扫描条件节点，根据装甲板识别信息判断开启自瞄或是扫描
    */
    class Scan_Condition : public BT::ConditionNode
    {
    public:
        Scan_Condition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts();

    private:

        /**
         * @brief 获取扫描状态，根据装甲板识别效果判断。True则开启扫描,否则开启自瞄。
         * 
         * @param armor_distance  最近装甲板距离
         * @param armor_number    识别到的装甲板数量
         * @param shootable          是否击打
         * @param if_track           是否跟踪
         * 
         * @return 是否开启扫描
        */ 
        bool GetScanStatus(); 

        /**
         * @brief 根据扫描状态判断开启扫描或是自瞄
        */
        BT::NodeStatus tick() override;
    };

    /**
     * @brief 导航条件节点，根据导航状态判断开启导航或者扫描
    */
    class Navigation_Condition : public BT::ConditionNode
    {
    public:
        Navigation_Condition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts();

    private:

        /**
         * @brief 获取导航状态，根据导航状态判断。True则开启导航,否则开启扫描。
         * 
         * @param navigation_status  当前导航状态
         * 
         * @return 是否开启导航
        */ 
        bool GetNavigationStatus(); 

        /**
         * @brief 根据导航状态判断开启导航或是扫描
        */
        BT::NodeStatus tick() override;
    };

    /**
     * @brief 默认动作条件节点，根据识别信息判断开启返航或者自瞄
    */
    class Back_Condition : public BT::ConditionNode
    {
    public:
        Back_Condition(const std::string &name, const BT::NodeConfig &config);
        static BT::PortsList providedPorts();

    private:

        /**
         * @brief 根据识别信息判断开启返航或者自瞄，True则返航，否则开启自瞄无限制击打
         * 
         * @param armor_distance   最近装甲板距离
         * @param armor_number     识别到的装甲板数量
         * @param shootable          是否击打
         * @param if_track           是否跟踪
         * 
         * @return 是否返航
        */ 
        bool GetBackStatus(); 

        /**
         * @brief 根据返航状态判断开启返航或是自瞄
        */
        BT::NodeStatus tick() override;
    };
}