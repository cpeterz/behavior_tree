#include "behavior_tree/BehaviorTree.hpp"

namespace wmj
{
    void DataReadNode::readParam(std::string path, Read_Param mode)
    {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        switch (mode)
        {
        case ALL:
            RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Get init msg");
            fs["init_armor_number"] >> this->armor_msg.armor_number;
            fs["init_armor_distance"] >> this->armor_msg.armor_distance;
            fs["init_armor_timestamp"] >> this->armor_msg.armor_timestamp;
            fs["init_navigation_timestamp"] >> this->navigation_msg.navigation_timestamp;
            fs["init_navigation_status "] >> this->navigation_msg.navigation_status;
            fs["init_navigation_back"] >> this->navigation_msg.navigation_back;
            fs["init_outpost_blood"] >> this->game_msg.outpost_blood;
            fs["init_sentry_blood"] >> this->game_msg.sentry_blood;
            fs["init_bullet_num"] >> this->game_msg.bullet_num;
            fs["init_time_left"] >> this->game_msg.time_left;
            fs["init_game_timestamp"] >> this->game_msg.game_timestamp;
            fs["init_manual_top"] >> this->game_msg.manual_top;
            break;
        case ARMOR:
            RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Get default-armor msg");
            fs["armor_number"] >> this->armor_msg.armor_number;
            fs["armor_distance"] >> this->armor_msg.armor_distance;
            fs["outpost_blood"] >> this->game_msg.outpost_blood;
            fs["sentry_blood"] >> this->game_msg.sentry_blood;
            fs["bullet_num"] >> this->game_msg.bullet_num;
            fs["time_left"] >> this->game_msg.time_left;
            break;
        case NAVIGATION: 
            RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Get default-navigation msg");
            fs["navigation_status"] >> this->navigation_msg.navigation_status;
            break;
        case GAME:
            RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Get default-game msg");
            fs["manual_top"] >> this->game_msg.manual_top;
            fs["navigation_back"] >> this->navigation_msg.navigation_back;
            break;     
        }
    }

//------------------------------------------------------ DataReadNode ---------------------------------------------------------------    
    DataReadNode::DataReadNode(const std::string &name, const BT::NodeConfig &config,
                               rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "DataReadNode begin working");
        readParam(BT_YAML, ALL);
        sub_armors = node->create_subscription<base_interfaces::msg::Armors>(
            "Armors", 10, std::bind(&DataReadNode::armor_call_back, this, _1));
        sub_game = node->create_subscription<base_interfaces::msg::Game>(
            "Game", 10, std::bind(&DataReadNode::game_call_back, this, _1));
        sub_navigation = node->create_subscription<base_interfaces::msg::Navigation>(
            "Navigation", 10, std::bind(&DataReadNode::navigation_call_back, this, _1));
    }

    void DataReadNode::armor_call_back(const base_interfaces::msg::Armors::SharedPtr msg)
    {
        // 更新装甲板数据
        armor_msg.armor_distance = msg->distance;
        armor_msg.armor_timestamp = msg->time_stamp;
        armor_msg.armor_number = msg->num;
        RCLCPP_INFO(rclcpp::get_logger("get msg"), "Armor msg get");
    }

    void DataReadNode::game_call_back(const base_interfaces::msg::Game::SharedPtr msg)
    {
        // 更新比赛状况信息
        game_msg.bullet_num = msg->bullet_num;
        game_msg.game_timestamp = msg->game_timestamp;
        game_msg.manual_top = msg->manual_top;
        game_msg.outpost_blood = msg->outpost_blood;
        game_msg.time_left = msg->time_left;
        game_msg.sentry_blood = msg->sentry_blood;
        RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Game msg get");
    }

    // 订阅当前导航状态
    void DataReadNode::navigation_call_back(const base_interfaces::msg::Navigation::SharedPtr msg)
    {
        navigation_msg.navigation_status = msg->navigation_status;
        navigation_msg.navigation_timestamp = msg->navigation_timestamp;
        RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Navigation msg get");
    }

    // 定义接口
    BT::PortsList DataReadNode::providedPorts()
    {
        BT::PortsList ports_list;

        ports_list.insert(BT::OutputPort<int>("armor_number"));
        ports_list.insert(BT::OutputPort<double>("armor_distance"));
        ports_list.insert(BT::OutputPort<double>("armor_timestamp"));

        ports_list.insert(BT::OutputPort<double>("navigation_timestamp"));
        ports_list.insert(BT::OutputPort<bool>("navigation_status"));

        ports_list.insert(BT::OutputPort<int>("outpost_blood"));
        ports_list.insert(BT::OutputPort<int>("sentry_blood"));
        ports_list.insert(BT::OutputPort<int>("bullet_num"));
        ports_list.insert(BT::OutputPort<int>("time_left"));
        ports_list.insert(BT::OutputPort<double>("game_timestamp"));
        ports_list.insert(BT::OutputPort<bool>("manual_top"));

        ports_list.insert(BT::OutputPort<bool>("navigation_back"));
        return ports_list;
    }

    BT::NodeStatus DataReadNode::tick()
    {
        rclcpp::Rate loop_rate(10);   
        while (last_game_timestamp == game_msg.game_timestamp)
        {
            // 一秒内未收到消息则使用默认数据
            if (m_waitGameMsgTime > 1000 || game_msg_count == 0)
            {
                game_msg_count++;
                readParam(BT_YAML, GAME);
                break;
            }
            loop_rate.sleep();    // 等待 100ms
            m_waitGameMsgTime += 100 ;
            RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Waiting for Game Msg %lf ms", m_waitGameMsgTime);
        }
        if (last_game_timestamp != game_msg.game_timestamp)
        {
            m_waitGameMsgTime = 0;
        }

        while (last_armor_timestamp == armor_msg.armor_timestamp)
        {
            if (m_waitArmorMsgTime > 1000 || armor_msg_count == 0)
            {
                armor_msg_count++;
                readParam(BT_YAML, ARMOR);
                break;
            }
            loop_rate.sleep();    // 等待 100ms
            m_waitArmorMsgTime += 100 ;
            RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Waiting for Armor Msg %lf ms", m_waitArmorMsgTime);
        }
        if (last_armor_timestamp != armor_msg.armor_timestamp)
        {
            m_waitArmorMsgTime = 0;
        }

        std::cout << "armor_number:" << armor_msg.armor_number << std::endl;
        std::cout << "armor_distance:" << armor_msg.armor_distance << std::endl;

        while (last_navigation_timestamp == navigation_msg.navigation_timestamp)
        {
            if (m_waitNavigationMsgTime > 1000 || navigation_msg_count == 0)
            {
                navigation_msg_count++;
                readParam(BT_YAML, NAVIGATION);
                break;
            }
            loop_rate.sleep();    // 等待 100ms
            m_waitNavigationMsgTime += 100 ;
            RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Waiting for Navigation Msg %lf ms", m_waitNavigationMsgTime);
        }
        if (last_navigation_timestamp != navigation_msg.navigation_timestamp)
        {
            m_waitNavigationMsgTime = 0;
            // 导航信息更新中则不自动返回巡逻区
            navigation_msg.navigation_back = false;
        }
        
        last_armor_timestamp = armor_msg.armor_timestamp;
        last_game_timestamp = game_msg.game_timestamp;
        last_navigation_timestamp = navigation_msg.navigation_timestamp;

        setOutput("armor_number", armor_msg.armor_number);
        setOutput("armor_distance", armor_msg.armor_distance);
        setOutput("armor_timestamp", armor_msg.armor_timestamp);
        
        setOutput("navigation_timestamp", navigation_msg.navigation_timestamp);
        setOutput("navigation_status", navigation_msg.navigation_status);
        setOutput("navigation_back", navigation_msg.navigation_back);

        setOutput("outpost_blood", game_msg.outpost_blood);
        setOutput("sentry_blood", game_msg.sentry_blood);
        setOutput("bullet_num", game_msg.bullet_num);
        setOutput("time_left", game_msg.time_left);
        setOutput("game_timestamp", game_msg.game_timestamp);
        setOutput("manual_top", game_msg.manual_top);

        return BT::NodeStatus::SUCCESS;
    }

//------------------------------------------------------ Top_Node ---------------------------------------------------------------    
    
    /**
     * @brief 小陀螺基类,子类包括开启小陀螺节点和关闭小陀螺节点
    */
    Top_Node::Top_Node(const std::string &name, const BT::NodeConfig &config,
                       rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        topPub = node->create_publisher<base_interfaces::msg::BtTop>("BT_top", 10);
    }

    BT::PortsList Top_Node::providedPorts()
    {
        BT::PortsList port_lists;
        port_lists.insert(BT::OutputPort<bool>("top_status"));
        return port_lists;
    }
   
    /**
     * @brief 开启小陀螺节点
     * 
    */
    BT::NodeStatus Top_on_Node::tick()
    {
        msg.start = true;
        msg.top_timestamp = wmj::now();
        setOutput<bool>("top_status", msg.start);
        topPub->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
    
    /**
     * @brief 关闭小陀螺节点
    */
    BT::NodeStatus Top_off_Node::tick()
    {
        msg.start = false;
        msg.top_timestamp = wmj::now();
        setOutput<bool>("top_status", msg.start);
        topPub->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

//------------------------------------------------------ Shoot_Node ---------------------------------------------------------------    
    
    /**
     * @brief 发射基类，子类包括 限制性发射节点 ， 无限制发射节点 ， 不发射节点
    */
    Shoot_Node::Shoot_Node(const std::string &name, const BT::NodeConfig &config,
                           rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        pub_shooter = node->create_publisher<base_interfaces::msg::BtAimer>("BT_shooter", 10);
    }

    BT::PortsList Shoot_Node::providedPorts()
    {
        BT::PortsList port_list;
        port_list.insert(BT::InputPort<int>("bullet_rate"));
        return port_list;
    }
    
    /**
     * @brief 无限制发射节点
     * 
     * @return 最大弹频
    */
    BT::NodeStatus No_Limit_Shoot_Node::tick()
    {
        msg.bullet_rate = 20; // 最大弹速
        msg.btaimer_timestamp = wmj::now();
        pub_shooter->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
    
    /**
     * @brief 限制性发射节点
     * 
     * @return bullet_rate 弹频
    */
    BT::NodeStatus Limit_Shoot_Node::tick()
    {
        auto bullet_rate = getInput<int>("bullet_rate");
        msg.bullet_rate = bullet_rate.value();
        msg.btaimer_timestamp = wmj::now();
        pub_shooter->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
    
    /**
     * @brief 不发射节点
    */
    BT::NodeStatus No_Shoot_Node::tick()
    {
        msg.bullet_rate = -1;
        msg.btaimer_timestamp = wmj::now();
        pub_shooter->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

//------------------------------------------------------ Navigation_Node ---------------------------------------------------------------    
    
    /**
     * @brief 导航基类，子类包括 开启导航节点，关闭导航节点，自动返航节点
    */
    Navigation_Node::Navigation_Node(const std::string &name, const BT::NodeConfig &config,
                                     rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        navigationPub = node->create_publisher<base_interfaces::msg::BtNavigation>("BT_navigation", 10);
    }
    
    BT::PortsList Navigation_Node::providedPorts()
    {
        BT::PortsList port_lists;

        port_lists.insert(BT::InputPort<bool>("navigation_back"));

        return port_lists;
    }

    /**
     * @brief 开启导航节点
     * 
     * @param navigation_back 是否返航
    */
    BT::NodeStatus Navigation_on_Node::tick()
    {
        auto navigation_back = getInput<bool>("navigation_back");

        if (!navigation_back)
        {
            throw BT::RuntimeError("detect_node missing required input [message]:",
                                   navigation_back.error());
        }

        msg.navigation_back = navigation_back.value();
        msg.navigation_continue = true;
        msg.bt_navigation_timestamp = wmj::now();

        navigationPub->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
    
    /**
     * @brief 导航关闭节点
    */
    BT::NodeStatus Navigation_off_Node::tick()
    {
        msg.navigation_continue = false;
        msg.navigation_back = false;
        msg.bt_navigation_timestamp = wmj::now();

        navigationPub->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief 返航节点
    */
    BT::NodeStatus Navigation_back_Node::tick()
    {
        msg.navigation_back = true;
        msg.navigation_continue = true;
        msg.bt_navigation_timestamp = wmj::now();
        
        navigationPub->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

//------------------------------------------------------ Scan_Node ---------------------------------------------------------------    
    
    /**
     * @brief 扫描基类
    */
    Scan_Node::Scan_Node(const std::string &name, const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        ScanPub = node->create_publisher<base_interfaces::msg::BtScan>("BtScan", 10);
    }

    BT::PortsList Scan_Node::providedPorts()
    {
        BT::PortsList port_lists;
        return port_lists;
    }
    
    /**
     * @brief 开启扫描节点
    */
    BT::NodeStatus Scan_on_Node::tick()
    {
        msg.scan_on = true;
        msg.scan_timestamp = wmj::now();
        ScanPub->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief 关闭扫描节点
    */
    BT::NodeStatus Scan_off_Node::tick()
    {
        msg.scan_on = false;
        msg.scan_timestamp = wmj::now();
        ScanPub->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

// ------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------               Condition               --------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------ Navigation_Condition ---------------------------------------------------------------    
    /**
     * @brief 导航条件节点，根据导航状态判断开启导航或者扫描
    */
    Navigation_Condition::Navigation_Condition(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
        RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Navigation_Condition Node is working");
    }

    BT::PortsList Navigation_Condition::providedPorts()
    {
        BT::PortsList port_lists;
        port_lists.insert(BT::InputPort<bool>("navigation_status"));
        return port_lists;
    }
    
    /**
     * @brief 获取导航状态，根据导航状态判断。True则开启导航,否则开启扫描。
     * 
     * @param navigation_status  当前导航状态
     * 
     * @return 是否开启导航
     */ 
    bool Navigation_Condition::GetNavigationStatus()
    {
        auto navigation_status = getInput<bool>("navigation_status");
        if(!navigation_status)
        {
            throw BT::RuntimeError("navigation_condition missing required input [message]:",
                                   navigation_status.error());
        }

        if(navigation_status.value())
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    
    /**
     * @brief 根据导航状态判断开启导航或是扫描
     */
    BT::NodeStatus Navigation_Condition::tick()
    {
        bool navigation_status = GetNavigationStatus();
        if(navigation_status)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

//------------------------------------------------------ Top_Condition ---------------------------------------------------------------    
    // 小陀螺条件节点
    /**
     * @brief 小陀螺条件节点，根据手动命令或是前哨站血量判断是否开启小陀螺
    */
    Top_Condition::Top_Condition(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
        RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Top_Condition Node is working");
    }

    BT::PortsList Top_Condition::providedPorts()
    {
        BT::PortsList port_lists;

        port_lists.insert(BT::InputPort<int>("outpost_blood"));
        port_lists.insert(BT::InputPort<bool>("manual_top"));

        return port_lists;
    }
    
    /**
     * @brief 获取小陀螺状态
     * 
     * @param outpost_blood 前哨站血量
     * @param manual_top 是否手动开启小陀螺
     * 
     * @return 是否开启小陀螺，开启则返回 true
    */
    bool Top_Condition::Get_Top_Status()
    {
        auto outpost_blood = getInput<int>("outpost_blood");
        auto manual_top = getInput<bool>("manual_top");

        if (!outpost_blood)
        {
            throw BT::RuntimeError("detect_node missing required input [message]:",
                                   outpost_blood.error());
        }

        if (manual_top.value() || outpost_blood.value() <= 500)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
   
    BT::NodeStatus Top_Condition::tick()
    {
        bool top_status = Get_Top_Status();
        if (top_status)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

//------------------------------------------------------ Pose_Condition ---------------------------------------------------------------    
    
    /**
     * @brief 姿态条件节点，根据小陀螺状态判断当前姿态
     * 
     * @param top_status 小陀螺状态
     */
    Pose_Condition::Pose_Condition(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
        RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Pose_Condition Node is working");
    }

    BT::PortsList Pose_Condition::providedPorts()
    {
        BT::PortsList port_lists;
        port_lists.insert(BT::InputPort<bool>("top_status"));
        return port_lists;
    }

    bool Pose_Condition::Get_Current_Pose()
    {
        auto current_pose = getInput<bool>("top_status");

        if (!current_pose)
        {
            throw BT::RuntimeError("detect_node missing required input [message]:",
                                   current_pose.error());
        }

        if (current_pose.value())
        {
            return true; // 进入防御姿态
        }
        else
        {
            return false;
        }
    }

    BT::NodeStatus Pose_Condition::tick()
    {
        bool top_status = Get_Current_Pose();
        if (top_status)
        {
            return BT::NodeStatus::SUCCESS; 
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
//------------------------------------------------------ Scan_Condition ---------------------------------------------------------------    
    /**
     * @brief 扫描条件节点，根据装甲板识别信息判断开启自瞄或是扫描
    */
    Scan_Condition::Scan_Condition(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
        RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Scan_Condition Node is working");
    }

    BT::PortsList Scan_Condition::providedPorts()
    {
        BT::PortsList port_lists;
        port_lists.insert(BT::InputPort<double>("armor_distance"));
        port_lists.insert(BT::InputPort<int>("armor_number"));
        return port_lists;
    }
    
    /**
     * @brief 获取扫描状态，根据装甲板识别效果判断。True则开启扫描,否则开启自瞄。
     * 
     * @param armor_distance  最近装甲板距离
     * @param armor_number    识别到的装甲板数量
     * 
     * @return 是否开启扫描
    */ 
    bool Scan_Condition::GetScanStatus()
    {
        auto armor_number = getInput<int>("armor_number");
        auto armor_distance = getInput<double>("armor_distance");

        if (!armor_distance)
        {
            throw BT::RuntimeError("Scan Condition missing required input [message]:",
                                   armor_distance.error());
        }
        
        // 若在七米内识别到装甲板，则开启自瞄无限制击打，否则开启扫描
        if (armor_number.value() > 0 && armor_distance.value() < 700)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    /**
     * @brief 根据扫描状态判断开启扫描或是自瞄
    */
    BT::NodeStatus Scan_Condition::tick()
    {
        bool scan_status = GetScanStatus();
        if (scan_status)
        {
            return BT::NodeStatus::SUCCESS; 
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

//------------------------------------------------------ Back_Condition ---------------------------------------------------------------    
 
    /**
     * @brief 默认动作条件节点，根据识别信息判断开启返航或者自瞄
    */
    Back_Condition::Back_Condition(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
        RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Back_Condition Node is working");
    }

    BT::PortsList Back_Condition::providedPorts()
    {
        BT::PortsList port_lists;
        port_lists.insert(BT::InputPort<double>("armor_distance"));
        port_lists.insert(BT::InputPort<int>("armor_number"));
        return port_lists;
    }
    
    /**
     * @brief 根据识别信息判断开启返航或者自瞄，True则返航，否则开启自瞄无限制击打
     * 
     * @param armor_distance   最近装甲板距离
     * @param armor_number     识别到的装甲板数量
     * 
     * @return 是否返航
    */ 
    bool Back_Condition::GetBackStatus()
    {
        auto armor_number = getInput<int>("armor_number");
        auto armor_distance = getInput<double>("armor_distance");

        if (!armor_distance)
        {
            throw BT::RuntimeError("Back Condition missing required input [message]:",
                                   armor_distance.error());
        }
        
        // 若在两米内识别到装甲板，则开启自瞄无限制击打，否则返航
        if (armor_number.value() > 0 && armor_distance.value() < 2000)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    /**
     * @brief 根据返航状态判断开启返航或是自瞄
    */
    BT::NodeStatus Back_Condition::tick()
    {
        bool back_status = GetBackStatus();
        if (back_status)
        {
            return BT::NodeStatus::SUCCESS; 
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

//------------------------------------------------------ Attack_Main_Condition ---------------------------------------------------------------    

    /**
     * @brief 进攻姿态下条件节点，综合各种情况进行行为选择
    */
    Attack_Main_Condition::Attack_Main_Condition(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
        RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Attack_Main_Condition Node is working");
    }

    BT::PortsList Attack_Main_Condition::providedPorts()
    {
        BT::PortsList port_lists;

        port_lists.insert(BT::InputPort<int>("armor_number"));
        port_lists.insert(BT::InputPort<double>("armor_distance"));

        port_lists.insert(BT::InputPort<int>("sentry_blood"));
        port_lists.insert(BT::InputPort<int>("bullet_num"));
        port_lists.insert(BT::InputPort<int>("time_left"));
        port_lists.insert(BT::InputPort<int>("outpost_blood"));
        port_lists.insert(BT::InputPort<bool>("navigation_status"));

        port_lists.insert(BT::OutputPort<int>("bullet_rate"));
        return port_lists;
    }
    
    /**
     * @brief 进攻姿态下决定是否开启自瞄，将综合各种情况计算出当前弹频
     * 
     * @param armor_distance     最近装甲板距离
     * @param armor_number       装甲板数量
     * @param bullet_num         剩余子弹数量
     * @param navigation_status  当前导航状态
     * @param outpost_blood      前哨站血量
     * @param sentry_blood       哨兵血量 
     * 
     * @return bullet_rate弹频
    */
    int Attack_Main_Condition::AttackCondition()
    {
        auto armor_number = getInput<int>("armor_number");
        auto armor_distance = getInput<double>("armor_distance");
        auto sentry_blood = getInput<int>("sentry_blood");
        auto bullet_num = getInput<int>("bullet_num");
        auto time_left = getInput<int>("time_left");
        auto outpost_blood = getInput<int>("outpost_blood");
        auto navigation_status = getInput<bool>("navigation_status");


        if (!armor_number)
        {
            throw BT::RuntimeError("detect_node missing required input [message]:",
                                   armor_number.error());
        }
 
        // int bullet_rate = (int)(-10 + sqrt(time_left.value()) + 0.3 * sqrt(bullet_num.value()) -
        //                         (armor_distance.value() / 100) * (armor_distance.value() / 100) * (armor_distance.value() / 100)); 

        int bullet_rate = (int)(15 - armor_distance.value()/100);
        if(armor_distance == 0)
        {
            bullet_rate = 0;
        }

    
        
        // 导航状态对判断条件进行削减并对确定后条件进行增强
        if( navigation_status )
        {
            if(bullet_rate*0.75 > 5)
            {
                bullet_rate = bullet_rate*1.25;
            }
            else
            {
                bullet_rate = -1;
            }
        }
        
        if(bullet_rate > 20)
        {
            bullet_rate = 20;
        }
        std::cout << "attack_bullet_rate:" << bullet_rate << std::endl;
        setOutput("bullet_rate", bullet_rate);
        return bullet_rate;
    }

    /**
     * @brief Attack_Main_Condition判断函数，将弹频与阈值进行比较，大于阈值则返回SUCCESS，
     *        开启自瞄，否则返回FAILURE，不开启自瞄。
     * 
     * @param bullet_rate      弹频
    */
    BT::NodeStatus Attack_Main_Condition::tick()
    {
        int bullet_rate = AttackCondition();
        if (bullet_rate > 5)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }


//------------------------------------------------------ Defend_Main_Condition ---------------------------------------------------------------    

    /**
     * @brief 防御姿态下条件节点，综合各种情况进行行为选择
    */
    Defend_Main_Condition::Defend_Main_Condition(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
        RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Defend_Main_Condition Node is working");
    }

    BT::PortsList Defend_Main_Condition::providedPorts()
    {
        BT::PortsList port_lists;

        port_lists.insert(BT::InputPort<int>("armor_number"));
        port_lists.insert(BT::InputPort<double>("armor_distance"));

        port_lists.insert(BT::InputPort<int>("sentry_blood"));
        port_lists.insert(BT::InputPort<int>("bullet_num"));
        port_lists.insert(BT::InputPort<int>("time_left"));
        port_lists.insert(BT::InputPort<int>("outpost_blood"));
        port_lists.insert(BT::InputPort<bool>("navigation_status"));

        port_lists.insert(BT::OutputPort<int>("bullet_rate"));
        return port_lists;
    }

    /**
     * @brief 防御姿态下决定是否开启导航，将综合各种情况计算出当前弹频
     * 
     * @param armor_distance     最近装甲板距离
     * @param armor_number       装甲板数量
     * @param bullet_num         剩余子弹数量
     * @param navigation_status  当前导航状态
     * @param outpost_blood      前哨站血量
     * @param sentry_blood       哨兵血量 
     * 
     * @return bullet_rate弹频
    */
    int Defend_Main_Condition::DefendCondition()
    {
        auto armor_number = getInput<int>("armor_number");
        auto armor_distance = getInput<double>("armor_distance");
        auto sentry_blood = getInput<int>("sentry_blood");
        auto bullet_num = getInput<int>("bullet_num");
        auto time_left = getInput<int>("time_left");
        auto outpost_blood = getInput<int>("outpost_blood");
        auto navigation_status = getInput<bool>("navigation_status");

        if (!armor_number)
        {
            throw BT::RuntimeError("detect_node missing required input [message]:",
                                   armor_number.error());
        }

        // int bullet_rate = (int)(-10 + sqrt(time_left.value()) + 0.3 * sqrt(bullet_num.value()) -
        //                         (armor_distance.value() / 100) * (armor_distance.value() / 100) * (armor_distance.value() / 100)); 

        int bullet_rate = (int)(15 - armor_distance.value() / 100);
        if(armor_distance == 0)
        {
            bullet_rate = 0;
        }
        if(navigation_status)
        {
            if( bullet_rate*0.8 > 10)
            {
                bullet_rate = 20;
            }
            else
            { 
                bullet_rate = -1;
            }
        }
        
        if(bullet_rate > 20)
        {
            bullet_rate = 20;
        }
        std::cout << "defend_bullet_rate:" << bullet_rate << std::endl;
        setOutput("bullet_rate", bullet_rate);
        return bullet_rate;
    }

    /**
     * @brief Defend_Main_Condition判断函数，将弹频与阈值进行比较，小于阈值则返回SUCCESS，
     *        开始导航，否则返回FAILURE，不开启导航。
     * 
     * @param bullet_rate    弹频
    */
    BT::NodeStatus  Defend_Main_Condition::tick()
    {
        int bullet_rate = DefendCondition();
        if (bullet_rate > 10)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    // 共用rclcpp::Node节点
    auto node = std::make_shared<rclcpp::Node>("topic");

    factory.registerNodeType<wmj::DataReadNode>("DataReadNode", node);
    factory.registerNodeType<wmj::No_Shoot_Node>("No_Shoot_Node", node);
    factory.registerNodeType<wmj::Limit_Shoot_Node>("Limit_Shoot_Node", node);
    factory.registerNodeType<wmj::No_Limit_Shoot_Node>("No_Limit_Shoot_Node", node);
    factory.registerNodeType<wmj::Navigation_on_Node>("Navigation_on_Node", node);
    factory.registerNodeType<wmj::Navigation_off_Node>("Navigation_off_Node", node);
    // factory.registerNodeType<wmj::Navigation_back_Node>("Navigation_back_Node", node);
    factory.registerNodeType<wmj::Top_on_Node>("Top_on_Node", node);
    factory.registerNodeType<wmj::Top_off_Node>("Top_off_Node", node);
    factory.registerNodeType<wmj::Scan_on_Node>("Scan_on_Node", node);
    factory.registerNodeType<wmj::Scan_off_Node>("Scan_off_Node", node);

    factory.registerNodeType<wmj::Top_Condition>("Top_Condition");
    factory.registerNodeType<wmj::Navigation_Condition>("Navigation_Condition");
    factory.registerNodeType<wmj::Pose_Condition>("Pose_Condition");
    factory.registerNodeType<wmj::Back_Condition>("Back_Condition");
    factory.registerNodeType<wmj::Scan_Condition>("Scan_Condition");
    factory.registerNodeType<wmj::Attack_Main_Condition>("Attack_Main_Condition");
    factory.registerNodeType<wmj::Defend_Main_Condition>("Defend_Main_Condition");
    
    auto tree = factory.createTreeFromFile(BT_XML);

    // 调试工具
    // BT::StdCoutLogger std_count_logger(tree);
    //  BT::FileLogger logger_file(tree,"bt_trace.fbl");
    //  BT::MinitraceLogger logger_minitrace(tree,"bt_trace.json");
    BT::PublisherZMQ publisher_zmq(tree);
    // BT::printTreeRecursively(tree.rootNode());
    // tree.subtrees[0]->blackboard->debugMessage();
   
    rclcpp::Rate loop_rate(2);
    while (rclcpp::ok())
    {
        tree.tickWhileRunning();
        loop_rate.sleep();
        rclcpp::spin_some(node);
    }
    return 0;
}
