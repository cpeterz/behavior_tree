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
            fs["init_aimer_if_track"] >> this->aimer_msg.aimer_if_track;
            fs["init_aimer_shootable"] >> this->aimer_msg.aimer_shootable;
            fs["init_aimer_timestamp"] >> this->aimer_msg.aimer_timestamp;
            fs["init_navigation_timestamp"] >> this->navigation_msg.navigation_timestamp;
            fs["init_outpost_blood"] >> this->game_msg.outpost_blood;
            fs["init_sentry_blood"] >> this->game_msg.sentry_blood;
            fs["init_bullet_num"] >> this->game_msg.bullet_num;
            fs["init_time_left"] >> this->game_msg.time_left;
            fs["init_game_timestamp"] >> this->game_msg.game_timestamp;
            fs["init_manual_top"] >> this->game_msg.manual_top;
            fs["init_game_start"] >> this->game_msg.game_start;
            fs["init_navigation_status"] >> this->navigation_msg.navigation_status;
            fs["init_m_alive"] >> this->game_msg.m_alive;
            fs["init_enemy_alive"] >> this->game_msg.enemy_alive;
            fs["init_position"] >> this->navigation_msg.navigation_default_position;
            fs["init_hityaw"] >> this->game_msg.hityaw;
            fs["init_shootable"] >> this->aimer_msg.aimer_shootable;
            fs["init_if_track"] >> this->aimer_msg.aimer_if_track;
            fs["init_if_navigation"] >> this->if_navigation;
            fs["init_debug_game"] >> this->debug_game;
            fs["init_debug_all"] >> this->debug_all;
            break;
        case ARMOR:
            // fs["armor_number"] >> this->armor_msg.armor_number;
            // fs["armor_distance"] >> this->armor_msg.armor_distance;
            // fs["shootable"] >> this->aimer_msg.aimer_shootable;
            // fs["if_track"] >> this->aimer_msg.aimer_if_track;
            RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Get default-armor msg");
            break;
        case SHOOT:
            fs["armor_number"] >> this->armor_msg.armor_number;
            fs["armor_distance"] >> this->armor_msg.armor_distance;
            fs["shootable"] >> this->aimer_msg.aimer_shootable;
            fs["if_track"] >> this->aimer_msg.aimer_if_track;
            RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Get default-aimer msg");
            break;
        case NAV: 
            // fs["position"] >> this->navigation_msg.navigation_default_position;
            fs["navigation_status"] >> this->navigation_msg.navigation_status;
            RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Get default-navigation msg");
            break;
        case GAME:
            fs["manual_top"] >> this->game_msg.manual_top;
            fs["game_start"] >> this->game_msg.game_start;
            fs["m_alive"] >> this->game_msg.m_alive;
            fs["enemy_alive"] >> this->game_msg.enemy_alive;
            fs["position"] >> this->navigation_msg.navigation_default_position;
            fs["hityaw"] >> this->game_msg.hityaw; 
            fs["outpost_blood"] >> this->game_msg.outpost_blood;
            fs["sentry_blood"] >> this->game_msg.sentry_blood;
            fs["bullet_num"] >> this->game_msg.bullet_num;
            fs["time_left"] >> this->game_msg.time_left;
            RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Get default-game msg");
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
            "Armors", 1, std::bind(&DataReadNode::armor_call_back, this, _1));
        sub_aimer = node->create_subscription<base_interfaces::msg::Aimer>(
            "Shoot", 1, std::bind(&DataReadNode::aimer_call_back, this, _1));
        sub_game = node->create_subscription<base_interfaces::msg::Game>(
            "Game", 1, std::bind(&DataReadNode::game_call_back, this, _1));
        sub_navigation = node->create_subscription<base_interfaces::msg::Navigation>(
            "Navigation", 1, std::bind(&DataReadNode::navigation_call_back, this, _1));
    }

    void DataReadNode::armor_call_back(const base_interfaces::msg::Armors::SharedPtr msg)
    {
        // 更新装甲板数据
        armor_msg.armor_distance = msg->distance;
        armor_msg.armor_timestamp = msg->time_stamp;
        armor_msg.armor_number = msg->num;
        RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Armor msg get");
    }

    void DataReadNode::aimer_call_back(const base_interfaces::msg::Aimer::SharedPtr msg)
    {
        // 更新装甲板数据
        aimer_msg.aimer_shootable = msg->shootable;
        aimer_msg.aimer_if_track = msg->is_track;
        std::cout << "aimer_if_track:" <<  aimer_msg.aimer_if_track << std::endl;
        aimer_msg.aimer_timestamp = msg->timestamp;
        RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Aimer msg get");
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
        game_msg.game_start = msg->game_start;
        game_msg.hityaw = msg->hityaw;
        RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Game msg get");
    }

    // 订阅当前导航状态
    void DataReadNode::navigation_call_back(const base_interfaces::msg::Navigation::SharedPtr msg)
    {
        navigation_msg.navigation_cur_position_x = msg->cur_position.pose.position.x;
        navigation_msg.navigation_cur_position_y = msg->cur_position.pose.position.y;
        navigation_msg.navigation_cur_position_z = msg->cur_position.pose.position.z;
        navigation_msg.navigation_cur_orientation_x = msg->cur_position.pose.orientation.x;
        navigation_msg.navigation_cur_orientation_y = msg->cur_position.pose.orientation.y;
        navigation_msg.navigation_cur_orientation_z = msg->cur_position.pose.orientation.z;
        navigation_msg.navigation_cur_orientation_w = msg->cur_position.pose.orientation.w;
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

        ports_list.insert(BT::OutputPort<int>("aimer_if_track"));
        ports_list.insert(BT::OutputPort<bool>("aimer_shootable"));

        ports_list.insert(BT::OutputPort<int>("outpost_blood"));
        ports_list.insert(BT::OutputPort<int>("sentry_blood"));
        ports_list.insert(BT::OutputPort<int>("bullet_num"));
        ports_list.insert(BT::OutputPort<int>("time_left"));
        ports_list.insert(BT::OutputPort<double>("game_timestamp"));
        ports_list.insert(BT::OutputPort<bool>("manual_top"));
        ports_list.insert(BT::OutputPort<bool>("game_start"));
        ports_list.insert(BT::OutputPort<int>("m_alive"));
        ports_list.insert(BT::OutputPort<int>("enemy_alive"));
        ports_list.insert(BT::OutputPort<double>("hityaw"));

        ports_list.insert(BT::OutputPort<double>("navigation_cur_position_x"));
        ports_list.insert(BT::OutputPort<double>("navigation_cur_position_y"));
        ports_list.insert(BT::OutputPort<double>("navigation_cur_position_z"));
        ports_list.insert(BT::OutputPort<double>("navigation_cur_orientation_x"));
        ports_list.insert(BT::OutputPort<double>("navigation_cur_orientation_y"));
        ports_list.insert(BT::OutputPort<double>("navigation_cur_orientation_z"));
        ports_list.insert(BT::OutputPort<double>("navigation_cur_orientation_w"));
        ports_list.insert(BT::OutputPort<double>("navigation_timestamp"));
        ports_list.insert(BT::OutputPort<int>("navigation_status"));
        ports_list.insert(BT::OutputPort<int>("default_position"));
        ports_list.insert(BT::OutputPort<int>("navigation_timeout"));

        ports_list.insert(BT::OutputPort<int>("scan_mode"));

        return ports_list;
    }

    BT::NodeStatus DataReadNode::tick()
    {
        navigation_msg.navigation_default_position = -1;      // 默认导航位置在这里不改变，后面若收不到导航或者比赛信息将更新此值
        rclcpp::Rate loop_rate(10);   
        if( !debug_all )
        {
            while (last_armor_timestamp == armor_msg.armor_timestamp)
            {
                if ( armor_msg_count == 0 )
                {
                    armor_msg_count++;
                    break;
                }
                if (m_waitArmorMsgTime > 1000 )
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
                m_waitAimerMsgTime = 0;
            }

            while (last_aimer_timestamp == aimer_msg.aimer_timestamp)
            {
                if ( aimer_msg_count == 0 )
                {
                    aimer_msg_count++;
                    break;
                }
                if (m_waitAimerMsgTime > 1000 )
                {
                    aimer_msg_count++;
                    readParam(BT_YAML, SHOOT);
                    break;
                }
                loop_rate.sleep();    // 等待 100ms
                m_waitAimerMsgTime += 100 ;
                RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Waiting for Aimer Msg %lf ms", m_waitAimerMsgTime);
            }
            if (last_aimer_timestamp != aimer_msg.aimer_timestamp)
            {
                m_waitAimerMsgTime = 0;
            }

            while (last_navigation_timestamp == navigation_msg.navigation_timestamp)
            {
                if(navigation_msg_count == 0)
                {
                    navigation_msg_count++;
                    break;
                }
                if (m_waitNavigationMsgTime > 1000)
                {
                    navigation_msg_count++;
                    readParam(BT_YAML, NAV);
                    break;
                }
                loop_rate.sleep();    // 等待 100ms
                m_waitNavigationMsgTime += 100 ;
                RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Waiting for rqt Navigation Msg %lf ms", m_waitNavigationMsgTime);
            }
            if (last_navigation_timestamp != navigation_msg.navigation_timestamp)
            {
                m_waitNavigationMsgTime = 0;
            }
            if( !debug_game )
            {
                while (last_game_timestamp == game_msg.game_timestamp)
                {
                    if( game_msg_count == 0 )
                    {
                        game_msg_count++;
                        break;
                    }
                    // 一秒内未收到消息则使用默认数据
                    if (m_waitGameMsgTime > 1000)
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
            }
            else
            {
                RCLCPP_INFO(rclcpp::get_logger("INFO"), " debug game !!!!!!!!!!!!!");
                while ( game_msg.game_timestamp == 1 )
                {
                    if( game_msg_count == 0 )
                    {
                        game_msg_count++;
                        break;
                    }
                    // 一秒内未收到消息则使用默认数据
                    if (m_waitGameMsgTime > 1000)
                    {
                        game_msg_count++;
                        readParam(BT_YAML, GAME);
                        break;
                    }
                    loop_rate.sleep();    // 等待 100ms
                    m_waitGameMsgTime += 100 ;
                    RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Waiting for rqt Game Msg %lf ms", m_waitGameMsgTime);
                }
                if ( game_msg.game_timestamp != 1 )
                {
                    m_waitGameMsgTime = 0;
                    RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Get debug Game info");
                }
            }
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("INFO"), " debug all !!!!!!!!!!!!!");
            while (armor_msg.armor_timestamp == 1)
            {
                if ( armor_msg_count == 0 )
                {
                    armor_msg_count++;
                    break;
                }
                if (m_waitArmorMsgTime > 1000 )
                {
                    armor_msg_count++;
                    readParam(BT_YAML, ARMOR);
                    break;
                }
                loop_rate.sleep();    // 等待 100ms
                m_waitArmorMsgTime += 100 ;
                RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Waiting for Armor Msg %lf ms", m_waitArmorMsgTime);
            }
            if ( armor_msg.armor_timestamp != 1 )
            {
                m_waitAimerMsgTime = 0;
                RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Get debug Armor info");
            }

            while ( aimer_msg.aimer_timestamp == 1 )
            {
                if ( aimer_msg_count == 0 )
                {
                    aimer_msg_count++;
                    break;
                }
                if (m_waitAimerMsgTime > 1000 )
                {
                    aimer_msg_count++;
                    readParam(BT_YAML, SHOOT);
                    break;
                }
                loop_rate.sleep();    // 等待 100ms
                m_waitAimerMsgTime += 100 ;
                RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Waiting for Aimer Msg %lf ms", m_waitAimerMsgTime);
            }
            if ( aimer_msg.aimer_timestamp != 1 )
            {
                m_waitAimerMsgTime = 0;
                RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Get debug Aimer info");
            }

            while ( navigation_msg.navigation_timestamp == 1 )
            {
                if(navigation_msg_count == 0)
                {
                    navigation_msg_count++;
                    break;
                }
                if (m_waitNavigationMsgTime > 1000)
                {
                    navigation_msg_count++;
                    readParam(BT_YAML, NAV);
                    break;
                }
                loop_rate.sleep();    // 等待 100ms
                m_waitNavigationMsgTime += 100 ;
                RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Waiting for rqt Navigation Msg %lf ms", m_waitNavigationMsgTime);
            }
            if ( navigation_msg.navigation_timestamp != 1 )
            {
                m_waitNavigationMsgTime = 0;
                RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Get debug Navigation info");
            }
            while ( game_msg.game_timestamp == 1 )
            {
                if( game_msg_count == 0 )
                {
                    game_msg_count++;
                    break;
                }
                // 一秒内未收到消息则使用默认数据
                if (m_waitGameMsgTime > 1000)
                {
                    game_msg_count++;
                    readParam(BT_YAML, GAME);
                    break;
                }
                loop_rate.sleep();    // 等待 100ms
                m_waitGameMsgTime += 100 ;
                RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Waiting for rqt Game Msg %lf ms", m_waitGameMsgTime);
            }
            if ( game_msg.game_timestamp != 1)
            {
                m_waitGameMsgTime = 0;
                RCLCPP_INFO(rclcpp::get_logger("WAIT INFO"), "Get debug Game info");
            }
        }
        // std::cout << "armor_msg.armor_timestamp:" << armor_msg.armor_timestamp << std::endl;
        // std::cout << "navigation_msg.navigation_timestamp:" << navigation_msg.navigation_timestamp << std::endl;
        // std::cout << "aimer_msg.aimer_timestamp" << aimer_msg.aimer_timestamp << std::endl;
        // std::cout << "game_msg.game_timestamp" << game_msg.game_timestamp << std::endl;

        last_armor_timestamp = armor_msg.armor_timestamp;
        last_game_timestamp = game_msg.game_timestamp;
        last_navigation_timestamp = navigation_msg.navigation_timestamp;
        last_aimer_timestamp = aimer_msg.aimer_timestamp;

        if( !if_navigation )
        {
            navigation_msg.navigation_status = 2;
        }
        int scan_mode = 0;

        if( first_time )
        {
            int navigation_timeout = 0;
            first_time = false;
            setOutput("navigation_timeout", navigation_timeout);
        }
        
        setOutput("default_position", navigation_msg.navigation_default_position);

        setOutput("armor_number", armor_msg.armor_number);
        setOutput("armor_distance", armor_msg.armor_distance);
        setOutput("armor_timestamp", armor_msg.armor_timestamp);

        setOutput("aimer_if_track", aimer_msg.aimer_if_track);
        setOutput("aimer_shootable", aimer_msg.aimer_shootable);
        setOutput("aimer_timestamp", aimer_msg.aimer_timestamp);

        setOutput("outpost_blood", game_msg.outpost_blood);
        setOutput("sentry_blood", game_msg.sentry_blood);
        setOutput("bullet_num", game_msg.bullet_num);
        setOutput("time_left", game_msg.time_left);
        setOutput("game_timestamp", game_msg.game_timestamp);
        setOutput("manual_top", game_msg.manual_top);
        setOutput("game_start", game_msg.game_start);
        setOutput("m_alive", game_msg.m_alive);
        setOutput("enemy_alive", game_msg.enemy_alive);
        setOutput("hityaw", game_msg.hityaw);

        setOutput("navigation_cur_position_x", navigation_msg.navigation_cur_position_x);
        setOutput("navigation_cur_position_y", navigation_msg.navigation_cur_position_y);
        setOutput("navigation_cur_position_z", navigation_msg.navigation_cur_position_z);
        setOutput("navigation_cur_orientation_x", navigation_msg.navigation_cur_orientation_x);
        setOutput("navigation_cur_orientation_y", navigation_msg.navigation_cur_orientation_y);
        setOutput("navigation_cur_orientation_z", navigation_msg.navigation_cur_orientation_z);
        setOutput("navigation_cur_orientation_w", navigation_msg.navigation_cur_orientation_w);
        setOutput("navigation_status", navigation_msg.navigation_status);
        setOutput("navigation_timestamp", navigation_msg.navigation_timestamp);

        setOutput("scan_mode", scan_mode);

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
        std::cout << "sent_top:" << msg.start << std::endl;
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
        std::cout << "sent_top:" << msg.start << std::endl;
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
        port_list.insert(BT::InputPort<int>("aimer_if_track"));
        port_list.insert(BT::InputPort<bool>("aimer_shootable"));
        return port_list;
    }
    
    /**
     * @brief 跟随不发射节点
     * 
    */
    BT::NodeStatus Track_Node::tick()
    {
        msg.bullet_rate = 0; // 保持跟随不发射
        msg.btaimer_timestamp = wmj::now();
        std::cout << "sent_bullet_rate:" << msg.bullet_rate << std::endl;
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
        auto aimer_if_track = getInput<int>("aimer_if_track");
        auto aimer_shootable = getInput<bool>("aimer_shootable");
        // msg.bullet_rate = bullet_rate.value();

        std::cout << "aimer_if_track:" << aimer_if_track.value() << std::endl;
        std::cout << "aimer_shootable:" << aimer_shootable.value() << std::endl;
        if( aimer_if_track.value() )
        {
            if( aimer_shootable.value() )
            {
                
                msg.bullet_rate = 1;
            }
            else
            {
                msg.bullet_rate = 0;
            }
        }
        else
        {
            msg.bullet_rate = -1;
        }

        msg.btaimer_timestamp = wmj::now();
        std::cout << "sent_bullet_rate:" << msg.bullet_rate << std::endl;
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
        std::cout << "sent_bullet_rate:" << msg.bullet_rate << std::endl;
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
        startPosition = GetPositionInfo(0);
        resumePosition = GetPositionInfo(1);
        gainPosition = GetPositionInfo(2);
        viewPosition = GetPositionInfo(3);
    }
    
    BT::PortsList Navigation_Node::providedPorts()
    {
        BT::PortsList port_lists;

        port_lists.insert(BT::InputPort<int>("position"));
        port_lists.insert(BT::OutputPort<int>("navigation_timeout"));
        
        return port_lists;
    }

     /**
     * @brief 获取位置信息
     * 
     * @param position 位置
     * 
     * @return PoseStamped 目标位置，四元数类型
    */
    geometry_msgs::msg::PoseStamped Navigation_Node::GetPositionInfo(int position)
    {
        cv::FileStorage fs(BT_YAML, cv::FileStorage::READ);
        geometry_msgs::msg::PoseStamped goal_position;
        switch (position)
        {
            case 0:          // 出发点
                fs["start_position_x"] >> goal_position.pose.position.x;
                fs["start_position_y"] >> goal_position.pose.position.y;
                fs["start_position_z"] >> goal_position.pose.position.z;
                fs["start_orientation_x"] >> goal_position.pose.orientation.x;
                fs["start_orientation_y"] >> goal_position.pose.orientation.y;
                fs["start_orientation_z"] >> goal_position.pose.orientation.z;
                fs["start_orientation_w"] >> goal_position.pose.orientation.w;
                break;
            case 1:        // 回血点
                fs["resume_position_x"] >> goal_position.pose.position.x;
                fs["resume_position_y"] >> goal_position.pose.position.y;
                fs["resume_position_z"] >> goal_position.pose.position.z;
                fs["resume_orientation_x"] >> goal_position.pose.orientation.x;
                fs["resume_orientation_y"] >> goal_position.pose.orientation.y;
                fs["resume_orientation_z"] >> goal_position.pose.orientation.z;
                fs["resume_orientation_w"] >> goal_position.pose.orientation.w;
                break;
            case 2:        // 增益点
                fs["gain_position_x"] >> goal_position.pose.position.x;
                fs["gain_position_y"] >> goal_position.pose.position.y;
                fs["gain_position_z"] >> goal_position.pose.position.z;
                fs["gain_orientation_x"] >> goal_position.pose.orientation.x;
                fs["gain_orientation_y"] >> goal_position.pose.orientation.y;
                fs["gain_orientation_z"] >> goal_position.pose.orientation.z;
                fs["gain_orientation_w"] >> goal_position.pose.orientation.w;
                break;
            case 3:        // 增益点
                fs["view_position_x"] >> goal_position.pose.position.x;
                fs["view_position_y"] >> goal_position.pose.position.y;
                fs["view_position_z"] >> goal_position.pose.position.z;
                fs["view_orientation_x"] >> goal_position.pose.orientation.x;
                fs["view_orientation_y"] >> goal_position.pose.orientation.y;
                fs["view_orientation_z"] >> goal_position.pose.orientation.z;
                fs["view_orientation_w"] >> goal_position.pose.orientation.w;
                break;
        }
        goal_position.header.frame_id = "map";
        goal_position.header.stamp.nanosec = wmj::now();
        return goal_position;
    }

    /**
     * @brief 开启导航节点
     * 
     * @param position 选择目标位置
    */
    BT::NodeStatus Navigation_on_Node::tick()
    {
        auto position = getInput<int>("position");
        if (!position)
        {
            throw BT::RuntimeError("navigation_node missing required input [message]:",
                                   position.error());
        }

        std::cout << "navigation_position:" << position.value() << std::endl;
        switch (position.value())
        {
        case 0:
            msg.goal_position = startPosition;
            break;
        case 1:
            msg.goal_position = resumePosition;
            break;
        case 2:
            msg.goal_position = gainPosition;
            break;
        case 3:
            msg.goal_position = viewPosition;
        }

        std::cout << "navigation_last_position:" << final_last_position << std::endl;
        if(position.value() == final_last_position)
        {
            msg.navigation_continue = 2;                          // 与上一帧位置相同，则continue置为 2
        }
        else
        {
            msg.navigation_continue = 1;                          // 与上一帧位置不同，则continue置为 1                        
        }

        if( msg.navigation_continue == 2 )                        // 导航状态内10帧还未抵达目的地，将再次命令导航开启规划
        {
            time_count++;
            if( time_count > 15)
            {
                msg.navigation_continue = 1;
                time_count = 0;
                out_time ++;
            }
        }
        else
        {
            out_time = 0;
            time_count = 0;
        }

        int navigation_timeout = 0;
        if( out_time == 4 )
        {
            navigation_timeout = 1;
            setOutput("navigation_timeout", navigation_timeout);
        }
        
        msg.bt_navigation_timestamp = wmj::now();
        std::cout << "navigation_on_time_count:" << time_count << std::endl;
        std::cout << "navigation_on_out_time_count:" << out_time << std::endl;
        std::cout << "navigation_on_timeout:" << navigation_timeout << std::endl;
        final_last_position = position.value();

        std::cout << "sent_navigation_continue:" << msg.navigation_continue << std::endl;
        navigationPub->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }
    
    /**
     * @brief 导航关闭节点
    */
    BT::NodeStatus Navigation_off_Node::tick()
    {
        msg.navigation_continue = 0;
        msg.bt_navigation_timestamp = wmj::now();
        msg.goal_position.pose.position.x = 0;
        msg.goal_position.pose.position.y = 0;
        msg.goal_position.pose.position.z = 0;
        msg.goal_position.pose.orientation.x = 0;
        msg.goal_position.pose.orientation.y = 0;
        msg.goal_position.pose.orientation.z = 0;
        msg.goal_position.pose.orientation.w = 0;
        msg.goal_position.header.frame_id = "map";
        msg.goal_position.header.stamp.sec = wmj::now();

        std::cout << "sent_navigation_continue:" << msg.navigation_continue << std::endl;
        navigationPub->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief 返航节点
    */
    // BT::NodeStatus Navigation_back_Node::tick()
    // {
    //     msg.navigation_continue = true;
    //     msg.bt_navigation_timestamp = wmj::now();
        
    //     navigationPub->publish(msg);
    //     return BT::NodeStatus::SUCCESS;
    // }

//------------------------------------------------------ Scan_Node ---------------------------------------------------------------    
    
    /**
     * @brief 扫描基类
    */
    Scan_Node::Scan_Node(const std::string &name, const BT::NodeConfig &config,
                         rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        ScanPub = node->create_publisher<base_interfaces::msg::ScanCtrlInfo>("ScanCtrlInfo", 10);
    }

    BT::PortsList Scan_Node::providedPorts()
    {
        BT::PortsList port_lists;
        port_lists.insert(BT::InputPort<int>("scan_mode"));
        return port_lists;
    }
    
    /**
     * @brief 开启扫描节点
    */
    BT::NodeStatus Scan_on_Node::tick()
    {
        // msg.scan_timestamp = wmj::now();
        auto scan_mode = getInput<int>("scan_mode");
        if (!scan_mode)
        {
            throw BT::RuntimeError("detect_node missing required input [message]:",
                                   scan_mode.error());
        }
        msg.scan_mode = scan_mode.value();  
        std::cout << "sent_scan:" << msg.scan_mode << std::endl;
        ScanPub->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief 开启90扫描节点
    */
    BT::NodeStatus Scan_90_Node::tick()
    {
        msg.scan_mode = wmj::SCAN_MODE::SCAN_90;  //SCAN_90
        // msg.scan_timestamp = wmj::now();
        std::cout << "sent_scan:" << msg.scan_mode << std::endl;
        ScanPub->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief 关闭扫描节点
    */
    BT::NodeStatus Scan_off_Node::tick()
    {
        msg.scan_mode = 4;  //SCAN_STOP
        // msg.scan_timestamp = wmj::now();
        std::cout << "sent_scan:" << msg.scan_mode << std::endl;
        ScanPub->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

// ------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------               Condition               --------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------------ Start_Condition      ---------------------------------------------------------------
    /**
     * @brief 比赛开始条件节点
    */
    Start_Condition::Start_Condition(const std::string &name, const BT::NodeConfig &config)
        : BT::ConditionNode(name, config)
    {
        RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Start_Condition Node is working");
    }

    BT::PortsList Start_Condition::providedPorts()
    {
        BT::PortsList port_lists;
        port_lists.insert(BT::InputPort<bool>("game_start"));
        return port_lists;
    }
    
    /**
     * @brief 获取比赛状态,true则证明比赛开始，开启主循环
     * 
     * @param game_start  比赛是否开始
     * 
     * @return 是否开启主循环
     */ 
    bool Start_Condition::GetGameStatus()
    {
        auto game_start = getInput<bool>("game_start");
        if(!game_start)
        {
            throw BT::RuntimeError("Game missing required input [message]:",
                                   game_start.error());
        }
        if(game_start.value())
        {
            return true;         // 比赛开始返回true
        }
        else
        {
            return false;        // 开启空转         
        }
    }
    
    /**
     * @brief 根据导航状态判断开启导航或是扫描
     */
    BT::NodeStatus Start_Condition::tick()
    {
        bool game_start = GetGameStatus();
        if(game_start)        
        {
            return BT::NodeStatus::SUCCESS;          
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("INFO"), "game hasn't started");    // 开启空转
            return BT::NodeStatus::FAILURE; 
        }
    }
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
        return false;
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
        port_lists.insert(BT::InputPort<int>("aimer_if_track"));
        port_lists.insert(BT::InputPort<int>("bullet_num"));
        return port_lists;
    }
    
    /**
     * @brief 获取扫描状态，根据装甲板识别效果判断。True则开启扫描,否则开启自瞄。
     * @param aimer_if_track
     * @param aimer_shootable
     * @return 是否开启扫描
    */ 
    bool Scan_Condition::GetScanStatus()
    {
        auto aimer_if_track = getInput<int>("aimer_if_track").value();
        auto bullet_num = getInput<int>("bullet_num").value();

        if( aimer_if_track == 1 && bullet_num > 0)
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
        port_lists.insert(BT::OutputPort<int>("position"));
        return port_lists;
    }
    
    /**
     * @brief 根据识别信息判断开启返航或者自瞄，True则返航，否则开启自瞄无限制击打
     * 
     * @param armor_distance   最近装甲板距离
     * @param armor_number     识别到的装甲板数量
     * 
     * @return 是否执行返航导航
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
        
        // 若在三米内识别到装甲板，则开启自瞄无限制击打，否则返回补血点
        if (armor_number.value() > 0 && armor_distance.value() < 300)
        {
            int position = 0;
            setOutput("position", position);
            return false;
        }
        else
        {
            int position = -1;
            setOutput("position", position);
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

        port_lists.insert(BT::OutputPort<int>("bullet_rate"));
        return port_lists;
    }
    
    /**
     * @brief 进攻姿态下决定是否开启自瞄，将综合各种情况计算出当前弹频
     * 
     * @param armor_distance     最近装甲板距离
     * @param armor_number       装甲板数量
     * @param bullet_num         剩余子弹数量
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


        if (!armor_number)
        {
            throw BT::RuntimeError("detect_node missing required input [message]:",
                                   armor_number.error());
        }
 
        // int bullet_rate = (int)(-10 + sqrt(time_left.value()) + 0.3 * sqrt(bullet_num.value()) -
        //                         (armor_distance.value() / 100) * (armor_distance.value() / 100) * (armor_distance.value() / 100)); 

        int bullet_rate = (int)(30 - 3*(armor_distance.value()/100));
        if(armor_distance.value() == 0 || armor_number.value() == 0)
        {
            bullet_rate = -1;
        }

    
        
        // 导航状态对判断条件进行削减并对确定后条件进行增强
        // if( navigation_status )
        // {
        //     if(bullet_rate*0.75 > 5)
        //     {
        //         bullet_rate = bullet_rate*1.25;
        //     }
        //     else
        //     {
        //         bullet_rate = -1;
        //     }
        // }
        
        if(bullet_rate > 20)
        {
            bullet_rate = 20;
        }
        
        std::cout << "armor_number:" << armor_number.value() << "  " << "armor_distance:" << armor_distance.value() << std::endl;
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
        cv::FileStorage fs(BT_YAML, cv::FileStorage::READ);
        fs["start_position_x"] >> goal_position[0][0];
        fs["start_position_y"] >> goal_position[0][1];
        fs["resume_position_x"] >> goal_position[1][0];
        fs["resume_position_y"] >> goal_position[1][1];
        fs["gain_position_x"] >> goal_position[2][0];
        fs["gain_position_y"] >> goal_position[2][1];
        fs["view_position_x"] >> goal_position[3][0];
        fs["view_position_y"] >> goal_position[3][1];
    }

    BT::PortsList Defend_Main_Condition::providedPorts()
    {
        BT::PortsList port_lists;

        port_lists.insert(BT::InputPort<int>("armor_number"));
        port_lists.insert(BT::InputPort<double>("armor_distance"));
        
        port_lists.insert(BT::InputPort<int>("aimer_if_track"));
        port_lists.insert(BT::InputPort<bool>("aimer_shootable"));

        port_lists.insert(BT::InputPort<int>("sentry_blood"));
        port_lists.insert(BT::InputPort<int>("bullet_num"));
        port_lists.insert(BT::InputPort<int>("time_left"));
        port_lists.insert(BT::InputPort<int>("outpost_blood"));
        port_lists.insert(BT::InputPort<int>("m_alive"));
        port_lists.insert(BT::InputPort<int>("enemy_alive"));
        port_lists.insert(BT::InputPort<double>("hityaw"));

        port_lists.insert(BT::InputPort<double>("navigation_cur_position_x"));
        port_lists.insert(BT::InputPort<double>("navigation_cur_position_y"));
        port_lists.insert(BT::InputPort<double>("navigation_cur_position_z"));
        port_lists.insert(BT::InputPort<double>("navigation_cur_orientation_x"));
        port_lists.insert(BT::InputPort<double>("navigation_cur_orientation_y"));
        port_lists.insert(BT::InputPort<double>("navigation_cur_orientation_z"));
        port_lists.insert(BT::InputPort<double>("navigation_cur_orientation_w"));
        port_lists.insert(BT::InputPort<int>("navigation_status"));

        port_lists.insert(BT::OutputPort<int>("bullet_rate"));
        port_lists.insert(BT::OutputPort<int>("scan_mode"));
        port_lists.insert(BT::OutputPort<int>("position"));
        port_lists.insert(BT::InputPort<int>("default_position"));
        port_lists.insert(BT::InputPort<int>("navigation_timeout"));

        return port_lists;
    }

    /**
     * @brief 防御姿态下决定是否开启导航，将综合各种情况计算出当前弹频
     * 
     * @param armor_distance     最近装甲板距离
     * @param armor_number       装甲板数量
     * @param bullet_num         剩余子弹数量
     * @param outpost_blood      前哨站血量
     * @param sentry_blood       哨兵血量 
     * @param navigation_xxxx    当前位置（四元数）
     * 
     * @return position 导航目标位置 -1 为不开启导航
    */
    int Defend_Main_Condition::DefendCondition()
    {
        auto armor_number = getInput<int>("armor_number");
        auto armor_distance = getInput<double>("armor_distance");
        auto sentry_blood = getInput<int>("sentry_blood");
        auto bullet_num = getInput<int>("bullet_num");
        auto time_left = getInput<int>("time_left");
        auto outpost_blood = getInput<int>("outpost_blood");
        auto m_alive = getInput<int>("m_alive");
        auto enemy_alive = getInput<int>("enemy_alive");
        auto default_position = getInput<int>("default_position");
        auto hityaw = getInput<double>("hityaw");
        auto aimer_if_track = getInput<int>("aimer_if_track");
        auto aimer_shootable = getInput<bool>("aimer_shootable");
        auto navigation_status = getInput<int>("navigation_status");
        auto navigation_timeout = getInput<int>("navigation_timeout");
        
        if (!armor_number)
        {
            throw BT::RuntimeError("detect_node missing required input [message]:",
                                   armor_number.error());
        }
 
        // 血量增幅计算
        int blood_diff = sentry_blood.value() - last_blood;  
        // 假设恢复血量时没有被击打，则累加血量增加量
        if(blood_diff > 0)
        {
            total_blood += blood_diff;
        }


        int m_position = 3;  // 默认导航前往视界位置

        if((time_left.value() < 160 && time_left.value() > 135))
        {   
            m_position = 2;         // 假设每次增益点都在增益点开启30s内解决,则规定时间内前往增益点抢夺，最后一波不会前往
        }        

        if( m_alive.value() < enemy_alive.value() )
        {
            m_position = 0;            // 我方陷入人数劣势时，若无需去往补血点，则返回出发点
        }

        if(sentry_blood.value() < 350)
        {
            if(time_left.value() > 60 && total_blood < 600)
            {
                m_position = 1;        // 血量低于350且在补血时间内，且补血点还有资源可补血，则返回补血点
            }
            else
            {
                m_position = 0;        // 若不在补血时间内，或者可用资源不足，则返回出发点
            }
        }

        if( time_left.value() < 120 && time_left.value() > 60 && sentry_blood.value() < 600 && total_blood < 600)
        {
            m_position = 1;            // 补血前一分钟，如果补血点还有血并且自身状态未满，则前往补血点
        }

        if( m_goal_position == 1 && total_blood < 600 && time_left.value() > 60 && sentry_blood.value() < 600)
        {
            m_position = 1;            // 在补血点补满血量后才会离开
        }

        // 第一次补给点一定会去
        if(time_left.value() < 250 && time_left.value() > 225 && sentry_blood.value() > 200)
        {
            m_position = 2;
        }

        if( aimer_if_track.value() == -1 )           // 自瞄信息收不到后会将默认位置改为出发点
        {
            if( m_position == 3 )
            {
                m_position = 0;
            }
        }

        if( time_left.value() < 60 )                      // 比赛最后一分钟返回出发点
        {
            m_position = 0; 
        }

        if( default_position.value() != -1)           // 比赛信息收不到，默认返回出发点
        {
            m_position = default_position.value();
        }

        std::cout << "init_position:" << m_position << std::endl;

        if( navigation_status.value() == 1 )                                    // 如果导航节点认为自己已经到了且目标点一直未改变，则停止导航
        {
            if_arrive = 2;
        }

        // 如果导航有目标位姿且导航节点认为自身未到达目标点，判断目标位置和当前位置的差距
        if( m_position != -1 )           
        {
            double goal_pose[2];
            double m_pose[2];
            goal_pose[0] = goal_position[m_position][0];
            goal_pose[1] = goal_position[m_position][1];
            m_pose[0] = getInput<double>("navigation_cur_position_x").value();
            m_pose[1] = getInput<double>("navigation_cur_position_y").value();
            std::cout << "cur_pose_x:" << m_pose[0] << "  " << "cur_pose_y:" << m_pose[1] << std::endl;
            std::cout << "goal_pose_y:" << goal_pose[0] << "  " << "goal_pose_y:" << goal_pose[1] << std::endl;
            std::cout << "diff_pose_x:" << fabs(m_pose[0] - goal_pose[0]) << "  " << "diff_pose_y:" << fabs(m_pose[1] - goal_pose[1]) << std::endl;
            if( if_arrive == 0 )       // 如果当前未到达位置
            {         
                if( fabs(m_pose[0] - goal_pose[0]) < 0.3 && fabs(m_pose[1] - goal_pose[1]) < 0.3)               // 未到达目标位置的情况下，25cm内认为已到目标位置，first_arrive置1
                {
                    if_arrive = 1;
                    m_position = -1;
                }
            }
            else                      // 如果已到达指定位置
            {
                if( fabs(m_pose[0] - goal_pose[0]) < 0.68 && fabs(m_pose[1] - goal_pose[1]) < 0.68 )                // 未到达目标位置的情况下，50cm内认为已到目标位置，first_arrive置1
                {
                    m_position = -1;
                }
                else
                {
                    if_arrive = 0;                                                                               // 若误差过大，则认为当前已经偏离目标位置，需要重新导航，if_arrive置0
                }
            }
        }

        if( navigation_status.value() == 2 || navigation_timeout.value() == 1 )                               // 导航崩掉了则不会再进行导航
        {
            m_position = -1;             
            m_last_position = -1;
            m_goal_position = -1;
        }

        // 防止在补血点受到伤害后，误以为自身还有血量可补充，做的时间最大值退出
        if( if_arrive != 0 && m_goal_position == 1 )      // 如果我已经到达补血点
        {
            std::cout << "blood_time:" << blood_time << std::endl;
            blood_time ++;
            if( blood_time > 30 )  // 如果30帧都没有离开补血区域，则认为已经没有可以补充的血量。 15s TimeOut
            {
                total_blood = 600;
            }
        }
        else
        {
            blood_time = 0;
        }

             
        int m_last_goal_position = m_goal_position;
        if( m_position != -1)                                                // 记录本次目标点
        {
            m_goal_position = m_position;
        }

        if( m_last_goal_position != -1 && m_goal_position != m_last_goal_position )         // 记录上一此目标点   
        {
            m_last_position = m_last_goal_position;
        } 
        

        // 被打了不会导航,除非是回补血点或者在补血点出发，但如果从补血点出发到了目标位置后，会继续反击
        if( blood_diff <= -5  )
        {
            detect_time = 15;
        }
        if( m_goal_position == 1 || (m_last_position == 1 && ( m_goal_position != 1 && if_arrive != 0 )))
        {
            detect_time = 0;
        }
       if(detect_time > 0)
        {
            detect_time--;
            m_position = -1;
        } 
        std::cout << "defend_detect_time:" << detect_time << std::endl;

        int scan_mode = 0;
        int bullet_rate = -1;

        //  丢失别后扫描逻辑
        if( aimer_if_track.value() == 1 )
        {
            m_position = -1;
            bullet_rate = 20;
            m_last_track = 1;
            m_lost_detect_count = 0;           // 每次识别到后重置扫描时间
        }
        else if(aimer_if_track.value() == 0 && m_last_track == 1 )     
        {
            if( m_goal_position != 1  && ( m_last_position != 1 || ( m_last_position == 1 && if_arrive != 0 )))  // 丢失别后开启扫描，如果去往1或者从1出发则不会激发此效果，若从1出发但到达目标点后则可以激发  
            {
                if( m_lost_detect_count < 15 )   // 90度扫7.5s
                {
                    m_position = -1;
                    bullet_rate = 20;
                    m_lost_detect_count++;
                    scan_mode = 2;
                }
                else
                {
                    m_last_track = 0;
                    m_lost_detect_count = 0;
                }
            }
            else
            {
                m_last_track = 0;
                m_lost_detect_count = 0;
            }
        }
        std::cout << "defend_m_lost_detect_count:" << m_lost_detect_count << std::endl;
        
        last_blood = sentry_blood.value();
        
        std::cout << "defend_total_blood:" << total_blood << std::endl;
        std::cout << "blood_diff:" << blood_diff << std::endl;
        std::cout << "defend_armor_number:" << armor_number.value() << std::endl;
        std::cout << "defend_armor_distance:" << armor_distance.value() << std::endl;
        std::cout << "defend_bullet_rate:" << bullet_rate << std::endl;
        std::cout << "defend_goal_position:" << m_goal_position << std::endl;
        std::cout << "defend_last_position:" << m_last_position << std::endl;
        std::cout << "defend_position:" << m_position << std::endl;
        std::cout << "defend_if_arrive:" << if_arrive << std::endl;
        std::cout << "defend_scan_mode:" << scan_mode << std::endl;
        // std::cout << "defend_hityaw:" << hityaw.value() << std::endl;
        std::cout << "defend_if_track:" << aimer_if_track.value() << std::endl;
        std::cout << "defend_shootable:" << aimer_shootable.value() << std::endl;
        std::cout << "defend_navigation_status:" << navigation_status.value() << std::endl;
        std::cout << "defend_navigation_timeout:" << navigation_timeout.value() << std::endl;

        setOutput("bullet_rate", bullet_rate);
        setOutput("position", m_position);
        setOutput("scan_mode", scan_mode);
        return m_position;
    }

    /**
     * @brief Defend_Main_Condition判断函数，根据position的值进行判断
     * 
     * @param position    导航目标位置
    */
    BT::NodeStatus  Defend_Main_Condition::tick()
    {
        int position = DefendCondition();
        if (position != -1)
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
    auto node = std::make_shared<rclcpp::Node>("behavior_tree");

    factory.registerNodeType<wmj::DataReadNode>("DataReadNode", node);
    factory.registerNodeType<wmj::No_Shoot_Node>("No_Shoot_Node", node);
    factory.registerNodeType<wmj::Limit_Shoot_Node>("Limit_Shoot_Node", node);
    factory.registerNodeType<wmj::Track_Node>("Track_Node", node);
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
    factory.registerNodeType<wmj::Start_Condition>("Start_Condition");
    
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
