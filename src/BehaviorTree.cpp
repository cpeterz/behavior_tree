#include "behavior_tree/BehaviorTree.hpp"

namespace wmj
{
    DataReadNode::DataReadNode(const std::string &name, const BT::NodeConfig &config,
                               rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "DataReadNode begin working");
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
        armor_msg.armor_timestamp = msg->time_seq[0];
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
        // 更新导航信息
        navigation_msg.navigation_timestamp = msg->navigation.navigation_timestamp;
        navigation_msg.QUAT_1 = msg->navigation.quat_1;
        navigation_msg.QUAT_i = msg->navigation.quat_i;
        navigation_msg.QUAT_j = msg->navigation.quat_j;
        navigation_msg.QUAT_k = msg->navigation.quat_k;
        RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Navigation msg get");
    }

    void DataReadNode::navigation_call_back(const base_interfaces::msg::Navigation::SharedPtr msg)
    {
        // 从导航发来的数据，显示当前导航状态
        navigation_msg.navigation_status = msg->navigation_on;
        RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Navigation status msg get");
    }

    // 定义接口
    BT::PortsList DataReadNode::providedPorts()
    {
        BT::PortsList ports_list;

        ports_list.insert(BT::OutputPort<int>("armor_number"));
        ports_list.insert(BT::OutputPort<double>("armor_distance"));
        ports_list.insert(BT::OutputPort<double>("armor_timestamp"));

        ports_list.insert(BT::OutputPort<double>("QUAT_1"));
        ports_list.insert(BT::OutputPort<double>("QUAT_i"));
        ports_list.insert(BT::OutputPort<double>("QUAT_j"));
        ports_list.insert(BT::OutputPort<double>("QUAT_k"));
        ports_list.insert(BT::OutputPort<double>("navigation_timestamp"));
        ports_list.insert(BT::OutputPort<bool>("navigation_status"));

        ports_list.insert(BT::OutputPort<double>("outpost_blood"));
        ports_list.insert(BT::OutputPort<double>("sentry_blood"));
        ports_list.insert(BT::OutputPort<int>("bullet_num"));
        ports_list.insert(BT::OutputPort<double>("time_left"));
        ports_list.insert(BT::OutputPort<double>("game_timestamp"));
        ports_list.insert(BT::OutputPort<bool>("manual_top"));

        return ports_list;
    }

    BT::NodeStatus DataReadNode::tick()
    {
        // 连续三次消息未更新，则使用默认数据
        if (last_armor_timestamp == armor_msg.armor_timestamp)
        {
            RCLCPP_INFO(rclcpp::get_logger("REEOR INFO"), "Armor msg missing %d", armor_msg_count);
            armor_msg_count++;
            if (armor_msg_count > 2)
            {
                armor_msg.armor_distance = 10;
                armor_msg.armor_number = 0;
            }
        }
        else
        {
            armor_msg_count = 0;
        }

        if (last_game_timestamp == game_msg.game_timestamp)
        {
            RCLCPP_INFO(rclcpp::get_logger("REEOR INFO"), "Game msg missing %d", game_msg_count);
            game_msg_count++;
            if (game_msg_count > 2)
            {
                game_msg.bullet_num = 700;
                game_msg.manual_top = true;
                game_msg.outpost_blood = 0;
                game_msg.sentry_blood = 100;
                game_msg.time_left = 300;
            }
        }
        else
        {
            game_msg_count = 0;
        }

        if (last_navigation_timestamp == navigation_msg.navigation_timestamp)
        {
            RCLCPP_INFO(rclcpp::get_logger("REEOR INFO"), "Navigation msg missing %d", navigation_msg_count);
            navigation_msg_count++;
            if (navigation_msg_count > 2)
            {
                navigation_msg.navigation_status = true;
                navigation_msg.QUAT_1 = 0; // 默认回到巡航位置
                navigation_msg.QUAT_i = 0;
                navigation_msg.QUAT_j = 0;
                navigation_msg.QUAT_k = 0;
            }
        }
        else
        {
            navigation_msg_count = 0;
        }

        last_armor_timestamp = armor_msg.armor_timestamp;
        last_game_timestamp = game_msg.game_timestamp;
        last_navigation_timestamp = navigation_msg.navigation_timestamp;

        setOutput("armor_number", armor_msg.armor_number);
        setOutput("armor_distance", armor_msg.armor_distance);
        setOutput("armor_timestamp", armor_msg.armor_timestamp);

        setOutput("QUAT_1", navigation_msg.QUAT_1);
        setOutput("QUAT_i", navigation_msg.QUAT_i);
        setOutput("QUAT_j", navigation_msg.QUAT_j);
        setOutput("QUAT_k", navigation_msg.QUAT_k);
        setOutput("navigation_timestamp", navigation_msg.navigation_timestamp);
        setOutput("navigation_status", navigation_msg.navigation_status);

        setOutput("outpost_blood", game_msg.outpost_blood);
        setOutput("sentry_blood", game_msg.sentry_blood);
        setOutput("bullet_num", game_msg.bullet_num);
        setOutput("time_left", game_msg.time_left);
        setOutput("game_timestamp", game_msg.game_timestamp);
        setOutput("manual_top", game_msg.manual_top);

        return BT::NodeStatus::SUCCESS;
    }

    Detect_Node::Detect_Node(const std::string &name, const BT::NodeConfig &config,
                             rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Detect_Node begin working");
        pub_shooter = node->create_publisher<base_interfaces::msg::Shooter>("BT_shooter", 10);
    }

    BT::PortsList Detect_Node::providedPorts()
    {
        BT::PortsList port_lists;

        port_lists.insert(BT::InputPort<int>("armor_number"));
        port_lists.insert(BT::InputPort<double>("armor_distance"));
        port_lists.insert(BT::InputPort<double>("armor_timestamp"));

        port_lists.insert(BT::InputPort<double>("sentry_blood"));
        port_lists.insert(BT::InputPort<int>("bullet_num"));
        port_lists.insert(BT::InputPort<double>("time_left"));
        port_lists.insert(BT::InputPort<double>("outpost_blood"));

        port_lists.insert(BT::OutputPort<int>("bullet_rate"));

        port_lists.insert(BT::InputPort<bool>("top_status"));
        port_lists.insert(BT::InputPort<bool>("navigation_status"));

        return port_lists;
    }

    BT::NodeStatus Detect_Node::tick()
    {
        // 从黑板取出数据
        auto armor_number = getInput<int>("armor_number");
        auto armor_distance = getInput<double>("armor_distance");
        auto sentry_blood = getInput<double>("sentry_blood");
        auto bullet_num = getInput<int>("bullet_num");
        auto time_left = getInput<int>("time_left");
        auto outpost_blood = getInput<double>("outpost_blood");
        auto top_status = getInput<bool>("top_status");
        auto navigation_status = getInput<bool>("navigation_status");

        base_interfaces::msg::Shooter msg;

        if (!armor_number)
        {
            throw BT::RuntimeError("detect_node missing required input [message]:",
                                   armor_number.error());
        }

        // 若小陀螺已开启，则无限制持续击打
        if (top_status.value() == true)
        {
            bullet_rate = 20; // 无限制击打
        }
        else
        {
            bullet_rate = (int)(20 + sqrt(time_left.value()) - 0.3 * sqrt(bullet_num.value()) -
                                (armor_distance.value() / 1000) * (armor_distance.value() / 1000) * (armor_distance.value() / 1000)) %
                          20;
        }

        // 导航开启情况
        if (navigation_status == true)
        {
            if (bullet_rate < 5) // 导航模式下如果弹速低于五则不设计
                bullet_rate = 0;
        }

        setOutput<int>("bullet_rate", bullet_rate);

        msg.bulletnum = bullet_rate;
        pub_shooter->publish(msg);

        return BT::NodeStatus::SUCCESS;
    }

    Top_Node::Top_Node(const std::string &name, const BT::NodeConfig &config,
                       rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Top_Node begin working");
        pub_top = node->create_publisher<base_interfaces::msg::Top>("BT_top", 10);
    }

    BT::PortsList Top_Node::providedPorts()
    {
        BT::PortsList port_lists;

        port_lists.insert(BT::InputPort<double>("outpost_blood"));
        port_lists.insert(BT::InputPort<bool>("manual_top"));
        // 将当前状态输出到黑板中
        port_lists.insert(BT::OutputPort<bool>("top_status"));

        return port_lists;
    }

    BT::NodeStatus Top_Node::tick()
    {
        auto outpost_blood = getInput<double>("outpost_blood");
        auto manual_top = getInput<bool>("manual_top");
        base_interfaces::msg::Top msg;

        if (!outpost_blood)
        {
            throw BT::RuntimeError("detect_node missing required input [message]:",
                                   outpost_blood.error());
        }

        // 若手动指令要求开启小陀螺，则启动小陀螺，无限制享受最高优先级
        if (manual_top.value() == true || outpost_blood.value() <= 500)
        {
            msg.start = true;
            top_status = true;
        }
        else
        {
            msg.start = false;
            top_status = false;
        }
        pub_top->publish(msg);
        RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "Top msg has been sent");
        setOutput("top_status", top_status);

        return BT::NodeStatus::SUCCESS;
    }

    Navigation_Node::Navigation_Node(const std::string &name, const BT::NodeConfig &config,
                                     rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
        RCLCPP_INFO(rclcpp::get_logger("STATUS INFO"), "Navigation_Node begin working");
        pub_navigation = node->create_publisher<base_interfaces::msg::Navigation>("BT_navigation", 10);
    }

    BT::PortsList Navigation_Node::providedPorts()
    {
        BT::PortsList port_lists;

        port_lists.insert(BT::InputPort<int>("bullet_rate"));
        port_lists.insert(BT::InputPort<double>("QUAT_1"));
        port_lists.insert(BT::InputPort<double>("QUAT_i"));
        port_lists.insert(BT::InputPort<double>("QUAT_j"));
        port_lists.insert(BT::InputPort<double>("QUAT_k"));
        port_lists.insert(BT::InputPort<bool>("top_status"));

        return port_lists;
    }

    BT::NodeStatus Navigation_Node::tick()
    {
        auto bullet_rate = getInput<int>("bullet_rate");
        auto QUAT_1 = getInput<double>("QUAT_1");
        auto QUAT_i = getInput<double>("QUAT_i");
        auto QUAT_j = getInput<double>("QUAT_j");
        auto QUAT_k = getInput<double>("QUAT_k");
        auto top_status = getInput<bool>("top_status");

        base_interfaces::msg::Navigation msg;
        msg.quat_1 = QUAT_1.value();
        msg.quat_i = QUAT_i.value();
        msg.quat_j = QUAT_j.value();
        msg.quat_k = QUAT_k.value();

        // 弹速达到一定程度，未开启小陀螺的情况下，会停止导航
        if (bullet_rate.value() > 5 && top_status == false)
        {
            msg.navigation_continue = false;
        }
        else
        {
            msg.navigation_continue = true;
        }

        pub_navigation->publish(msg);
        RCLCPP_INFO(rclcpp::get_logger("MSG INFO"), "navigation_msg has been sent!");
        return BT::NodeStatus::SUCCESS;
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    // 共用rclcpp::Node节点
    auto node = std::make_shared<rclcpp::Node>("topic");

    factory.registerNodeType<wmj::DataReadNode>("DataReadNode", node);
    factory.registerNodeType<wmj::Detect_Node>("Detect_Node", node);
    factory.registerNodeType<wmj::Top_Node>("Top_Node", node);
    factory.registerNodeType<wmj::Navigation_Node>("Navigation_Node", node);

    auto tree = factory.createTreeFromFile(BT_XML);

    rclcpp::Rate loop_rate(2);
    while (rclcpp::ok())
    {
        tree.tickWhileRunning();
        // std::cout << "STATUS:" << status << std::endl;
        loop_rate.sleep();
        rclcpp::spin_some(node);
    }
    return 0;
}
