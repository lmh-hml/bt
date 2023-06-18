#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "states.hpp"
#include "state_setter.hpp"

class MoveBaseAction : public BT::SyncActionNode
{
    public:
        MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<std::string>("goal") };
        }

        BT::NodeStatus tick() override
        {
            std::string goal;
            if (!getInput<std::string>("goal", goal)) {
                throw BT::RuntimeError("missing required input [goal]: ", goal);
            }
            printf("MoveBase: %s\n", goal.c_str());

            return BT::NodeStatus::SUCCESS;
        }
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_node");

    node->declare_parameter("xml", "/home/lmh/humble_ws/src/bt/trees/tree.xml");
    node->declare_parameter("plugin", "none");

    std::string plugin = node->get_parameter("plugin").as_string();
    std::string xml = node->get_parameter("xml").as_string();

    BT::RosNodeParams params;
    params.nh = node;

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<MoveBaseAction>("MoveBase");
    factory.registerNodeType<StateSetterNode>("StateSetter", params);
    factory.registerScriptingEnums<STATE>();

    // BT::RosNodeParams params;
    // params.nh = node;
    // RegisterRosNode(factory, plugin, params);

    auto tree = factory.createTreeFromFile(xml);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok()) {
        status = tree.tickOnce();
    }
    

    return 0;
}
