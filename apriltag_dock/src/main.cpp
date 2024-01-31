#include <memory>
#include <thread>
#include <limits>

#include "rclcpp/rclcpp.hpp"

#include "apriltag_dock/auto_dock_client.hpp"

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoDockClient>();

    auto handle_user_input = [&]()
    {
        while (rclcpp::ok())
        {
            if (!node->isGoalExecuting())
            {
                RCLCPP_INFO(node->get_logger(),
                    "\nPress f to start the action"
                );

                char input = getchar();

                if (input == 'f')
                {
                    try
                    {
                        node->sendGoal();
                    }
                    catch (std::runtime_error& e)
                    {
                        RCLCPP_WARN(node->get_logger(), e.what());
                    }
                }
                else
                {
                    RCLCPP_WARN(node->get_logger(), "Invalid input");
                }
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
            else
            {
                RCLCPP_INFO(node->get_logger(),
                    "\nPress g to cancel the action"
                );

                char input = getchar();

                if (input == 'g')
                {
                    node->sendCancel();
                }
                else
                {
                    RCLCPP_WARN(node->get_logger(), "Invalid input");
                }
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }
    };
    std::thread user_input_thrd(handle_user_input);
    rclcpp::spin(node);
    user_input_thrd.join();
    rclcpp::shutdown();

    return 0;
}