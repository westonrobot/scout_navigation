#include "scout_webots_sim/scout_webots_runner.hpp"
#include <signal.h>
namespace westonrobot
{
    ScoutWebotsRunner::ScoutWebotsRunner()
    {
        std::cout<<"Creating runner"<<std::endl;
    }

    void ScoutWebotsRunner::AddExtension(WebotsExtension* extension_pointer)

    {
        extensions_.push_back(extension_pointer);
    }

    int ScoutWebotsRunner::Run(int argc, char *argv[])
    {
        ros::init(argc, argv, "scout_webots_node", ros::init_options::AnonymousName);
        ros::NodeHandle nh, private_node("~");

        ScoutROSMessenger messenger(&nh);
        private_node.param<std::string>("odom_frame", messenger.odom_frame_,
                                        std::string("odom"));
        private_node.param<std::string>("base_frame", messenger.base_frame_,
                                        std::string("base_link"));
        private_node.param<std::string>("odom_topic_name", messenger.odom_topic_name_,
                                        std::string("odom"));
        private_node.param<int>("sim_control_rate", messenger.sim_control_rate_, 50);
        private_node.param<bool>("simulated_robot", messenger.simulated_robot_, true);
        messenger.SetupSubscription();

        const uint32_t time_step = 1000 / messenger.sim_control_rate_;
        ScoutWebotsInterface scout_webots(&nh, &messenger, time_step);
        ROS_INFO(
            "Chosen time step: '%d', make sure you set the same time step in Webots "
            "scene",
            time_step);

        // signal(SIGINT, Quit);

        // subscribe to the topic model_name to get the list of availables controllers
        std::string controllerName;
        ros::Subscriber nameSub =
            nh.subscribe("model_name", 100, &ScoutWebotsRunner::ControllerNameCallback, this);
        while (controller_count_ == 0 || controller_count_ < nameSub.getNumPublishers())
        {
            ros::spinOnce();
            ros::spinOnce();
            ros::spinOnce();
        }
        ros::spinOnce();

        // if there is more than one controller available, it let the user choose
        if (controller_count_ == 1)
            controllerName = controllerList[0];
        else
        {
            int wantedController = 0;
            std::cout << "Choose the # of the controller you want to use:\n";
            std::cin >> wantedController;
            if (1 <= wantedController && wantedController <= controllerCount)
                controllerName = controllerList[wantedController - 1];
            else
            {
                ROS_ERROR("Invalid number for controller choice.");
                return 1;
            }
        }
        ROS_INFO("Using controller: '%s'", controllerName.c_str());

        // leave topic once it is not necessary anymore
        nameSub.shutdown();

        // init robot components
        scout_webots.InitComponents(controllerName);
        if (!extensions_.empty())
        {
            scout_webots.AddExtensions(extensions_);
            scout_webots.InitExtensions();
        }

        ROS_INFO("Entering ROS main loop...");

        // main loop
        time_step_client_ = nh.serviceClient<webots_ros::set_int>("/" + controllerName +
                                                               "/robot/time_step");
        time_step_srv_.request.value = time_step;
        ros::Rate loop_rate(messenger.sim_control_rate_);
        ros::AsyncSpinner spinner(2);
        spinner.start();
        while (ros::ok())
        {
            if (time_step_client_.call(time_step_srv_) && time_step_srv_.response.success)
            {

                scout_webots.UpdateSimState();
            }
            else
            {
                static int32_t error_cnt = 0;
                ROS_ERROR("Failed to call service time_step for next step.");
                if (++error_cnt > 50)
                    break;
            }
            // ros::spinOnce();
            loop_rate.sleep();
        }
        time_step_srv_.request.value = 0;
        time_step_client_.call(time_step_srv_);

        spinner.stop();
        ros::shutdown();
        return 0;
    }

    void ScoutWebotsRunner::Quit(int sig)
    {

        ROS_INFO("User stopped the 'scout_webots_node'.");
        time_step_srv_.request.value = 0;
        time_step_client_.call(time_step_srv_);
        ros::shutdown();
        exit(0);
    }

    void ScoutWebotsRunner::ControllerNameCallback(const std_msgs::String::ConstPtr &name)
    {
        controller_count_++;
        controllerlList_.push_back(name->data);
        ROS_INFO("Controller #%d: %s.", controllerCount,
                 controller_list_.back().c_str());
    }
} // namespace westonrobot