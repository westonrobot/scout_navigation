#include "scout_webots_sim/scout_webots_runner.hpp"
#include <signal.h>
namespace westonrobot
{
    ScoutWebotsRunner::ScoutWebotsRunner()
    {
        std::cout<<"Creating runner"<<std::endl;
    }

    void ScoutWebotsRunner::AddExtension(std::shared_ptr<WebotsExtension> extensionPointer)

    {
        extensions.push_back(extensionPointer);
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

        signal(SIGINT, Quit);

        // subscribe to the topic model_name to get the list of availables controllers
        std::string controllerName;
        ros::Subscriber nameSub =
            nh.subscribe("model_name", 100, ControllerNameCallback);
        while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers())
        {
            ros::spinOnce();
            ros::spinOnce();
            ros::spinOnce();
        }
        ros::spinOnce();

        // if there is more than one controller available, it let the user choose
        if (controllerCount == 1)
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
        if (!extensions.empty())
        {
            scout_webots.AddExtensions(extensions);
            scout_webots.InitExtensions();
        }

        ROS_INFO("Entering ROS main loop...");

        // main loop
        timeStepClient = nh.serviceClient<webots_ros::set_int>("/" + controllerName +
                                                               "/robot/time_step");
        timeStepSrv.request.value = time_step;
        ros::Rate loop_rate(messenger.sim_control_rate_);
        ros::AsyncSpinner spinner(2);
        spinner.start();
        while (ros::ok())
        {
            if (timeStepClient.call(timeStepSrv) && timeStepSrv.response.success)
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
        timeStepSrv.request.value = 0;
        timeStepClient.call(timeStepSrv);

        spinner.stop();
        ros::shutdown();
        return 0;
    }

    void ScoutWebotsRunner::Quit(int sig)
    {

        ROS_INFO("User stopped the 'scout_webots_node'.");
        timeStepSrv.request.value = 0;
        timeStepClient.call(timeStepSrv);
        ros::shutdown();
        exit(0);
    }

    void ScoutWebotsRunner::ControllerNameCallback(const std_msgs::String::ConstPtr &name)
    {
        controllerCount++;
        controllerList.push_back(name->data);
        ROS_INFO("Controller #%d: %s.", controllerCount,
                 controllerList.back().c_str());
    }
} // namespace westonrobot