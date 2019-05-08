#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <linux/input.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>

#define KEY_W 17
#define KEY_A 30
#define KEY_S 31
#define KEY_D 32
#define KEY_UP 103
#define KEY_LEFT 105
#define KEY_DOWN 108
#define KEY_RIGHT 106

#define DEBUG 0

using namespace std;
class Keyboard
{
private:
    double walk_vel_;
    double run_vel_;
    double max_tv;
    double max_rv;
    std::string input;

    int kfd;
    struct termios cooked, raw;
    struct input_event t;

    geometry_msgs::Twist cmdvel_;
    ros::NodeHandle n_;
    ros::Publisher pub_;

public:
    Keyboard()
    {
        pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        ros::NodeHandle n_private("~");
        n_private.param("walk_vel", walk_vel_, 0.5);
        n_private.param("run_vel", run_vel_, 0.5);
        n_private.param("max_tv", max_tv, 2.0);
        n_private.param("max_rv", max_rv, 2.6);
        n_private.param("input", input, std::string("/dev/input/event3"));

        Hidden_Character();
    }
    ~Keyboard()
    {
        tcsetattr(kfd, TCSANOW, &cooked); //恢复参数
    }

    void keyboardLoop();
    void stopRobot()
    {
        cmdvel_.linear.x = 0.0;
        cmdvel_.angular.z = 0.0;
        pub_.publish(cmdvel_);
    }
    void Hidden_Character()
    {
        //隐藏字符
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
    }
};
string msg = "\n\
Reading from the keyboard  and Publishing to Twist!\n\
---------------------------\n\
Moving around:\n\
                            \n\
        w       \n\
    a   s   d  \n\
                            \n\
UP/DOWN : increase/decrease only linear speed by 0.2\n\
LEFT/RIGHT : increase/decrease only angular speed by 0.2\n\
CTRL-C to quit";

void Keyboard::keyboardLoop()
{
    char c;
    int speed = 0, turn = 0;
    int keys_fd;
    char ret[2];

    ROS_INFO_STREAM(msg);

    keys_fd = open(input.c_str(), O_RDONLY);
    if (keys_fd <= 0)
    {
        ROS_INFO_STREAM("open " << input << " device error");
    }

    for (;;)
    {
        if (read(keys_fd, &t, sizeof(t)) == sizeof(t))
        {
            if (t.type == EV_KEY)
            {
                if (t.value == 0 || t.value == 1)
                {
#if DEBUG
                    printf("key %d %d\n", t.code, t.value);
#endif

                    if (t.code == KEY_W)
                    {
                        if (t.value == 1)
                            speed = 1;
                        else
                            speed = 0;
                    }
                    else if (t.code == KEY_S)
                    {
                        if (t.value == 1)
                            speed = -1;
                        else
                            speed = 0;
                    }

                    if (t.code == KEY_A)
                    {
                        if (t.value == 1)
                            turn = -1;
                        else
                            turn = 0;
                    }
                    else if (t.code == KEY_D)
                    {
                        if (t.value == 1)
                            turn = 1;
                        else
                            turn = 0;
                    }

                    if (t.code == KEY_UP)
                    {
                        if (t.value == 1)
                        {
                            max_tv += 0.2;
                            if (max_tv > 2)
                                max_tv = 2;
                        }
                    }
                    else if (t.code == KEY_DOWN)
                    {
                        if (t.value == 1)
                        {
                            max_tv -= 0.2;
                            if (max_tv < 0.2)
                                max_tv = 0.2;
                        }
                    }

                    if (t.code == KEY_LEFT)
                    {
                        if (t.value == 1)
                        {
                            max_rv -= 0.2;
                            if (max_rv < 0.2)
                                max_rv = 0.2;
                        }
                    }
                    else if (t.code == KEY_RIGHT)
                    {
                        if (t.value == 1)
                        {
                            max_rv += 0.2;
                            if (max_rv > 3)
                                max_rv = 3;
                        }
                    }
                    cmdvel_.linear.x = speed * max_tv;
                    cmdvel_.angular.z = turn * max_rv;
                    pub_.publish(cmdvel_);
                }
            }
        }
    }
    close(keys_fd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keybord");
    Keyboard tbk;
    boost::thread t = boost::thread(boost::bind(&Keyboard::keyboardLoop, &tbk));
    ros::spin();
    return (0);
}
