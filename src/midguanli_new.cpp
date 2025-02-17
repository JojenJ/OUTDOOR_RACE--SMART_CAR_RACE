#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <ros/spinner.h>
#include <thread>
#include <signal.h>
#include <unistd.h>
#include <sys/types.h>
#include <iostream>

using namespace std;
// # 模式切换标志位
// # flag = 0 摄像头训线
// # flag = 1 雷达巡线
// # flag = 2 红绿灯停车3s
// # flag = 3 直道加速
// # flag = 4 雷达巡线
// # flag = 6 红绿灯停车3s
// # flag = 7 A点停车
// # flag = 8 结束
int flag=0,last_flag=0;
pid_t gmapping_pid = 0;

void start()
{
    system("rosrun laser_go laser_nav_A.py");
}

void start_gmapping()
{
    system("roslaunch racecar Run_gmapping.launch");
}

void save_map()
{
    system("rosrun map_server map_saver -f /racecar/src/racecar/map/map_10");
}

class PubAndSub
{
private:
    ros::NodeHandle nh_;
    ros::MultiThreadedSpinner spinner_;
    ros::Subscriber flag_sub;
public:
    PubAndSub(): spinner_(0) { }
  
    void initialize()
    {
        // 创建管家
        nh_ = ros::NodeHandle();
        // 定义多线程变量
        spinner_=ros::MultiThreadedSpinner(0);
        
        // 启动 gmapping 线程
        thread th1(start_gmapping);
        th1.detach();
        sleep(5);
        // 启动 laser_go 线程
        thread th0(start);
        th0.detach();

        flag_sub = nh_.subscribe("/flag", 10, &PubAndSub::FlagCallback, this);
    }

    void FlagCallback(const std_msgs::Int8& msg)
    {
        last_flag = flag;
        flag = msg.data;
        if (last_flag==3&&flag==4)
        {
            thread th2(save_map);
            th2.detach();
            sleep(1.2);
            system("rosnode kill /slam_gmapping");
        }
    }
};

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");   // 中文编码防止乱码
    ros::init(argc, argv, "midguanli_new");
    PubAndSub pub_and_sub;
    pub_and_sub.initialize();
    ros::spin();
    return 0;
}