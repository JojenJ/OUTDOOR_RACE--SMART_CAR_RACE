
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <thread>

using namespace std;
int flag=0;//控制停车，0不停车，1停车
int tlag = 0;//按键停车
int plag1 = 0;//这三个标志位是控制每个停车点只停一次
int plag2 = 0;
int plag3 = 0;
int kount = 0;
class PubAndSub1
{
private:
    ros::NodeHandle n_;
    ros::Publisher pub_control=n_.advertise<geometry_msgs::Twist>("/car/cmd_vel",10);
    // ros::Publisher pub_stoppoint=n_.advertise<std_msgs::Int8>("/stop_getpoints",10);
    ros::Subscriber sub_teleop_vel;
    ros::Subscriber sub_cmd_vel;
    ros::Subscriber sub_dis;
    double distance;
public:
    PubAndSub1()
    {
        sub_cmd_vel= n_.subscribe("/car/vel",5,&PubAndSub1::amclcallback,this);//订阅导航速度，导航速度发布到"/car/vel"话题中
        // sub_light= n_.subscribe("/light",5,&PubAndSub1::lightback,this);
        //红绿灯置flag为1，不订阅寻线，在此节点publish停车twist 5s，置flag=2
        //if flag==2
        sub_teleop_vel = n_.subscribe("teleop",5,&PubAndSub1::teleopback,this);//按键
        //if(flag==3)//发布永停
        sub_dis = n_.subscribe("/distance_traveled", 5, &PubAndSub1::distanceCallback, this);
    }
    void amclcallback(const geometry_msgs::Twist& twist);
    // void lightback(const std_msgs::Int8 &aaa);
    void teleopback(const std_msgs::Int8 &tele);
    void distanceCallback(const std_msgs::Float64::ConstPtr& distance_msg);
};

void PubAndSub1::amclcallback(const geometry_msgs::Twist& guide_twist)
{ 
    if(flag==0)
    {
    pub_control.publish(guide_twist);//发布导航速度，车跑
    }
    if(flag==1)
    {//满足停车点范围（一共3个）
    geometry_msgs::Twist twist1;
    twist1.linear.x=1500;
    twist1.linear.y=0;
    twist1.linear.z=0;
    twist1.angular.x=0;
    twist1.angular.y=0;
    twist1.angular.z=90;
    pub_control.publish(twist1);
    //stop car 3s
     flag=0;
    sleep(3);
    }
}

void PubAndSub1::teleopback(const std_msgs::Int8 &tele)
{
    tlag = tele.data;
    if (tlag == 1) {
        geometry_msgs::Twist twist2;
        twist2.linear.x = 1500;
        twist2.linear.y = 0;
        twist2.linear.z = 0;
        twist2.angular.x = 0;
        twist2.angular.y = 0;
        twist2.angular.z = 90;
        pub_control.publish(twist2);
        //stop car 3s
        tlag = 0;
        sleep(10);

    }
    if (tlag == 3) {


    }
    // ROS_INFO(flag);
}

void PubAndSub1::distanceCallback(const std_msgs::Float64::ConstPtr& distance_msg)
{//距离回调函数
    distance = distance_msg->data;
    double min_distance1 = 2.30; // 设置第一个停车点最小距离
    double max_distance1 = 2.60; // 设置第一个停车点最大距离
    double min_distance2 = 13.65; // 设置第二个停车点最小距离
    double max_distance2 =  13.90; // 设置第二个停车点最大距离
    double min_distance3 = 4.25; // 设置第三个停车点最小距离
    double max_distance3 = 4.85; // 设置第三个停车点最大距离
    if ((distance >= min_distance1) && (distance <= max_distance1)&& (plag1==0)) {//一旦满足第一个停车点的条件
        flag = 1;//控制停车，0不停车，1停车
        plag1 = 1;//再也不进第一个停车点了
    }
    if ((distance >= min_distance2) && (distance <= max_distance2)&& (plag2==0)) {//一旦满足第二个停车点的条件
        flag = 1;//控制停车，0不停车，1停车
        plag2 = 1;//再也不进第二个停车点了
        kount = 1;
    }
    if ((distance >= min_distance3) && (distance <= max_distance3)&& (plag3==0)&&(kount==1)) {//一旦满足第三个停车点的条件
        flag = 1;//控制停车，0不停车，1停车
        plag3 = 1;//再也不进第三个停车点了
    }
}


int main(int argc, char** argv)
{ 
   
    ros::init(argc, argv, "midguanli");
    ros::NodeHandle guanli_n;
    PubAndSub1 myPAS;
    ros::spin();
    //ros::Subscriber guanli_sub1 = n.subscribe("/car/cmd_vel1",1,RedGreenguanliCallback);
    return 0;
}