// // //**********************两圈雷达******************//
// #include <ros/ros.h>
// #include <ros/package.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <std_msgs/Int8.h>
// #include <std_msgs/Float64.h>
// #include <ros/spinner.h>
// #include <thread>
// #include "sensor_msgs/LaserScan.h"
// #include "geometry_msgs/Twist.h"
// #include <math.h>
// #include "Control.h"


// #define freq 1440//每转一圈雷达扫描次数
// #define speed 1875
// #define skip_lase 2
// #define total_lidar_point 220
// float new_laser[total_lidar_point] = {0}; //拼接后的数据  1140-1440 + 0-300   前方150度的数据，逆时针
// float new_laser_not_sort[total_lidar_point] = {0}; //拼接后的数据  1140-1440 + 0-300   前方150度的数据，逆时针
// int ind[total_lidar_point] = {0};         //排序后元素对应的位置
// void BubbleSort(float *p, int length, int *ind_diff)
// {
//     for (int m = 0; m < length; m++)
//     {
//         ind_diff[m] = m;
//     }  

//     for (int i = 0; i < length; i++)
//     {
//         for (int j = 0; j < length - i - 1; j++)
//         {
//             if (p[j] > p[j + 1])
//             {
//                 float temp = p[j];
//                 p[j] = p[j + 1];
//                 p[j + 1] = temp;

//                 int ind_temp = ind_diff[j];
//                 ind_diff[j] = ind_diff[j + 1];
//                 ind_diff[j + 1] = ind_temp;    //排序后保存角度信息
//             }
//         }
//     }
// }   
// double get_distance_squre_pp(float target_range,int target_ind,float beigin_range,int begin_ind)
// {
// double P1P2=0;
// double detta_theta=0.72*abs(begin_ind-target_ind)/180*3.1415926;
// P1P2=sqrt(target_range*target_range+beigin_range*beigin_range-2*beigin_range*target_range*cos(detta_theta));
// return P1P2;
// }   
// float pp_to_rectangle_x(float PP_range,int PP_ind)
// {
//    float theta= 0.72*(PP_ind-110)/180*3.1415926;
//    float x= PP_range*cosf64(theta);
//    return x;    
// }
// float pp_to_rectangle_y(float PP_range,int PP_ind)
// {
//    float theta= 0.72*(PP_ind-110)/180*3.1415926;
//    float y= PP_range*sinf64(theta);
//    return y; 
// }
// //贝塞尔曲线拟合
// int binomial(int n, int k) {
//     if (k > n) { // ��ֹ k ���� n
//         return 0;
//     }
//     if (k == 0) { // C(n, 0) = 1
//         return 1;
//     }
//     return binomial(n - 1, k - 1) * n / k;
// }
// //返回一个直角坐标

// void bezier_curve(float *get_arr_x_y,float *range, float *ind_diff, int count,float t) {
//     int n = count - 1;
//     float res_x_temp=0;
//     float res_y_temp=0;
//     for (int i = 0; i <= n; i++) {
//         float b =  binomial(n, i)* powf64(t, i) * powf64(1 - t, n-i);
//        // temp_point = polar_to_cartesian(points[i]);//极坐标化直角坐标
//         res_x_temp=pp_to_rectangle_x(range[i],ind_diff[i]);
//         res_y_temp=pp_to_rectangle_y(range[i],ind_diff[i]);
//         get_arr_x_y[0] = get_arr_x_y[0] + res_x_temp * b;
//         get_arr_x_y[1] = get_arr_x_y[1] + res_y_temp * b;
//     }
// }//贝塞尔曲线绘制


// void fit_and_print(int n, float *range, float *ind_diff,double interval,float* bezier_arr_x, float* bezier_arr_y,int* beizer_count) 
// {
//     double all_distance =0;
//     double new_interval =0;
//     if(n > 1)
//     {
//         for(int i =0;i < (n-1) ;i ++)
//         {
//         //all_distance += get_distance(points[i],points[i+1]);//计算输入点之间的距离
//         all_distance += get_distance_squre_pp(range[i],ind_diff[i],range[i+1],ind_diff[i+1]);//计算输入点之间的距离
//         }
//         new_interval = interval / all_distance;

//         for (float t = 0; t <= 1; t += new_interval) 
//         {
//             float get_point_x_y[2];
//             bezier_curve(get_point_x_y,range, ind_diff, n, t);//获得bezier拟合的直角坐标
//             bezier_arr_x[*beizer_count] = get_point_x_y[0];
//             bezier_arr_y[*beizer_count] = get_point_x_y[1];    //给外面的数组赋值
//             (*beizer_count)++;                         //给坐标++
//         }
//     } 


// }

// typedef struct //极坐标系下的点
// {   
//     double range;
//     double theta;
// }Point_polar;

// typedef struct //直角坐标系下的点
// {
//     double x;
//     double y;
// }Point_rectangular;

// class PubAndSub
// {
// private:
//     ros::NodeHandle n_;

//     ros::Publisher pub_;

//     ros::Subscriber sub_;
// public:
//     PubAndSub()
//     {
//         pub_ = n_.advertise<geometry_msgs::Twist>("/car/cmd_vel",12);// 整合后
//         sub_ = n_.subscribe("/scan",5,&PubAndSub::callback,this);
//     }
//     void callback(const sensor_msgs::LaserScan::ConstPtr &laser);
// };


// Control::PID pid;
// double length;
// double my_last_error=0;
// float new_beigin=0;
// void PubAndSub::callback(const sensor_msgs::LaserScan::ConstPtr &laser)
// {
//     int num = 0;       //做雷达数据拼接用
//     int num_division = 1; //做雷达数据分割用

//     char negetiveNum = 0,positiveNum = 0;
//     int i,j=0;
//     double range = 0,error = 0,negetiveSum = 0,positiveSum = 0;
//     geometry_msgs::Twist twist;
//     Point_polar pp[30] = {0,0};
//     Point_rectangular pr[30] = {0,0};

//     int now_left=0;
//     int now_right=0;

//     float left_point_range [10] ={0};
//     float left_point_ind [10] ={0};
//     int num_of_left_point=0;

//     float right_point_range [10] ={0};
//     float right_point_ind [10] ={0};
//     int num_of_right_point=0;

//      Point_polar pp_begain_left = {0,0};
//      Point_polar pp_begain_right = {0,0};
// for (int i = 780; i <= 998; i+=skip_lase)//300*0.25=75   0.75一个点
//     {
//         if(laser->ranges[i]<5&&laser->ranges[i]>0.001)
//         {
//             new_laser[num] =laser->ranges[i];
           
//         new_laser_not_sort[num] = laser->ranges[i];
       
//         }
//         else 
//         {
//              new_laser[num] =1000;
//              new_laser_not_sort[num]=1000;
//         }
        

//        //ROS_INFO("laser%.2f",new_laser[num]);
//         num++;
//     }
// for (int i = 0; i <= 218; i+=skip_lase)   //300*0.25=75   0.75一个点
//     { 
//         if(laser->ranges[i]<5&&laser->ranges[i]>0.001)
//         {
//           new_laser[num] =laser->ranges[i];
//          new_laser_not_sort[num] = laser->ranges[i];
//         }
   
//         else 
//          {
//              new_laser[num] =1000;
//              new_laser_not_sort[num]=1000;
//         }
//         num++;
//     }
// //2.对雷达数据的距离排序
//       BubbleSort(new_laser,total_lidar_point, ind);
   
//       //ind保存的是  这个点是 从最右点开始转 是第几个 3*1  点
// //3.找到一个距离最短的点  认为是左右点的一个  之后  通过角度范围确定最后的点
//       while (1)
//     {
//         if (abs(ind[num_division + 1] - ind[0]) >90&&get_distance_squre_pp(new_laser[0],ind[0],new_laser[num_division + 1],ind[num_division + 1])>1.35&&get_distance_squre_pp(new_laser[0],ind[0],new_laser[num_division + 1],ind[num_division + 1])<1.75)//100
//         {
//             break;
//         }
//         num_division++;
//         if(num_division>216)
//          break;
//     }
// //4.区分左右锥桶
//     if(ind[num_division + 1] - ind[0]>0)
//     {
//         now_left=ind[num_division + 1];
//         now_right=ind[0];
       

//     pp_begain_left.theta=(ind[num_division + 1]);
//     pp_begain_left.range=new_laser[num_division + 1];

//     pp_begain_right.theta=(ind[0]);
//     pp_begain_right.range=new_laser[0];

//        left_point_range[num_of_left_point]= pp_begain_left.range;
//        left_point_ind [num_of_left_point]=pp_begain_left.theta;
       
//        right_point_range[num_of_right_point]= pp_begain_right.range;
//        right_point_ind [num_of_right_point]=pp_begain_right.theta;
//        num_of_left_point++;
//        num_of_right_point++;

//     }
//     else 
//     {
//         now_right=ind[num_division + 1];
//         now_left=ind[0];

//     pp_begain_right.theta=(ind[num_division + 1]);
//     pp_begain_right.range=new_laser[num_division + 1];

//     pp_begain_left.theta=(ind[0]);
//     pp_begain_left.range=new_laser[0];

//        left_point_range[num_of_left_point]= pp_begain_left.range;
//        left_point_ind [num_of_left_point]=pp_begain_left.theta;
       
//        right_point_range[num_of_right_point]= pp_begain_right.range;
//        right_point_ind [num_of_right_point]=pp_begain_right.theta;
//        num_of_left_point++;
//        num_of_right_point++;
    
//     }
//    //ROS_INFO("L_1_2_car“%.2f",left_point_range[0]);
//     //ROS_INFO(" pp_begain_right.range“%.2f", pp_begain_right.range);
//    // ROS_INFO(" pp_begain_left.range“%.2f", pp_begain_left.range);
//     //ROS_INFO(" left.pp_to_rectangle“%.2f", pp_to_rectangle_x(pp_begain_left.range,pp_begain_left.theta));
//     //ROS_INFO(" right.pp_to_rectangle“%.2f", pp_to_rectangle_x(pp_begain_right.range,pp_begain_right.theta));
//     //ROS_INFO(" left.pp_to_rectangle_Y“%.2f", pp_to_rectangle_y(pp_begain_left.range,pp_begain_left.theta));
//     //ROS_INFO(" right.pp_to_rectangle_Y“%.2f", pp_to_rectangle_y(pp_begain_right.range,pp_begain_right.theta));
    
    
    
//    // ROS_INFO(" angle_by_start_points“%.2f", 0.72*abs(now_left-now_right));
//   //  ROS_INFO("get_distance_squre_pp%.2f",get_distance_squre_pp(pp_begain_right.range,now_right,pp_begain_left.range,now_left));
//   //  ROS_INFO("pp_begain_left.range“%.2f",pp_begain_left.range);
//     // ROS_INFO("dist_squre_left“%.2f",100*(now_left)+0.02);
//   //  ROS_INFO("dist_squre_right“%.2f",100*(now_right)+0.02);
// //5.根据与左右起始锥桶的关系进行左右锥桶划分
// float now_refer_range=0.75;
// int  leaest_num_left_right=0;
// //找左2


// //对找到的点进行筛选




// //贝塞尔拟合
// /*float left_bezier_arr_x[1000];
// float left_bezier_arr_y[1000];
// float right_bezier_arr_x[1000];
// float right_bezier_arr_y[1000];
// float middle_bezier_arr_x[1000];
// float middle_bezier_arr_y[1000];

// int left_bezier_count=0;
// int right_bezier_count=0;
// int middle_bezier_count=0;
// //拟合
// fit_and_print(2, left_point_range, left_point_ind,0.05,left_bezier_arr_x, left_bezier_arr_y,&left_bezier_count);
// fit_and_print(2, right_point_range, right_point_ind,0.05,right_bezier_arr_x, right_bezier_arr_y,&right_bezier_count);
// //获得最小的bezier_count
// int min_bezier_count=0;
// if(left_bezier_count>right_bezier_count)
// {
// min_bezier_count=right_bezier_count;
// }
// else
// {
// min_bezier_count=left_bezier_count;   
// }

// for (int i =0;i<min_bezier_count;i++)
// {
// middle_bezier_arr_x[i]=0.5*(left_bezier_arr_x[i]+right_bezier_arr_x[i]);
// middle_bezier_arr_y[i]=0.5*(left_bezier_arr_y[i]+right_bezier_arr_y[i]);
// middle_bezier_count++;
// }
// ROS_INFO("L_0_2_car“%.2f",left_point_range[0]);
// ROS_INFO("R_0_2_car“%.2f",right_point_range[0]);

// /*ROS_INFO("L_1_2_car“%.2f",left_point_range[1]);
// ROS_INFO("R_1_2_car“%.2f",right_point_range[1]);

// ROS_INFO("L_2_2_car“%.2f",left_point_range[2]);
// ROS_INFO("R_2_2_car“%.2f",right_point_range[2]);

// ROS_INFO("L_3_2_car“%.2f",left_point_range[3]);
// ROS_INFO("R_3_2_car“%.2f",right_point_range[3]);

// ROS_INFO("luck_right%.2f",num_of_left_point+0.01);
// ROS_INFO("luck_left%.2f",num_of_right_point+0.01);*/

// ROS_INFO("error_L%.2f",pp_to_rectangle_y(left_point_range[0],left_point_ind[0]));
// ROS_INFO("error_R%.2f",pp_to_rectangle_y(right_point_range[0],right_point_ind[0]));

// float left_RR_y=pp_to_rectangle_y(left_point_range[0],left_point_ind[0]);
// float right_RR_y=pp_to_rectangle_y(right_point_range[0],right_point_ind[0]);
 
// float theta_error=0;

// theta_error=110-0.5*(left_point_ind[0]+right_point_ind[0]);
 
// error =0.5*(left_RR_y+right_RR_y);

// ROS_INFO("error%.2f",error);
//    // if(abs(error)<0.05)
//    //   twist.linear.x = 1552;
 
//    //  if(left_RR_y<0.3||left_RR_y>1.2)
//   //    twist.linear.x = 1538;
// error=pid.PIDPositional(error);

// theta_error=0.4*theta_error;



   
    
//     //twist.angular.z = error;//将误差换算成角度并发布
//     if(twist.angular.z > 180)
//         twist.angular.z = 180;
//     if(twist.angular.z < 0)
//         twist.angular.z = 0;

//     twist.angular.z = 88+error;//将误差换算成角度并发布
//     twist.linear.x = 1557;//速度
//     twist.angular.y = 0;
//     twist.angular.x = 0;
//     twist.linear.y = 0;
//     twist.linear.z = 0;
//     ROS_INFO("twist.linear.x%.2f",twist.linear.x);

//     pub_.publish(twist);
//     // pub_ = twist;// 整合后
// }
    

// int main(int argc, char *argv[])
// {
//     pid.Init();
//     ros::init(argc,argv,"laser_go");
//     PubAndSub PAS;
//     ros::spin();
//     return 0;
// }//以上为源程序















//**********************两圈雷达******************//
// // // 在移植过程中需要注意：
// // // 1. 修改 car_control_new 发布的速度话题名:由 /car/cmd_vel 改为 /car/nav_vel(136-137行)
// // // 2. 修改 get_pos_sub 订阅的话题名：由 position 改为 /cmd_position
// // // 3. 修改 racecar 订阅的话题名：由 /car/cmd_vel 改为 /car/vel
// // // 4. 待 camera 节点写完后，注意修改 camera 节点中的发布话题名称:/cam_flag
// // // 5. 修改 对应工作空间 CMakeLists.txt，为 midguanli_new.cpp 添加依赖项
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <ros/spinner.h>
#include <thread>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include "Control.h"

#define freq 1440//每转一圈雷达扫描次数
#define speed 1875
#define skip_lase 2
#define total_lidar_point 220
// geometry_msgs::Twist pub_;// 整合后
float new_laser[total_lidar_point] = {0}; //拼接后的数据  1140-1440 + 0-300   前方150度的数据，逆时针
float new_laser_not_sort[total_lidar_point] = {1000}; //拼接后的数据  1140-1440 + 0-300   前方150度的数据，逆时针
float new_laser_get_left[total_lidar_point] = {1000}; //拼接后的数据  1140-1440 + 0-300   前方150度的数据，逆时针
float new_laser_get_right[total_lidar_point] = {1000}; //拼接后的数据  1140-1440 + 0-300   前方150度的数据，逆时针
int new_laser_get_left_ind[total_lidar_point] = {0}; 
int new_laser_get_right_ind[total_lidar_point] = {0}; 
int ind[total_lidar_point] = {0};         //排序后元素对应的位置
void BubbleSort_have_ind(float *p, int length, int *ind_diff)
{


    for (int i = 0; i < length; i++)
    {
        for (int j = 0; j < length - i - 1; j++)
        {
            if (p[j] > p[j + 1])
            {
                float temp = p[j];
                p[j] = p[j + 1];
                p[j + 1] = temp;

                int ind_temp = ind_diff[j];
                ind_diff[j] = ind_diff[j + 1];
                ind_diff[j + 1] = ind_temp;    //排序后保存角度信息
            }
        }
    }
}   
void BubbleSort(float *p, int length, int *ind_diff)
{
    for (int m = 0; m < length; m++)
    {
        ind_diff[m] = m;
    }  

    for (int i = 0; i < length; i++)
    {
        for (int j = 0; j < length - i - 1; j++)
        {
            if (p[j] > p[j + 1])
            {
                float temp = p[j];
                p[j] = p[j + 1];
                p[j + 1] = temp;

                int ind_temp = ind_diff[j];
                ind_diff[j] = ind_diff[j + 1];
                ind_diff[j + 1] = ind_temp;    //排序后保存角度信息
            }
        }
    }
}   
double get_distance_squre_pp(float target_range,int target_ind,float beigin_range,int begin_ind)
{
double P1P2=0;
double detta_theta=0.72*abs(begin_ind-target_ind)/180*3.1415926;
P1P2=sqrt(target_range*target_range+beigin_range*beigin_range-2*beigin_range*target_range*cos(detta_theta));
return P1P2;
}   
float pp_to_rectangle_x(float PP_range,int PP_ind)
{
   float theta= 0.72*(PP_ind-110)/180*3.1415926;
   float x= PP_range*cosf64(theta);
   return x;    
}
float pp_to_rectangle_y(float PP_range,int PP_ind)
{
   float theta= 0.72*(PP_ind-110)/180*3.1415926;
   float y= PP_range*sinf64(theta);
   return y; 
}
//贝塞尔曲线拟合
int binomial(int n, int k) {
    if (k > n) { // ��ֹ k ���� n
        return 0;
    }
    if (k == 0) { // C(n, 0) = 1
        return 1;
    }
    return binomial(n - 1, k - 1) * n / k;
}
//返回一个直角坐标

void bezier_curve(float *get_arr_x_y,float *range, float *ind_diff, int count,float t) {
    int n = count - 1;
    float res_x_temp=0;
    float res_y_temp=0;
    for (int i = 0; i <= n; i++) {
        float b =  binomial(n, i)* powf64(t, i) * powf64(1 - t, n-i);
       // temp_point = polar_to_cartesian(points[i]);//极坐标化直角坐标
        res_x_temp=pp_to_rectangle_x(range[i],ind_diff[i]);
        res_y_temp=pp_to_rectangle_y(range[i],ind_diff[i]);
        get_arr_x_y[0] = get_arr_x_y[0] + res_x_temp * b;
        get_arr_x_y[1] = get_arr_x_y[1] + res_y_temp * b;
    }
}//贝塞尔曲线绘制

void fit_and_print(int n, float *range, float *ind_diff,double interval,float* bezier_arr_x, float* bezier_arr_y,int* beizer_count) 
{
    double all_distance =0;
    double new_interval =0;
    if(n > 1)
    {
        for(int i =0;i < (n-1) ;i ++)
        {
        //all_distance += get_distance(points[i],points[i+1]);//计算输入点之间的距离
        all_distance += get_distance_squre_pp(range[i],ind_diff[i],range[i+1],ind_diff[i+1]);//计算输入点之间的距离
        }
        new_interval = interval / all_distance;

        for (float t = 0; t <= 1; t += new_interval) 
        {
            float get_point_x_y[2];
            bezier_curve(get_point_x_y,range, ind_diff, n, t);//获得bezier拟合的直角坐标
            bezier_arr_x[*beizer_count] = get_point_x_y[0];
            bezier_arr_y[*beizer_count] = get_point_x_y[1];    //给外面的数组赋值
            (*beizer_count)++;                         //给坐标++
        }
    } 
}

typedef struct //极坐标系下的点
{   
    double range;
    double theta;
}Point_polar;

typedef struct //直角坐标系下的点
{
    double x;
    double y;
}Point_rectangular;

class PubAndSub
{
public:
    ros::NodeHandle nh_;
    ros::MultiThreadedSpinner spinner_;
    ros::Subscriber sub_;

    ros::Subscriber nav_sub;
    ros::Publisher vel_pub;

    ros::Subscriber cam_sub;

    ros::Subscriber position_sub;
    ros::Publisher position_pub;

public:
    PubAndSub(): spinner_(0) { }
    void initialize()
    {
        // 创建管家
        nh_ = ros::NodeHandle();
        // 定义多线程变量
        spinner_=ros::MultiThreadedSpinner(0);

        // 订阅 激光雷达 速度信息(<--laser)
        sub_ = nh_.subscribe("/scan",5,&PubAndSub::callback,this);
        // // 订阅 导航 速度信息(<--car_control_new)
        // nav_sub = nh_.subscribe("/car/nav_vel", 5, &PubAndSub::NavCallback, this);
        // 发布 速度话题(-->racecar)
        vel_pub = nh_.advertise<geometry_msgs::Twist>("/car/vel", 10);

        // 订阅 摄像头 信息
        cam_sub = nh_.subscribe("/change", 3, &PubAndSub::CamCallback, this);

        // // 订阅 位置 信息(get_pos_sub)
        // position_sub = nh_.subscribe("position", 5, &PubAndSub::PositionCallback, this);
        // // 发布 位置 信息(get_pos_pub)
        // position_pub = nh_.advertise<geometry_msgs::PoseStamped>("/cmd_position", 10);
    }
    // 激光雷达sub回调函数
    void callback(const sensor_msgs::LaserScan::ConstPtr &laser);
    // // 导航sub回调函数
    // void NavCallback(const geometry_msgs::Twist& twist);
    // 摄像头sub回调函数
    void CamCallback(const std_msgs::Int8& cam_flag);
    // // 位置sub回调函数
    // void PositionCallback(const geometry_msgs::PoseStamped& position);

    // void LaserCallback(const geometry_msgs::Twist& twist);
};

Control::PID pid;
double length;
double my_last_error=0;
float new_beigin=0;

void LaserCallback(const geometry_msgs::Twist& twist);

// flag 模式切换标志位
// flag = 0 雷达巡线 记点
// flag = 1 停车3s后，flag = 2
// flag = 2 导航巡线
// flag = 3 停车
int flag=0;

void PubAndSub::callback(const sensor_msgs::LaserScan::ConstPtr &laser)
{
    geometry_msgs::Twist twist;
    if(flag == 0||flag == 2)
    {
    int num = 0;       //做雷达数据拼接用
    int num_division = 1; //做雷达数据分割用

    char negetiveNum = 0,positiveNum = 0;
    int i,j=0;
    double range = 0,error = 0,negetiveSum = 0,positiveSum = 0;

    Point_polar pp[30] = {0,0};
    Point_rectangular pr[30] = {0,0};

    int now_left=0;
    int now_right=0;

    float left_point_range [10] ={0};
    float left_point_ind [10] ={0};
    int num_of_left_point=0;

    float right_point_range [10] ={0};
    float right_point_ind [10] ={0};
    int num_of_right_point=0;

    Point_polar pp_begain_left = {0,0};
    Point_polar pp_begain_right = {0,0};
    for (int i = 780; i <= 998; i+=skip_lase)//300*0.25=75   0.75一个点
        {
            if(laser->ranges[i]<5&&laser->ranges[i]>0.001)
            {
            new_laser[num] =laser->ranges[i];
            
            new_laser_not_sort[num] = laser->ranges[i];
           new_laser_get_left[num] = laser->ranges[i];
           new_laser_get_right[num] = laser->ranges[i];
            }
            else 
            {
                new_laser[num] =1000;
                new_laser_not_sort[num]=1000;
            new_laser_get_left[num] = 1000;
           new_laser_get_right[num] = 1000;
            }
            

        //ROS_INFO("laser%.2f",new_laser[num]);
            num++;
        }
    for (int i = 0; i <= 218; i+=skip_lase)   //300*0.25=75   0.75一个点
        { 
            if(laser->ranges[i]<5&&laser->ranges[i]>0.001)
            {
            new_laser[num] =laser->ranges[i];
            new_laser_not_sort[num] = laser->ranges[i];
            new_laser_get_left[num] = laser->ranges[i];
           new_laser_get_right[num] = laser->ranges[i];
            }
    
            else 
            {
                new_laser[num] =1000;
                new_laser_not_sort[num]=1000;
            new_laser_get_left[num] = 1000;
           new_laser_get_right[num] = 1000;
            }
            num++;
        }
//2.对雷达数据的距离排序
      BubbleSort(new_laser,total_lidar_point, ind);
   
      //ind保存的是  这个点是 从最右点开始转 是第几个 3*1  点
//3.找到一个距离最短的点  认为是左右点的一个  之后  通过角度范围确定最后的点
      while (1)
    {
        if (abs(ind[num_division + 1] - ind[0]) >90&&get_distance_squre_pp(new_laser[0],ind[0],new_laser[num_division + 1],ind[num_division + 1])>1.35&&get_distance_squre_pp(new_laser[0],ind[0],new_laser[num_division + 1],ind[num_division + 1])<1.75)//100
        {
            break;
        }
        num_division++;
        if(num_division>216)
        {
            break;         
        }

    }
//4.区分左右锥桶
    if(ind[num_division + 1] - ind[0]>0)
    {
        now_left=ind[num_division + 1];
        now_right=ind[0];
       

        pp_begain_left.theta=(ind[num_division + 1]);
        pp_begain_left.range=new_laser[num_division + 1];

        pp_begain_right.theta=ind[0];
        pp_begain_right.range=new_laser[0];

       left_point_range[0]= pp_begain_left.range;
       left_point_ind [0]=ind[num_division + 1];
       
       right_point_range[0]= pp_begain_right.range;
       right_point_ind [0]=ind[0];
       num_of_left_point++;
       num_of_right_point++;

    }
    else 
    {
        now_right=ind[num_division + 1];
        now_left=ind[0];

        pp_begain_right.theta=ind[num_division + 1];
        pp_begain_right.range=new_laser[num_division + 1];

        pp_begain_left.theta=(ind[0]);
        pp_begain_left.range=new_laser[0];

       left_point_range[0]= pp_begain_left.range;
       left_point_ind [0]=pp_begain_left.theta;
       
       right_point_range[0]= pp_begain_right.range;
       right_point_ind [0]=ind[num_division + 1];
       num_of_left_point++;
       num_of_right_point++;
    
    }
//5.根据与左右起始锥桶的关系进行左右锥桶划分
   /*num_division=-1;
    for (int i = 0; i <= 218; i+=1)   //300*0.25=75   0.75一个点
        { 
        if(new_laser_get_left[i]<4.9)
        new_laser_get_left[i]=get_distance_squre_pp(left_point_range[0],left_point_ind[0],new_laser_get_left[i],i);//相对距离
        }

     BubbleSort(new_laser_get_left,total_lidar_point, new_laser_get_left_ind);//排序找最小
      while (1)
    {
        if (new_laser_get_left[num_division+1]>0.7&&new_laser_get_left[num_division+1]<1.55)//100
        {
           if(abs(new_laser_get_left_ind [num_division+ 1]-left_point_ind[1])<55&&new_laser_get_left[num_division+1]<get_distance_squre_pp(right_point_range[0],right_point_ind[0],new_laser[new_laser_get_left_ind [num_division+ 1]],new_laser_get_left_ind [num_division+ 1]))
            {
                left_point_range[1]=new_laser_not_sort[new_laser_get_left_ind [num_division+ 1]];
                left_point_ind[1]=new_laser_get_left_ind [num_division+ 1];
                break;
            }
        }
        num_division++;
        if(num_division>216)
        {
            break;         
        }

    }


    num_division=-1;
    for (int i = 0; i <= 218; i+=1)   //300*0.25=75   0.75一个点
        { 
        new_laser_get_right[i]=get_distance_squre_pp(right_point_range[0],right_point_ind[0],new_laser_get_right[i],i);
        }

     BubbleSort(new_laser_get_right,total_lidar_point, new_laser_get_right_ind);
      while (1)
    {
        if (abs(new_laser_get_right_ind [num_division+ 1]-right_point_ind[0])>2&&new_laser_get_right[num_division+1]>0.7&&new_laser_get_right[num_division+1]<1.55)//100
        {
           if(abs(new_laser_get_right_ind [num_division+ 1]-right_point_ind[1])<55&&new_laser_get_right[num_division+1]<get_distance_squre_pp(left_point_range[0],left_point_ind[0],new_laser[new_laser_get_right_ind [num_division+ 1]],new_laser_get_right_ind [num_division+ 1]))
            {
                right_point_range[1]=new_laser_not_sort[new_laser_get_right_ind [num_division+ 1]];
                right_point_ind[1]=new_laser_get_right_ind [num_division+ 1];
                    break;
            }
        }
        num_division++;
        if(num_division>216)
        {
            break;         
        }

    }

*/

    float now_refer_range=0.75;
    int  leaest_num_left_right=0;
//找左2

    //贝塞尔拟合
    /*float left_bezier_arr_x[1000];
    float left_bezier_arr_y[1000];
    float right_bezier_arr_x[1000];
    float right_bezier_arr_y[1000];
    float middle_bezier_arr_x[1000];
    float middle_bezier_arr_y[1000];

    int left_bezier_count=0;
    int right_bezier_count=0;
    int middle_bezier_count=0;
    //拟合
    fit_and_print(2, left_point_range, left_point_ind,0.05,left_bezier_arr_x, left_bezier_arr_y,&left_bezier_count);
    fit_and_print(2, right_point_range, right_point_ind,0.05,right_bezier_arr_x, right_bezier_arr_y,&right_bezier_count);
    //获得最小的bezier_count
    int min_bezier_count=0;
    if(left_bezier_count>right_bezier_count)
    {
    min_bezier_count=right_bezier_count;
    }
    else
    {
    min_bezier_count=left_bezier_count;   
    }

    for (int i =0;i<min_bezier_count;i++)
    {
    middle_bezier_arr_x[i]=0.5*(left_bezier_arr_x[i]+right_bezier_arr_x[i]);
    middle_bezier_arr_y[i]=0.5*(left_bezier_arr_y[i]+right_bezier_arr_y[i]);
    middle_bezier_count++;
    }
    ROS_INFO("L_0_2_car“%.2f",left_point_range[0]);
    ROS_INFO("R_0_2_car“%.2f",right_point_range[0]);

    /*ROS_INFO("L_1_2_car“%.2f",left_point_range[1]);
    ROS_INFO("R_1_2_car“%.2f",right_point_range[1]);

    ROS_INFO("L_2_2_car“%.2f",left_point_range[2]);
    ROS_INFO("R_2_2_car“%.2f",right_point_range[2]);

    ROS_INFO("L_3_2_car“%.2f",left_point_range[3]);
    ROS_INFO("R_3_2_car“%.2f",right_point_range[3]);*/

    ROS_INFO("luck_right%.2f",num_of_left_point+0.01);
    ROS_INFO("luck_left%.2f",num_of_right_point+0.01);

    ROS_INFO("right_point_ind111:%.2f",right_point_ind[1]);
      ROS_INFO("right_point_ind:%.2f",right_point_ind[0]);
    ROS_INFO("right_point_dis111:%.2f",right_point_range[1]);
        ROS_INFO("right_point_dis:%.2f",right_point_range[0]);
    ROS_INFO("left_point_ind111:%.2f",left_point_ind[1]);
        ROS_INFO("left_point_ind:%.2f",left_point_ind[0]);
    ROS_INFO("left_point_dis111:%.2f",left_point_range[1]);
    ROS_INFO("left_point_dis:%.2f",left_point_range[0]);
    float left_RR_y=pp_to_rectangle_y(left_point_range[0],left_point_ind[0]);
    float right_RR_y=pp_to_rectangle_y(right_point_range[0],right_point_ind[0]);
    float left_RR_y_1=pp_to_rectangle_y(left_point_range[1],left_point_ind[1]);
    float right_RR_y_1=pp_to_rectangle_y(right_point_range[1],right_point_ind[1]);
    float theta_error=0;

    theta_error=110-0.5*(left_point_ind[0]+right_point_ind[0]);
    
    error =0.5*(left_RR_y+right_RR_y);

  //  ROS_INFO("error%.2f",error);
    error=pid.PIDPositional(error);

    theta_error=0.4*theta_error;

    //twist.angular.z = error;//将误差换算成角度并发布
    if(twist.angular.z > 180)
        twist.angular.z = 180;
    if(twist.angular.z < 0)
        twist.angular.z = 0;

    twist.angular.z = 80+error;//将误差换算成角度并发布
    twist.linear.x = 1550;//速度
  //  if(abs(error>30))
  //  twist.linear.x -= 2;//速度
   // if(abs(error<5))
    //twist.linear.x+=2;//!!!!!!!!!!!!!!!!!!!加速的时候去掉这句话

    twist.angular.y = 0;

    twist.angular.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    ROS_INFO("twist.linear.x:%.2f",twist.linear.x);
    ROS_INFO("twist.angular.z:%.2f",twist.angular.z);
    }

    ROS_INFO("flag =%d",flag);
    if(flag == 0)
    {
        vel_pub.publish(twist);// publish函数每执行一次发布一条消息
        ROS_INFO("get_laser_info!");
    
    }
    else if(flag == 1)
    {
        ROS_INFO("get_stop_info!");
        geometry_msgs::Twist twist_stop;
        twist_stop.linear.x=1500;
        twist_stop.linear.y=0;
        twist_stop.linear.z=0;
        twist_stop.angular.x=0;
        twist_stop.angular.y=0;
        twist_stop.angular.z=88;
    
        for(int i = 0;i<300000;i++)
        {
            vel_pub.publish(twist_stop);
        }
        // sleep(3.5);
        flag = 2;       // 模式切换标志位 置2 切换模式（第二圈）
        ROS_INFO("start_to_navigate!");
    }
    // flag == 2 两圈雷达
    else if(flag == 2)
    {
        vel_pub.publish(twist);// publish函数每执行一次发布一条消息
        ROS_INFO("get_laser");
    }
    else if(flag == 3)
    {
        geometry_msgs::Twist twist_end;
        twist_end.linear.x=1500;
        twist_end.linear.y=0;
        twist_end.linear.z=0;
        twist_end.angular.x=0;
        twist_end.angular.y=0;
        twist_end.angular.z=90;
        // sleep(0.5)
        vel_pub.publish(twist_end);
        ROS_INFO("reach_end!");
    }
    // LaserCallback(twist);

    // pub_.publish(twist);
    // pub_ = twist;// 整合后
}

void start_nav()
{
    system("python3 racecar/src/multi_goal/src/goal_loop.py");
}
void save_map()
{
    system("cd racecar/src/racecar;rosrun map_server map_saver -f /racecar/src/racecar/map/map_10");
}

// // 雷达巡线回调函数
// void PubAndSub::LaserCallback(const geometry_msgs::Twist& twist)
// {
//     ROS_INFO("flag =%d",flag);
//     if(flag == 0)
//     {
//         vel_pub.publish(twist);// publish函数每执行一次发布一条消息
//         ROS_INFO("get_laser_info!");
//     }
//     else if(flag == 3)
//     {
//         ROS_INFO("get_stop_info!");
//         geometry_msgs::Twist twist_stop;
//         twist_stop.linear.x=1500;
//         twist_stop.linear.y=0;
//         twist_stop.linear.z=0;
//         twist_stop.angular.x=0;
//         twist_stop.angular.y=0;
//         twist_stop.angular.z=88;
//         vel_pub.publish(twist_stop);
//         sleep(3.5);
//         flag = 2;       // 模式切换标志位 置2 切换模式（第二圈）
//         ROS_INFO("start_to_navigate!");
//     }
//     // flag == 2 两圈雷达
//     else if(flag == 2)
//     {
//         vel_pub.publish(twist);// publish函数每执行一次发布一条消息
//         ROS_INFO("get_laser");
//     }
//     else if(flag == 1)
//     {
//         geometry_msgs::Twist twist_end;
//         twist_end.linear.x=1500;
//         twist_end.linear.y=0;
//         twist_end.linear.z=0;
//         twist_end.angular.x=0;
//         twist_end.angular.y=0;
//         twist_end.angular.z=90;
//         vel_pub.publish(twist_end);
//         ROS_INFO("reach_end!");
//     }
// }

// int cnt = 0;
// // 导航巡线回调函数
// void PubAndSub::NavCallback(const geometry_msgs::Twist& twist)
// {
//     // if(flag == 2)
//     // {
//     //     ROS_INFO("start_navigation!");
//     //     vel_pub.publish(twist);
//     // }
//     // if(flag == 2&& cnt == 0)
//     // {
//     //     thread th6(save_map);
//     //     th6.detach();
//     //     thread th4(start_nav);
//     //     th4.detach();
//     //     cnt = 1;
//     // }
// }

// // 位置节点回调函数
// void PubAndSub::PositionCallback(const geometry_msgs::PoseStamped& position)
// {
//     if(flag == 0)
//     {
//         ROS_INFO("start_write_point!");
//         position_pub.publish(position);
//     }
//     else
//     {
//         ROS_INFO("finish_writing!");
//     }
// }

// 相机节点回调函数
void PubAndSub::CamCallback(const std_msgs::Int8& cam_flag)
{
    ROS_INFO("flag_sub-->cam_flag:%d",cam_flag.data);
    flag = cam_flag.data;
}

int main(int argc, char *argv[])
{
    pid.Init();
    ros::init(argc,argv,"laser_go");
    PubAndSub pub_and_sub;
    pub_and_sub.initialize();
    ros::spin();
    return 0;
}
