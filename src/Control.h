namespace Control
{
#include "stdio.h"
#include "math.h"
#include "assert.h"
    typedef struct
{
	float X;
	float Y;
} PointF;
    class PID 
    {
    public:
        double kp;
        double ki;
        double kd;
        double error_out;
        double last_error;
        double integral;
        double inte_max;//积分限副
        double last_diff;//上一次微分值

        double PIDPositional(double error);//位置式PID控制器
        double PIDIncremental(double error);
        void Init();
        PointF bezier_interpolation_func(float t, PointF* points, const int count);
        void  draw_bezier_curves(PointF* points, int count, PointF* out_points, int out_count);
    };
    
    double PID::PIDPositional(double error)//位置式PID控制器
    {
        integral += error;

        if(integral>inte_max)
            integral = inte_max;

        error_out = (kp*error) + (ki*integral) + (kd*(error-last_error));
        last_error = error;
        return error_out;
    }
    
    double PID::PIDIncremental(double error)//增量式PID控制器
    {
        error_out = kp*(error-last_error) + ki*error + kd*((error-last_error)-last_diff);

        last_diff = error - last_error;
        last_error = error;

        return error_out;
    }

    void PID::Init()//PID初始化,参数在此调节
    {
        kp =250;//205
        ki = 0.0;
        kd = 210;//210
        error_out = 0.0;
        last_error = 0.0;
        integral = 0.0;
        inte_max = 8.0;
        last_diff = 0.0;
        // kp =18.0;s
        // ki = 0.1;
        // kd = 3.0;
        // error_out = 0.0;
        // last_error = 0.0;
        // integral = 0.0;
        // inte_max = 8.0;
        // last_diff = 0.0;
    }


PointF PID::bezier_interpolation_func(float t, PointF* points, const int count)
{
    PointF tmp_points[200];
	if(count>0){
	
	for (int i = 1; i < count; ++i)
	{
		for (int j = 0; j < count - i; ++j)
		{
			if (i == 1)
			{
				tmp_points[j].X = (float)(points[j].X * (1 - t) + points[j + 1].X * t);
				tmp_points[j].Y = (float)(points[j].Y * (1 - t) + points[j + 1].Y * t);
				continue;
			}
			tmp_points[j].X = (float)(tmp_points[j].X * (1 - t) + tmp_points[j + 1].X * t);
			tmp_points[j].Y = (float)(tmp_points[j].Y * (1 - t) + tmp_points[j + 1].Y * t);
		}
	}
    
    }
   
        return tmp_points[0];
   
    
	
}

void PID::draw_bezier_curves(PointF* points, int count, PointF* out_points, int out_count)
{

    PointF points_real[count]={0,0};
    for(int i=0;i<count;++i)
    {
    points_real[i]=points[i];
    }
    
	float step = 1.0 / out_count;
	float t = 0;
	for (int i = 0; i < out_count; i++)
	{
		PointF temp_point = bezier_interpolation_func(t, points, count);    // 计算插值点
		t += step;
		out_points[i] = temp_point;
	}
}

}
/*namespace Control
{
    class PID 
    {
    public:
        double kp;
        double ki;
        double kd;
        double error_out;
        double last_error;
        double integral;
        double inte_max;//积分限副
        double last_diff;//上一次微分值

        double PIDPositional(double error);//位置式PID控制器
        double PIDIncremental(double error);
        void Init();
        
    };
    
    double PID::PIDPositional(double error)//位置式PID控制器
    {
        integral += error;

        if(integral>inte_max)
            integral = inte_max;

        error_out = (kp*error) + (ki*integral) + (kd*(error-last_error));
        last_error = error;
        return error_out;
    }
    
    double PID::PIDIncremental(double error)//增量式PID控制器
    {
        error_out = kp*(error-last_error) + ki*error + kd*((error-last_error)-last_diff);

        last_diff = error - last_error;
        last_error = error;

        return error_out;
    }

    void PID::Init()//PID初始化,参数在此调节
    {
        kp=17;
        ki = 0.0;
        kd =0.0;
        error_out = 0.0;
        last_error = 0.0;
        integral = 0.0;
        inte_max = 0.0;
        last_diff = 0.0;
        // kp =18.0;
        // ki = 0.1;
        // kd = 3.0;
        // error_out = 0.0;
        // last_error = 0.0;
        // integral = 0.0;
        // inte_max = 8.0;
        // last_diff = 0.0;
    }
    
}*/