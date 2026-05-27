#ifndef __YAW_H__
#define __YAW_H__

#include "main.h"
#include "RM_Lib.h"

extern u8 YAW_Mode;
extern RC YK;
extern u16 Communicate_Send_Flag_1;
extern BMI088	GIMBAL_088;
class YAW
{
    public:      
        f Yaw_Out;        //Yaw最后的输出
        f Target_Angle;   //目标角度
        void Yaw_communicate();
        void set_SMCref(f Target_Angle);
        f Yaw_Out_Interface(u8 jianshu_flag);
        void set_Yaw_Angle(f Target_Angle);
    YAW():Yaw_Out(0),Target_Angle(0),Yaw_real(0),Yaw_vel(0),Yaw_acc(0),Back_Flag(0){}
    private:
        f Yaw_real,Yaw_vel,Yaw_acc;  //陀螺仪实际角度，角速度，角加速度
        u8 Back_Flag;
        
};
#endif
