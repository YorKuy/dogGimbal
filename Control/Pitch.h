#ifndef __PITCH_H__
#define __PITCH_H__

#include "RM_Lib.h"
#include "main.h"
extern u8 PITCH_Mode;
extern RC YK;
extern BMI088 GIMBAL_088;
extern Vision_LPF Pitch_PID_OUT;
class PITCH
{
    public:
        f Pitch_Out;
        f Target_Angle;
        f Pitch_Out_Interface(u8 jianshu_flag);
        void set_Pitch_Target(f Target_Angle);
        void set_SMCref(f Target_Angle);
    PITCH():Pitch_Out(0),Pitch_real(0),Pitch_vel(0),Pitch_acc(0),Target_Angle(0),Angle_buf(0),Last_Angle(0){}
    private:
        f Pitch_real,Pitch_vel,Pitch_acc;
        f Angle_buf;
        f Last_Angle;
};

#endif
