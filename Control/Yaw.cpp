#include "Yaw.h"
#include "SMC.h"
#include "RM.h"
#include "PID.h"
#include "DM.h"

#define YAW_ST 6600
UpDown_check_class UD_Yaw_Back(0);
SMC Yaw(40,60, 0, 0.01, 12000, 0.9, 1, 1),
    Yaw_Zm(20, 60, 0, 0.001, 12000, 0.9, 1, 1),
    Yaw_Back(15, 70, 0, 0.001, 16000, 0.9, 1, 1);
PID_class yaw_angle(20, 0, 0, 450, 0, 10, 500),
          yaw_speed(0.02, 0, 0, 10, 0, 0, 10),
          yaw_back_angle(30,0,10,450,0,10,500),
          yaw_back_speed(0.035,0,0,10,0,0,10),
          yaw_angle_zm(30,0,0,450,0,0,500),
          yaw_speed_zm(0.025,0,0,5,0,0,5);
YAW *yaw = new YAW();
extern MOTOR_RM M6020_YAW;
extern MOTOR_DM DM_YAW;
extern float Zm_Yaw_Vel, Zm_Yaw_Acc;
static float error_yaw;
float gm6020to_torq(float u)
{
    float A = u / (16384.0f / 3.0f);
    float nm = A * 0.741f * 4.0f;
    return nm;
}
f YAW::Yaw_Out_Interface(u8 jianshu_flag)
{
    if (YAW_Mode == GYRO_MODE)
    {
        yaw->Target_Angle -= jianshu_flag ? (float)(LIMIT(YK.shubiao.x, -400, 400) / 1600.0) : (float)YK.yaogan.ch2 / 6600.0;
        if (UD_Yaw_Back.updata(YK.Pressed_Check(KEY_PRESSED_R)) == UpDown_check_rising)
        {
            yaw->Target_Angle += 180.0;
            yaw->Back_Flag = 1;
            Communicate_Send_Flag_1 |= (0x0001 << 1);
        }

        if (yaw->Back_Flag)
        {
            if (abs(yaw->Target_Angle - GIMBAL_088.realAngle.yaw) > 5.0f)
            {
               Yaw_Back.SMC_Tick(GIMBAL_088.realAngle.yaw, GIMBAL_088.Anglespeed.Deal_yaw);
                yaw->Yaw_Out = gm6020to_torq(Yaw_Back.u);
//                  yaw_back_angle.PID_update_LP(yaw->Target_Angle, GIMBAL_088.realAngle.yaw, 20);
//                yaw_back_speed.PID_new_update(yaw_back_angle.OUT_PID, GIMBAL_088.Anglespeed.Deal_yaw);
//                yaw->Yaw_Out = yaw_back_speed.OUT_PID;
            }
            else
            {
                yaw->Back_Flag = 0;
                Communicate_Send_Flag_1 &= ~(0x0001 << 1);
            }
        }
        else
        {
            error_yaw = Yaw.ref - GIMBAL_088.realAngle.yaw;
            Yaw.SMC_Tick(GIMBAL_088.realAngle.yaw, GIMBAL_088.Anglespeed.Deal_yaw);
            yaw->Yaw_Out = gm6020to_torq(Yaw.u);
            // yaw_angle.PID_update_LP(yaw->Target_Angle, GIMBAL_088.realAngle.yaw, 20);
            // yaw_speed.PID_new_update(yaw_angle.OUT_PID, GIMBAL_088.Anglespeed.Deal_yaw);
            // yaw->Yaw_Out = yaw_speed.OUT_PID;
        }
    }
    else if (YAW_Mode == AUTO_MODE)
    {
        Yaw_Zm.SMC_AngleSpeed(yaw->Target_Angle, Zm_Yaw_Vel, Zm_Yaw_Acc, GIMBAL_088.realAngle.yaw, GIMBAL_088.Anglespeed.Deal_yaw);
        yaw->Yaw_Out = gm6020to_torq(Yaw_Zm.u);
    }
    else
    {
        yaw->Yaw_Out = 0;
        yaw->Target_Angle = GIMBAL_088.realAngle.yaw;
    }

    return yaw->Yaw_Out;
}

void YAW::set_SMCref(f Target_Angle)
{
    Yaw.ref = Target_Angle;
    Yaw_Zm.ref = Target_Angle;
    Yaw_Back.ref = Target_Angle;
}
void YAW::set_Yaw_Angle(f Target_Angle)
{
    yaw->Target_Angle = Target_Angle;
}