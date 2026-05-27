#include "Pitch.h"
#include "DM.h"
#include "PID.h"
#include "SMC.h"

#define PITCH_HIGH   0.3003F
#define PITCH_LOW    1.1349F
#define PITCH_UP_GRAVITY_ERR_DB_DEG        1.0f
#define PITCH_UP_GRAVITY_ERR_RANGE_DEG     9.0f
#define PITCH_UP_GRAVITY_LOW_ANGLE_DEG    -12.0f
#define PITCH_UP_GRAVITY_HIGH_ANGLE_DEG    30.0f
#define PITCH_UP_GRAVITY_COMP_BASE         0.3f
#define PITCH_UP_GRAVITY_COMP_LOW_K        2.0f
#define PITCH_UP_GRAVITY_COMP_MAX          2.0f
PID_class M6020_Pitch_Speed(0.018f, 0, 0, 5.0f, 0, 0, 5.0f), // 180,0.3,0,0,10000,0,30000,2,50
    M6020_Pitch_Angle(20.0f, 0, 10.0f, 350.0f, 0, 10.0f, 400.0f);   // 15,0,0,0,10000,0,30000
PID_class PID_Pitch_sp_zm(0.025, 0.1, 0, 5, 3, 0, 6,3),
    PID_Pitch_mang_zm(20, 1, 20, 400, 100, 30, 400,4.0f); //
SMC         Pitch(45,70,0,0.001,15000,0.9,1,1),
            Pitch_Zm(60, 130, 0, 0.1, 16000, 1, 1, 1);
SMC_PITCH SMC_Pitch(40,70, 0.15f, 0.001f, 12000, 0.8f, 1),
            SMC_Pitch_Zm(38,60, 0.5f, 0.001f, 12000, 0.8f, 1.0f);
PITCH *pitch = new PITCH();
extern MOTOR_DM DM_PITCH;
extern float Zm_Pitch_Vel, Zm_Pitch_Acc;
float gm6020to_torq_pitch(float u)
{
    float A = u / (16384.0f / 3.0f);
    float nm = A * 0.741f * 4.0f;
    return nm;
}
static inline float GetPitchUpGravityComp(float target_angle_deg, float current_angle_deg)
{
    const float up_error_deg = target_angle_deg - current_angle_deg;
    if (up_error_deg <= PITCH_UP_GRAVITY_ERR_DB_DEG)
        return 0.0f;
    const float low_factor = LIMIT((PITCH_UP_GRAVITY_HIGH_ANGLE_DEG - current_angle_deg) /
                                    (PITCH_UP_GRAVITY_HIGH_ANGLE_DEG - PITCH_UP_GRAVITY_LOW_ANGLE_DEG),0.0f,1.0f);

    const float error_factor = LIMIT((up_error_deg - PITCH_UP_GRAVITY_ERR_DB_DEG) 
                                    /PITCH_UP_GRAVITY_ERR_RANGE_DEG,0.0f,1.0f);

    float comp = PITCH_UP_GRAVITY_COMP_BASE + PITCH_UP_GRAVITY_COMP_LOW_K * low_factor;
    comp *= error_factor;
    return LIMIT(comp, 0.0f, PITCH_UP_GRAVITY_COMP_MAX);
}
f PITCH::Pitch_Out_Interface(u8 jianshu_flag)
{
    if (PITCH_Mode == GYRO_MODE)
    {
        pitch->Angle_buf = jianshu_flag ? (float)(LIMIT(YK.shubiao.y, -400, 400) / 1000.0) : (float)(YK.yaogan.ch3 / 3300.0);
        pitch->Target_Angle += pitch->Angle_buf;
        if (DM_PITCH.mang < PITCH_HIGH + 0.01 && pitch->Angle_buf > 0)
            pitch->Target_Angle = pitch->Last_Angle;
        if (DM_PITCH.mang > PITCH_LOW - 0.01 && pitch->Angle_buf < 0)
            pitch->Target_Angle = pitch->Last_Angle;
        pitch->Last_Angle = pitch->Target_Angle;
        pitch->Target_Angle = LIMIT(pitch->Target_Angle, -15.4f, 32.8f);
        Pitch.ref = pitch->Target_Angle;
        SMC_Pitch.SMC_Tick(pitch->Target_Angle, pitch->Angle_buf, 0, GIMBAL_088.realAngle.pitch, GIMBAL_088.Anglespeed.Deal_pitch);
        pitch->Pitch_Out = gm6020to_torq_pitch(SMC_Pitch.u);
        pitch->Pitch_Out = LIMIT(pitch->Pitch_Out, -4.0f, 4.0f);
    }
    else if (PITCH_Mode == AUTO_MODE)
    {
        SMC_Pitch_Zm.SMC_Tick(pitch->Target_Angle,Zm_Pitch_Vel , Zm_Pitch_Acc, GIMBAL_088.realAngle.pitch, GIMBAL_088.Anglespeed.Deal_pitch);
        pitch->Pitch_Out = gm6020to_torq_pitch(SMC_Pitch_Zm.u);
        pitch->Pitch_Out += GetPitchUpGravityComp(pitch->Target_Angle, GIMBAL_088.realAngle.pitch);
        pitch->Pitch_Out = LIMIT(pitch->Pitch_Out, -4.0f, 4.0f);
    }
    else
    {
        pitch->Pitch_Out = 0;
        pitch->Target_Angle = GIMBAL_088.realAngle.pitch;
    }
    return pitch->Pitch_Out;
}
void PITCH::set_Pitch_Target(f Target_Angle)
{
    pitch->Target_Angle = Target_Angle;
}
void PITCH::set_SMCref(f Target_Angle)
{
    Pitch.ref = Target_Angle;
    Pitch_Zm.ref = Target_Angle;
}