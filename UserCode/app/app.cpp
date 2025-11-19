#include "app.h"

#include "can.h"
#include "tim.h"

#include "interfaces/motor_if.h"

#ifdef __cplusplus
extern "C" {

UnitreeMotor* g_motor = NULL;

DJI_t dji;

Motor_PosCtrl_t pos_dji;

/**
 * 定时器回调函数，用于定时进行 PID 计算和 CAN 指令发送
 * @param htim
 */
void TIM_Callback(TIM_HandleTypeDef* htim)
{
    Motor_PosCtrlUpdate(&pos_dji); // 进行 PID 计算

    // TransmitData();
    // 发送控制信号
    DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_1_4);
    // DJI_SendSetIqCommand(&hcan1, IQ_CMD_GROUP_5_8);
}

void DJI_Control_Init()
{

    DJI_CAN_FilterInit(&hcan1, 0);

    HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, DJI_CAN_Fifo0ReceiveCallback);
    // HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID, DJI_CAN_Fifo1ReceiveCallback);

    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    // CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);

    DJI_Init(&dji, (DJI_Config_t){
                       .auto_zero  = false,      //< 是否在启动时自动清零角度
                       .motor_type = M3508_C620, //< 电机类型
                       .hcan       = &hcan1,     //< 电机挂载在的 CAN 句柄
                       .id1        = 1,          //< 电调 ID (1~8)
                   });

    Motor_PosCtrl_Init(&pos_dji, //
                       (Motor_PosCtrlConfig_t){
                           .motor_type = MOTOR_TYPE_DJI, //< 电机类型
                           .motor      = &dji,           //< 控制的电机
                           .velocity_pid =
                               (MotorPID_Config_t){
                                   .Kp             = 30.0f,  //<
                                   .Ki             = 0.001f, //<
                                   .Kd             = 5.00f,  //<
                                   .abs_output_max = 8000.0f //< DJI_M3508_C620_IQ_MAX //< 限幅为电流控制最大值
                               },
                           .position_pid =
                               (MotorPID_Config_t){
                                   .Kp             = 4.0f,   //<
                                   .Ki             = 0.01f,  //<
                                   .Kd             = 0.00f,  //<
                                   .abs_output_max = 2000.0f //< 限速，这是外环对内环的输出限幅
                               },
                           .pos_vel_freq_ratio = 1, //< 内外环频率比（外环的频率可能需要比内环低）
                       });

    HAL_TIM_RegisterCallback(&htim6, HAL_TIM_PERIOD_ELAPSED_CB_ID, TIM_Callback);
    HAL_TIM_Base_Start_IT(&htim6);
}

float target_angle = 0.0f;

// 使用时需要修改为对应的电机ID，串口句柄，kp，kd参数，与RE，DE引脚设置（在Unitree_user.h）

// 前馈力矩计算
// y = 0.002 * x^3 - 0.0496 * x^2 + 0.0611 * x + 1.8155
// y = ((0.002 * x - 0.0496) * x + 0.0611) * x + 1.8155
float tau_Cal(float angle)
{
    float tau = 0.002f;
    tau       = tau * angle - 0.0496f;
    tau       = tau * angle + 0.0611f;
    tau       = tau * angle + 1.8155f;
    return tau;
}

// 电机初始化任务

void Init(void* argument)
{
    g_motor = Unitree_Create_Motor();
    if (Unitree_init(g_motor, &UART_UNITREE_HANDLER, ID) != HAL_OK)
    {
        Error_Handler();
    }

    DJI_Control_Init(); // 初始化大疆电机
    osThreadExit();
}

void MotorCtrl(void* argument)
{
    for (;;)
    {
        Unitree_UART_tranANDrev(g_motor,
                                ID,
                                1,                     // mode
                                tau_Cal(target_angle), // tau
                                1.0f,                  // W
                                0.0f,                  // Pos
                                0.0f,                  // K_P
                                0.1f                   // K_W
        );
        osDelayUntil(10);
    }
}
}


#endif // __cplusplus
