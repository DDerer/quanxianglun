#include "stm32f10x.h"                  // Device header
#include <math.h>
#include "Delay.h"
#define GEAR_RATIO 21.3f //电机减速比
#define WHEEL_DIAMETER 0.022f //直径
#define ENCODER_LINES 11 //编码器线数
#define CONTROL_HZ   100 //TIM3的调控频率
#define MAX_LINEAR_SPEED 0.3f  //最大线速度
float motor_speed[3];
static const float wheel_circumference = 3.1415926f * WHEEL_DIAMETER; //车轮周长
//PID结构体定义
typedef struct
{
	float Kp;           // 比例增益
    float Ki;           // 积分增益
    float Kd;           // 微分增益
    float Integral;     // 积分项
    float Prev_error;   // 前一次误差
    float Output;       // PID输出
	float Setpoint;		// 目标值
}PID_Controller;

static PID_Controller motor_pid[3];
//PID结构体初始化函数
void PID_Init(PID_Controller * pid,float kp,float ki,float kd,float setpoint)
{
	pid->Kp = kp;
	pid->Kd = kd;
	pid->Ki = ki;
	pid->Setpoint = setpoint;
	pid->Integral = 0.0f;
	pid->Output = 0.0f;
	pid->Prev_error = 0.0f;
}
float Motor_Speed_Get(uint8_t motor_id)
{
	int16_t encoder_diff = 0;
	static int16_t last_count[3] = {0};
	//获取编码器差值
	switch(motor_id)
	{
		case 0:
			encoder_diff = TIM_GetCounter(TIM1) - last_count[0];
			last_count[0] = TIM_GetCounter(TIM1);
			break;
		case 1:
			encoder_diff = TIM_GetCounter(TIM4) - last_count[1];
			last_count[1] = TIM_GetCounter(TIM4);
			break;
		case 2:
			encoder_diff = TIM_GetCounter(TIM8) - last_count[2];
			last_count[2] = TIM_GetCounter(TIM8);
			break;		
	}
	float dist = (encoder_diff / (ENCODER_LINES * 4.0f * GEAR_RATIO)) * wheel_circumference;
	float v =  dist * CONTROL_HZ;//返回线速度
	return v / (MAX_LINEAR_SPEED) * 100.0f;
}
void PID_Update(PID_Controller * pid,float actual)
{
	float error = pid->Setpoint - actual;
	pid->Integral += error;
	
	if(pid->Integral > 100.0f) pid->Integral = 100.0f;
	else if(pid->Integral < -100.0f)  pid->Integral = -100.0f;
	//计算输出值
	pid->Output = pid->Kp * error + pid->Ki * pid->Integral + pid->Kd * (error - pid->Prev_error);
	if(pid->Output>100.0f) pid->Output = 100.0f;
	else if(pid->Output<-100.0f) pid->Output = -100.0f;
	pid->Prev_error = error;//更新误差值
}
//电机控制
void Motor_Control(uint8_t idx)
{
	float speed = Motor_Speed_Get(idx);
	PID_Update(&motor_pid[idx],speed);
	
	//设置PWM输出
	uint16_t pwm = (uint16_t)fabsf(motor_pid[idx].Output);
	
	//设置方向
	if(motor_pid[idx].Output>=0)
	{
		switch(idx)
		{
			case 0: GPIO_SetBits(GPIOA, GPIO_Pin_3); GPIO_ResetBits(GPIOA, GPIO_Pin_4); break;
			case 1: GPIO_SetBits(GPIOA, GPIO_Pin_6); GPIO_ResetBits(GPIOA, GPIO_Pin_5); break;
			case 2: GPIO_SetBits(GPIOB, GPIO_Pin_0); GPIO_ResetBits(GPIOA, GPIO_Pin_7); break;
		}
	}
	else
	{
		switch(idx)
		{
			case 0: GPIO_SetBits(GPIOA, GPIO_Pin_4); GPIO_ResetBits(GPIOA, GPIO_Pin_3); break;
			case 1: GPIO_SetBits(GPIOA, GPIO_Pin_5); GPIO_ResetBits(GPIOA, GPIO_Pin_6); break;
			case 2: GPIO_SetBits(GPIOA, GPIO_Pin_7); GPIO_ResetBits(GPIOB, GPIO_Pin_0); break;
		}
	}
	//设置PWM
	switch(idx)
	{
		case 0: TIM_SetCompare1(TIM2,pwm);break;
		case 1: TIM_SetCompare2(TIM2,pwm);break;
		case 2: TIM_SetCompare3(TIM2,pwm);break;
	}
}
void Motor_Speed_Update(float vx,float vy,float w)
{
	//运动学逆解求每个轮子的速度
	float v1 = vy + w;
	float v2 = - (vx * (sqrt(3.0f) / 2.0f)) - vy / 2.0f + w;
	float v3 = vx * (sqrt(3.0f) / 2.0f) - vy / 2.0f + w;
	
	//设置目标值
	motor_pid[0].Setpoint = v1;
	motor_pid[1].Setpoint = v2;
	motor_pid[2].Setpoint = v3;
}

//GPIO初始化
void GPIO_Config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	GPIO_InitTypeDef io;
	
	/* PWM: PA0, PA1, PA2 */
    io.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
    io.GPIO_Mode = GPIO_Mode_AF_PP; io.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &io);
	
	/* 方向引脚 */
    io.GPIO_Mode = GPIO_Mode_Out_PP;
    io.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;  // Motor1 AIN1/AIN2 PA3和PA4
    GPIO_Init(GPIOA, &io);
    io.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;  // Motor2 AIN1/AIN2 PA5和PA6
    GPIO_Init(GPIOA, &io);
    io.GPIO_Pin = GPIO_Pin_7;             // Motor3 AIN1  PA7和PB0
    GPIO_Init(GPIOA, &io);
    io.GPIO_Pin = GPIO_Pin_0;             // Motor3 AIN2
    GPIO_Init(GPIOB, &io);
	
	/* 编码器输入 */
	io.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	io.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;  // TIM1_CH1/2  
    GPIO_Init(GPIOA, &io);
    io.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;  // TIM4_CH1/2
    GPIO_Init(GPIOB, &io);
    io.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;  // TIM8_CH1/2
    GPIO_Init(GPIOC, &io);
}
/* TIM2 3路 PWM输出 */
void TIM2_PWM_Config(void)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;
		GPIO_InitTypeDef io;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	/* PWM: PA0, PA1, PA2 */
    io.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
    io.GPIO_Mode = GPIO_Mode_AF_PP; io.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &io);
	
    tb.TIM_Period = 100-1; tb.TIM_Prescaler = 72-1;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
	tb.TIM_RepetitionCounter = 0 ; tb.TIM_ClockDivision  = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &tb);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
	
    oc.TIM_Pulse = 0;
    // CH1
	TIM_OC1Init(TIM2, &oc); TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    // CH2
    TIM_OC2Init(TIM2, &oc); TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    // CH3
    TIM_OC3Init(TIM2, &oc); TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

}
/* 三路定时器编码器模式 */
void TIM_Encoder_Config(void)
{
    TIM_TimeBaseInitTypeDef tb;
	TIM_ICInitTypeDef ic;
    // TIM1 for Motor1
    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Period = 0xFFFF;
    TIM_TimeBaseInit(TIM1, &tb);
    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&ic);
	ic.TIM_ICFilter = 10;
	TIM_ICInit(TIM1, &ic);  TIM_ClearFlag(TIM1, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_SetCounter(TIM1, 0); TIM_Cmd(TIM1, ENABLE);

    // TIM4 for Motor2
    TIM_TimeBaseInit(TIM4, &tb);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);	TIM_ICStructInit(&ic);
	ic.TIM_ICFilter = 10;
	TIM_ICInit(TIM4, &ic);  TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_SetCounter(TIM4, 0); TIM_Cmd(TIM4, ENABLE);

	//TIM8 for Motor3
	TIM_TimeBaseInit(TIM8,&tb);
	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&ic);
	ic.TIM_ICFilter = 10;
	TIM_ICInit(TIM8, &ic);  TIM_ClearFlag(TIM8, TIM_FLAG_Update);//清除TIM的更新标志位
	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
	TIM_SetCounter(TIM8, 0); TIM_Cmd(TIM8, ENABLE);

}

//TIM6用于中断，用于电机控制的调控周期
void TIM3_Config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_InternalClockConfig(TIM3);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseIntiStructure;
	TIM_TimeBaseIntiStructure.TIM_ClockDivision  = TIM_CKD_DIV1;
	TIM_TimeBaseIntiStructure.TIM_CounterMode  = TIM_CounterMode_Up;
	TIM_TimeBaseIntiStructure.TIM_Period =1000-1 ;
	TIM_TimeBaseIntiStructure.TIM_Prescaler = 720-1;
	TIM_TimeBaseIntiStructure.TIM_RepetitionCounter = 0 ;  
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseIntiStructure);
	
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructrue;
	NVIC_InitStructrue.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructrue.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructrue.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructrue.NVIC_IRQChannelSubPriority = 1 ;
	NVIC_Init(&NVIC_InitStructrue);
	
	TIM_Cmd(TIM3,ENABLE);		
}
//三角形运动
void Move_Triangle() {
    for (int i = 0; i < 10; i++) {
        Motor_Speed_Update(25.0f * sqrt(3.0f), -25.0f, 0);
        Delay_ms(100);
    }
    for (int i = 0; i < 10; i++) {
        Motor_Speed_Update(-25.0f * sqrt(3.0f), -25.0f, 0);
        Delay_ms(100);
    }
    for (int i = 0; i < 10; i++) {
        Motor_Speed_Update(0, 50.0f, 0);
        Delay_ms(100);
    } 
    // 停止并重置积分项
    Motor_Speed_Update(0, 0, 0);
    motor_pid[0].Integral = 0;
    motor_pid[1].Integral = 0;
    motor_pid[2].Integral = 0;
}
//正方形运动
void Move_Square() {
    for (int i = 0; i < 10; i++) {
        Motor_Speed_Update(50.0f, 0, 0);
        Delay_ms(100);
    }
    for (int i = 0; i < 10; i++) {
        Motor_Speed_Update(0, -50.0f, 0);
        Delay_ms(100);
    }
    for (int i = 0; i < 10; i++) {
        Motor_Speed_Update(-50.0f, 0, 0);
        Delay_ms(100);
    }
    for (int i = 0; i < 10; i++) {
        Motor_Speed_Update(0, 50.0f, 0);
        Delay_ms(100);
    }
    Motor_Speed_Update(0, 0, 0);
    motor_pid[0].Integral = 0;
    motor_pid[1].Integral = 0;
    motor_pid[2].Integral = 0;
}
//圆形运动
void Move_Round()
{
	float r = 0.5f;       
    float l_speed = 0.1f;  
    float time_per_round = (2 * 3.1415926f * r) / l_speed; // 一圈所需时间（毫秒）
    uint32_t steps = 100;       
    uint32_t step_time = time_per_round / steps; 
    // 将实际速度转换为PWM百分比（基于MAX_LINEAR_SPEED）
    float speed_pwm = (l_speed / MAX_LINEAR_SPEED) * 100.0f;
    for(uint32_t i = 0; i < steps; i++) {
        float angle = 2 * 3.1415926f * i / steps; // 当前角度
        // 计算x和y方向的PWM分量
        float vx_pwm = speed_pwm * sinf(angle); // 负号是因为坐标系可能需要反转
        float vy_pwm = -speed_pwm * cosf(angle);
        Motor_Speed_Update(vx_pwm, vy_pwm, 0); // w=0保持朝向不变
        Delay_ms(step_time);
    }
}
int main()
{
	GPIO_Config();
    TIM2_PWM_Config();
    TIM_Encoder_Config();
    TIM3_Config();
	// 在main()初始化部分添加：
	PID_Init(&motor_pid[0], 1.0f, 0.005f, 0.5f, 0);  // 电机1
	PID_Init(&motor_pid[1], 1.0f, 0.005f, 0.5f, 0);  // 电机2  
	PID_Init(&motor_pid[2], 1.0f, 0.005f, 0.5f, 0);  // 电机3
	while(1)
	{
//		Move_Triangle();
		Move_Square();
//		Move_Round();
	}
}
void TIM3_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)
	{
		Motor_Control(0);
		Motor_Control(1);
		Motor_Control(2);
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}
}