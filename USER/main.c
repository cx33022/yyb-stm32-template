#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "board.h"
#include "Emm_V5.h"
#include "uart6_ckp.h"
#include "Serial.h"
#include "yyb_move.h"
#include "imu.h"
#include "PWM.h"
#include "LED.h"
#include "OLED.h"
#include "swith.h"
#include "pid.h"
#include "usart5.h"
#include "wit_c_sdk.h"
#include "exti.h"
#include "app_config.h"



#if CHASSIS_CAN_MOTOR_TEST
/* 在 Keil Watch 中观察：循环次数、是否收到回帧、CAN 错误与发送状态。 */
volatile uint32_t can_test_internal_loopback = CAN_INTERNAL_LOOPBACK_TEST;
/* 仅在内部回环模式有效：0 等待结果；1 收发通过；2 连续五轮仍未通过。 */
volatile uint32_t can_test_loopback_result = 0;
volatile uint32_t can_test_stage = 0;
volatile uint32_t can_test_loop_count = 0;
volatile uint32_t can_test_rx_count = 0;
volatile uint32_t can_test_tx_no_mailbox_count = 0;
volatile uint32_t can_test_last_rx_id = 0;
volatile uint32_t can_test_error_reg = 0;
volatile uint32_t can_test_tx_error_count = 0;
volatile uint32_t can_test_bus_off = 0;
volatile uint32_t can_test_tx_reg = 0;

/*
 * 轮位与原 yyb_move.c 保持一致：前左地址 2、前右地址 1、
 * 后左地址 3、后右地址 4。左右安装方向相反，所以正向滚动的
 * 电机方向分别为 0、1、0、1。速度只用 30 RPM，先架空车轮测试。
 */
static void ChassisCanMotorTest_Enable(void)
{
    Emm_V5_En_Control(2, true, false);
    delay_ms(10);
    Emm_V5_En_Control(1, true, false);
    delay_ms(10);
    Emm_V5_En_Control(3, true, false);
    delay_ms(10);
    Emm_V5_En_Control(4, true, false);
    delay_ms(10);
}

static void ChassisCanMotorTest_SendSpeed(void)
{
    /* false 表示立即执行，不等待同步启动命令。 */
    Emm_V5_Vel_Control(2, 0, 30, 30, false);
    delay_ms(10);
    Emm_V5_Vel_Control(1, 1, 30, 30, false);
    delay_ms(10);
    Emm_V5_Vel_Control(3, 0, 30, 30, false);
    delay_ms(10);
    Emm_V5_Vel_Control(4, 1, 30, 30, false);
}
#endif

//x减物块右移（从外侧往内看）//y+物块往地图内侧走
#define  x_center_red       955
#define  y_center_red		519

#define  x_center_green		955
#define  y_center_green		519

#define  x_center_blue		955
#define  y_center_blue		519

char tjcstr[100];
int acc[4];
float direct_x = 0;
float direct_y = 0;
float distance_wukuai = 150;
float x_origin = 0;
float y_origin = 0;
int task[6]; //存储颜色信息
int ypcode[6]; //存储颜色信息
float angle = 0;

void transfer_position_wukuai(uint8_t task_choose,float distance,uint8_t color);
void transfer_position_sehuan(uint8_t fenbian_choose,uint8_t task_num);

void fangzhi(uint8_t task); //色环位置放置物块
void fangzhi_maduo(uint8_t task);  //码垛位置放置物块
void na(uint8_t task);   //色环位置拿物块

void yuantai_na(uint8_t task);//圆盘位置拿物块
void yuantai_na1(uint8_t task);//圆盘位置拿物块，识别就位
void yuantai_na2(uint8_t task);//圆盘位置拿物块，识别到物块静止，此时抓取

void wait_send_shumeipai(uint8_t message,uint16_t time);
void clear_ckp(void);
void jixie_reset(void);
char message[] = "Hello, BT - 04 from STM32F407!\r\n";
#if CHASSIS_POWER_ON_TEST && !CHASSIS_CAN_MOTOR_TEST
/*
 * 复现原 main.c 两轮任务中的固定底盘行驶段。
 * car_move1/2 继续使用原位置模式：CAN 地址、方向、同步启动均未改动。
 * car_move1 实际只使用 y，car_move2 实际只使用 x；保留原调用参数便于逐行对照。
 * PID_move 继续使用 HWT101 航向角。视觉修正 direct_x/direct_y 需要树莓派数据，
 * 因此本测试只能走固定路线；标注“视觉对位”的位置尚不能精确对准工位。
 */
static void ChassisRoute_TurnTo(float yaw_deg)
{
    mypid.integral = 0;
    PID_move(0, 0, 0, 1000, yaw_deg, 0); /* 先转到目标航向 */
    PID_move(0, 0, 0, 1000, yaw_deg, 3); /* 再按原程序保持并微调航向 */
}

static void ChassisOriginalFixedRoute_Run(void)
{
    /* 第一轮：起点 -> 扫码区 -> 原料区。 */
    car_move2(150, 0, 150, 150);
	
    car_move1(0, -530, 150, 200);
    car_move1(0, -910, 150, 200);
    car_move2(-30, 0, 100, 100);
    /* 原程序在此等待树莓派识别原料，并用 direct_x/direct_y 视觉对位。 */

    /* 原料区 -> 粗加工区。 */
    PID_move(-30, 0, 0, 400, 0, 1);
    delay_ms1(200);
    car_move1(0, 400, 150, 200);
    ChassisRoute_TurnTo(90.0f);
    car_move1(0, -1700, 150, 250);
    ChassisRoute_TurnTo(-180.0f);
    /* 原程序在此对色环进行粗、细两次视觉对位。 */
    PID_move(0, 0, 0, 1000, -180.0f, 4);

    /* 粗加工区 -> 色环区。 */
    PID_move(-30, 0, 0, 300, -180, 1);
    PID_move(0, 0, 0, 500, -180, 3);
    car_move1(140, 140 + 680, 180, 180);
    delay_ms1(100);
    ChassisRoute_TurnTo(90.0f);
    car_move1(140, 140 + 620, 180, 150);
    /* 原程序在此进行色环视觉对位。 */
    PID_move(0, 0, 0, 1000, 90.0f, 4);

    /* 色环区 -> 圆盘区。 */
    PID_move(-30, 0, 0, 300, 90, 1);
    delay_ms1(100);
    car_move1(140, 220 + 650, 180, 180);
    ChassisRoute_TurnTo(0.0f);
    car_move1(140, 370 + 20, 180, 180);

    /* 第二轮：从第一轮终点继续，不重新回到上电起点。 */
    PID_move(30, 0, 0, 300, 0, 1);
    delay_ms1(100);
    /* 原程序在此等待树莓派视觉对位和第二轮取料。 */

    /* 第二轮原料区 -> 粗加工区。 */
    PID_move(-30, 0, 0, 400, 0, 1);
    delay_ms1(200);
    car_move1(0, 400, 150, 200);
    ChassisRoute_TurnTo(90.0f);
    car_move1(0, -1700, 150, 250);
    ChassisRoute_TurnTo(-180.0f);
    /* 原程序在此进行色环视觉对位。 */
    PID_move(0, 0, 0, 1000, -180.0f, 4);

    /* 第二轮粗加工区 -> 码垛区。 */
    PID_move(-30, 0, 0, 300, -180, 1);
    PID_move(0, 0, 0, 500, -180, 3);
    car_move1(140, 140 + 680, 180, 180);
    delay_ms1(100);
    ChassisRoute_TurnTo(90.0f);
    car_move1(140, 140 + 620, 180, 150);
    /* 原程序在此进行码垛区视觉对位和取放。 */

    /* 码垛区 -> 终点。 */
    PID_move(-30, 0, 0, 300, 90, 1);
    car_move1(140, 220 + 650, 180, 180);
    ChassisRoute_TurnTo(0.0f);
    car_move1(140, 2000 - 150, 180, 180);
    car_move2(-130, 0, 150, 150);
}
#endif

#if PA2_PA3_SERVO_TEST
/* Current assembly: PA2 drives the claw, PA3 drives the lift; PA1 stays idle. */
#define PA2_TEST_HOME_US 648U
#define PA2_TEST_TARGET_US 688U
#define PA3_TEST_HOME_US 838U
#define PA3_TEST_TARGET_US 878U
volatile uint32_t pa3_gimbal_test_stage = 0U;
volatile uint32_t pa3_gimbal_test_pulse_us = PA3_TEST_HOME_US;
volatile uint32_t pa3_gimbal_test_cycle_count = 0U;
volatile uint32_t pa2_servo_test_stage = 0U;
volatile uint32_t pa2_servo_test_pulse_us = PA2_TEST_HOME_US;
volatile uint32_t pa2_servo_test_cycle_count = 0U;

static void Pa2Pa3ServoTest_Init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef oc;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

    /* Same 1 MHz timer tick and 50 Hz PWM as the existing PWM_Init(). */
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 84U - 1U;
    timer.TIM_Period = 20000U - 1U;
    TIM_TimeBaseInit(TIM2, &timer);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_Pulse = PA2_TEST_HOME_US;  /* Existing PA2 startup pulse. */
    TIM_OC3Init(TIM2, &oc);
    oc.TIM_Pulse = PA3_TEST_HOME_US;  /* Existing PA3 startup pulse. */
    TIM_OC4Init(TIM2, &oc);
    TIM_Cmd(TIM2, ENABLE);
}

static void Pa3GimbalTest_MoveTo(uint16_t target_us)
{
    pa3_gimbal_test_pulse_us = target_us;
    TIM_SetCompare4(TIM2, target_us);
}

#if !PA3_FAST_SWEEP_TEST
static void Pa2ServoTest_MoveTo(uint16_t target_us)
{
    while (pa2_servo_test_pulse_us != target_us) {
        if (pa2_servo_test_pulse_us < target_us) {
            pa2_servo_test_pulse_us += 4U;
        } else {
            pa2_servo_test_pulse_us -= 4U;
        }
        TIM_SetCompare3(TIM2, (uint16_t)pa2_servo_test_pulse_us);
        delay_ms(40);
    }
}
#endif
#endif
int main(void)
{	
/*
	所有的中断：
		1.OLED每隔500ms更新角度数据（最好不启用，里面耗时很长）（会影响delay函数的调用正确与否）
		2.delayms1函数在启用时最好关闭所有中断
		3.PID每间隔50ms进行一次PID计算，花25ms给所有电机发送指令（耗时中等）
		4.IMU每次接受信息触发一次中断
		5.树莓派给stm32发送信息时触发中断
		6.步进电机到位时发送信息中断
		7.舵机控制每隔20ms定时控制角度
*/
	delay_init(168);
#if APP_SAFE_IDLE
    while (1) { delay_ms(1000); }
#elif PA2_PA3_SERVO_TEST
    Pa2Pa3ServoTest_Init();
    pa3_gimbal_test_stage = 1U;
    pa2_servo_test_stage = 1U;
    delay_ms(1500);
#if PA3_FAST_SWEEP_TEST
    while (1) {
        Pa3GimbalTest_MoveTo(PA3_TEST_TARGET_US);
        pa3_gimbal_test_stage = 2U;
        delay_ms(700);
        Pa3GimbalTest_MoveTo(PA3_TEST_HOME_US);
        pa3_gimbal_test_stage = 3U;
        ++pa3_gimbal_test_cycle_count;
        delay_ms(700);
    }
#else
    while (1) {
        Pa3GimbalTest_MoveTo(PA3_TEST_TARGET_US);
        pa3_gimbal_test_stage = 2U;
        delay_ms(1000);
        Pa3GimbalTest_MoveTo(PA3_TEST_HOME_US);
        pa3_gimbal_test_stage = 3U;
        ++pa3_gimbal_test_cycle_count;
        delay_ms(500);
        Pa2ServoTest_MoveTo(PA2_TEST_TARGET_US);
        pa2_servo_test_stage = 2U;
        delay_ms(1000);
        Pa2ServoTest_MoveTo(PA2_TEST_HOME_US);
        pa2_servo_test_stage = 3U;
        ++pa2_servo_test_cycle_count;
        delay_ms(500);
    }
#endif
#elif CHASSIS_CAN_MOTOR_TEST
    /*
     * 这一路测试跳过 HWT101、PID 和定位路线，避免角度未就绪时
     * 卡在原来的等待循环。速度模式发令后电机会持续转动。
     */
    can_test_stage = 1;  /* 已进入四轮 CAN 测试入口。 */
    board_init();
    can_test_stage = 2;  /* CAN 初始化完成。 */
    delay_ms(500);
    ChassisCanMotorTest_Enable();
    can_test_stage = 3;  /* 四个使能命令已提交。 */
    delay_ms(100);
    while (1) {
        ChassisCanMotorTest_SendSpeed();
        can_test_stage = 4;  /* 四个速度命令已提交，开始持续循环。 */
        ++can_test_loop_count;
        /* 中断里的计数是实际收到的 CAN 帧数；最后帧 ID 用于核对电机地址。 */
        can_test_rx_count = can_rx_frame_count;
        can_test_tx_no_mailbox_count = can_tx_no_mailbox_count;
        if (can.rxFrameFlag) {
            can.rxFrameFlag = false;
            can_test_last_rx_id = can.CAN_RxMsg.ExtId;
        }
        /* ESR 可看发送/接收错误计数及 Bus-Off；TSR 可看发送邮箱状态。 */
        can_test_error_reg = CAN1->ESR;
        can_test_tx_error_count = (can_test_error_reg >> 16) & 0xFFU;
        can_test_bus_off = (can_test_error_reg & CAN_ESR_BOFF) ? 1U : 0U;
        can_test_tx_reg = CAN1->TSR;
#if CAN_INTERNAL_LOOPBACK_TEST
        /*
         * 回环帧不经过收发器和外部电机。实际收到回环帧且没有发送错误，
         * 才能说明 MCU 内部 CAN 控制器、过滤器和接收中断这段链路正常。
         */
        if (can_test_rx_count > 0U &&
            can_test_tx_no_mailbox_count == 0U &&
            can_test_tx_error_count == 0U &&
            can_test_bus_off == 0U) {
            can_test_loopback_result = 1U;
        } else if (can_test_loop_count >= 5U) {
            can_test_loopback_result = 2U;
        }
#endif
        delay_ms(500);
    }
#elif CHASSIS_POWER_ON_TEST
    board_init();        /* 原 CAN1 500 kbps 接口，驱动四个步进电机 */
    Timer6_Init();       /* car_move1/2 的位置命令等待时间依赖 TIM6 */
    delay_ms1(1000);     /* 等电机驱动器上电 */
    imu_init();          /* HWT101 接在 USART2，提供车身航向角 */
    PIDInit();           /* TIM4 每 50 ms 进行一次航向 PID 计算 */
    delay_ms1(500);      /* 等待 HWT101 至少送来一次航向角 */
    if (!imu_angle_ready()) {
        while (1) { delay_ms1(1000); } /* 没收到航向角就不自动行驶 */
    }
    dianji_move_flag = 1;
    WHT101_ANGLEZCali(); /* 以上电摆放方向作为 0 度 */
    delay_ms1(300);
    ChassisOriginalFixedRoute_Run();
    while (1) { delay_ms1(1000); } /* 两轮固定路线只跑一次 */
#else
	board_init();  //配置can总线协议
	LED_Init();  //配置LED指示灯
	OLED_Init();   //暂时不启用OLED //OLED中断和TIM5绑定在一起（两个必须同时用）
	Timer6_Init(); //delayms1的时钟
	PWM_Init();	  //使用定时器二，开启中断，实时控制舵机角度
	uart5_init(9600);
	uart6_init(9600);//串口屏用的UART6
	Serial_Init();	//通讯串口1初始化，用UART1
	Swith_Init();   //开关初始化
	imu_init();     //使用UART2
	PIDInit();		//用定时器实现定时中断//周期为50ms//该中断最好不要被别的中断打断
	Key_EXTI_Init();//按键中断//向树莓派发送重启命令
    clear_ckp();    //清空串口屏
	dianji_move_flag = 1;//使能电机的运动
	TIM_SetCompare1(TIM3,0);  //关闭补光灯
//**************************测试代码***************************//
	while(swith_out() == 1){		
		LED_Toggle2();
		delay_ms1(50);
	}//树莓派已经准备好，按下开关即可启动
	move_all(-120.0f,1000,200   ,0,2000,200   ,278.0f,13);
	set_zhuashou_kai();
	jixie_reset();
	while(1){}
//*************************第一轮*************************//
	while(1){
		LED_Toggle2();
		delay_ms1(200);
		if((Serial_RxPacket[0]==0x39) && (Serial_RxPacket[1]==0x38)){
			break;
	    }
	}
//**************************指示灯闪烁，可以开始***************************//
	while(swith_out() == 1){		
		LED_Toggle2();
		delay_ms1(50);
	}//树莓派已经准备好，按下开关即可启动
	mypid.integral = 0;
	WHT101_ANGLEZCali();//101z轴置零
	Serial_GetRxFlag();Serial_SendByte(0x31);
//***************************第一部分，识别二维码并显示，大致移动到原料区*********************//	
	//扫二维码
	car_move2(150,0,150,150);
	car_move1(0,-530,150,200);
    //读取树莓派信息	
	while(1){
		if (Serial_GetRxFlag() == 1){   //收到数据包
			   sprintf(tjcstr, "t0.txt=\"%d%d%d+%d%d%d \"",
										Serial_RxPacket[0]-0x30,Serial_RxPacket[1]-0x30,Serial_RxPacket[2]-0x30,
										Serial_RxPacket[4]-0x30,Serial_RxPacket[5]-0x30,Serial_RxPacket[6]-0x30);
				HMISends(tjcstr);
	            HMISendb(0xff);//在显示屏上面显示任务
				break;
			  }
	}	
    task[0]= Serial_RxPacket[0]-0x30;task[1] = Serial_RxPacket[1]-0x30;task[2] = Serial_RxPacket[2]-0x30;
	task[3]= Serial_RxPacket[4]-0x30;task[4] = Serial_RxPacket[5]-0x30;task[5] = Serial_RxPacket[6]-0x30;
	set_wukuaipingtai_weizhi(0x01);
//****************************调整(亮度等参数)****************************//
	TIM_SetCompare1(TIM3,300);
//********************第二部分，识别物块颜色，按照指定的顺序搬运**************************//
	//复位到识别的位置
	move_all1(-120.0f,1000,200   ,0,2000,200   ,278,13);
	set_zhuashou_kai();
	
	car_move1(0,-1000+90,150,200);
	move_all1(40.0f,1000,200   ,0,2000,200   ,278,13);
	//提前发送命令2
	Serial_GetRxFlag();Serial_SendByte(0x32);
	car_move2(-30,0,100,100);	
	while(1){
		while(1){
			if(Serial_GetRxFlag() == 1){
				break;  //是wait发送的分解
			}
		}
		//0x01代表选择的是转盘上的物块，70.0f是物块的高度，0x00代表不选择颜色
		transfer_position_wukuai(0x01,70.0f,0x00); 
		car_move_distance(direct_x,direct_y,100,80);
		if(Serial_RxPacket[8] != 0x34){
			break;
		}else{
			wait_send_shumeipai(0x32,15000);
		}
	}
	move_all(60.0f,1000,200   ,0,2000,200   ,278,13);
	wait_send_shumeipai(0x33,20000);  //识别圆盘上要抓物块的顺序
	ypcode[0]= Serial_RxPacket[0]-0x30;ypcode[1] = Serial_RxPacket[1]-0x30;ypcode[2] = Serial_RxPacket[2]-0x30;
	//1.圆台拿走第一个物块
	set_wukuaipingtai_weizhi(0x01);
	yuantai_na(ypcode[0]);
	
	//2.圆台拿走第二个物块
	set_wukuaipingtai_weizhi(0x02);
	yuantai_na1(ypcode[1]);   //达到指定的识别的位置
	//判断物块是否静止
	wait_send_shumeipai(0x35,30000);
	while(1){
		if(Serial_RxPacket[0] == 0x39 && Serial_RxPacket[1] == 0x38){
			break;
		}
	}Serial_RxPacket[0] = 0x30;Serial_RxPacket[1] = 0x30;
	yuantai_na2(ypcode[1]);  //一旦识别到物块静止，抓物块到物块平台上
		
	//3.圆台拿走第三个物块
	set_wukuaipingtai_weizhi(0x03);
	yuantai_na1(ypcode[2]);	 //达到指定的识别的位置
	//判断物块是否静止
	wait_send_shumeipai(0x36,30000);
	while(1){
		if(Serial_RxPacket[0] == 0x39 && Serial_RxPacket[1] == 0x38){
			break;
		}
	}Serial_RxPacket[0] = 0x30;Serial_RxPacket[1] = 0x30;	
	yuantai_na2(ypcode[2]);	 //一旦识别到物块静止，抓物块到物块平台上

	TIM_SetCompare1(TIM3,0); //关闭补光灯
	//复位到识别的位置
	set_wukuaipingtai_weizhi(0x01);
	move_all1(-120.0f,1000,200   ,0,1000,200   ,278,13);
	set_zhuashou_kai();
//***************************到下一个区域**************************//	
	//到粗加工区
	PID_move(-30,0,0,400,0,1);//回倒车
	delay_ms1(200);
	car_move1(0,400,150,200);
	
	mypid.integral = 0;PID_move(0,0,0,1000,90,0);//在原地旋转
	PID_move(0,0,0,1000,90,3);
	
	car_move1(0,-1700,150,250);//快速通过
	
	mypid.integral = 0;PID_move(0,0,0,1000,-180,0);//在粗加工区原地旋转
	PID_move(0,0,0,1000,-180,3);
//****************************调整(亮度等参数)****************************//
	TIM_SetCompare1(TIM3,300);
//*************************第三部分，色环位置拿并放置**********************//
	while(1){
		wait_send_shumeipai(0x37,10000);//如果发送了信息，但是一直没有得到回应，那么就一直发送
		transfer_position_sehuan(0x01,0x02);
		car_move_distance(direct_x,direct_y,180,150);
		if(Serial_RxPacket[8] != 0x34){
			break;
		}
		//如果返回了没看到的信息，那就微微调整一下，再重复给树莓派发送
		//反之如果返回了看到的信息（也即第九位并非0x34）,则可以退出该程序
	}
	PID_move(0,0,0,1000,-180.0f,4);  //暂时按照没有角度矫正的思路调试
	//精细定标
	{
		wait_send_shumeipai(0x38,10000);//如果发送了信息，但是一直没有得到回应，那么就一直发送
		transfer_position_sehuan(0x02,0x02);	
		if(Serial_RxPacket[8] == 0x34){
			car_move_distance(0,0,70,50);//如果没有看到颜色，那就不移动
		}else{	
			car_move_distance(direct_x,direct_y,130,100);//如果看到颜色，那就按照正常的方式移动
		}
	}//至此第二次定标完成
	
	//物块平台抓物块
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);
	set_zhuashou_he();
	//色换放置第一个
	fangzhi(task[0]);	
	set_wukuaipingtai_weizhi(0x02);
	delay_ms1((int)shengjiang_control(33,2000,240));
	//物块平台抓物块
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);
	set_zhuashou_he();
	
	//色环放置第二个	
	fangzhi(task[1]);
	set_wukuaipingtai_weizhi(0x03);
	delay_ms1((int)shengjiang_control(33,2000,240));
	//物块平台抓物块
	move_all(-48.0f,1000,200   ,33,2000,200   ,135,13);
	set_zhuashou_he();
	
	//色环放置第三个	
	fangzhi(task[2]);
	set_wukuaipingtai_weizhi(0x01);
	//色环拿一
	na(task[0]);
	set_wukuaipingtai_weizhi(0x02);
	//色环拿二
	na(task[1]);
	set_wukuaipingtai_weizhi(0x03);
	//色环拿三
	na(task[2]);
	//复位到识别的位置
	set_wukuaipingtai_weizhi(0x01);
	move_all1(-120.0f,1000,200   ,0,1000,200   ,278,13);
	set_zhuashou_kai();
//*************************到下一个区域***************************//	
	PID_move(-30,0,0,300,-180,1);//在粗加工区回退
	PID_move(0,0,0,500,-180,3);
	car_move1(140,140+680,180,180);//从绿色出发
	delay_ms1(100);
	
	mypid.integral = 0;PID_move(0,0,0,1000,90,0);
	PID_move(0,0,0,1000,90,3);
	
	car_move1(140,140+620,180,150);
//****************************调整(亮度等参数)****************************//
	TIM_SetCompare1(TIM3,300);
//*************************第四部分，色环位置放置**********************//
	while(1){
		wait_send_shumeipai(0x37,10000);//如果发送了信息，但是一直没有得到回应，那么就一直发送
		transfer_position_sehuan(0x01,0x02);
		car_move_distance(direct_x,direct_y,180,150);
		if(Serial_RxPacket[8] != 0x34){
			break;
		}
		//如果返回了没看到的信息，那就微微调整一下，再重复给树莓派发送
		//反之如果返回了看到的信息（也即第九位并非0x34）,则可以退出该程序
	}
	PID_move(0,0,0,1000,90.0f,4);
	//精细定标
	{
		wait_send_shumeipai(0x38,10000);//如果发送了信息，但是一直没有得到回应，那么就一直发送
		transfer_position_sehuan(0x02,0x02);	
		if(Serial_RxPacket[8] == 0x34){
			car_move_distance(0,0,70,50);//如果没有看到颜色，那就不移动
		}else{	
			car_move_distance(direct_x,direct_y,130,100);//如果看到颜色，那就按照正常的方式移动
		}
	}//至此第二次定标完成
	
	//物块平台抓物块
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);
	set_zhuashou_he();
	//色环放置第一个
	fangzhi(task[0]);
	set_wukuaipingtai_weizhi(0x02);
	delay_ms1((int)shengjiang_control(33,2000,240));
	
	//物块平台抓物块
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);
	set_zhuashou_he();
	//色环放置第二个	
	fangzhi(task[1]);
	set_wukuaipingtai_weizhi(0x03);
	delay_ms1((int)shengjiang_control(33,2000,240));
	
	//物块平台抓物块
	move_all(-48.0f,1000,200   ,33,2000,200   ,135,13);
	set_zhuashou_he();
	//放置第三个	
	fangzhi(task[2]);
	
	//复位到识别的位置
	set_wukuaipingtai_weizhi(0x01);
	move_all1(-120.0f,1000,200   ,0,1000,200   ,278,13);
	set_zhuashou_kai();
//*************************第五部分，回到圆盘**********************//
	PID_move(-30,0,0,300,90,1);//回退
	delay_ms1(100);
	//回到原料
	car_move1(140,220+650,180,180);//从绿色出发
	
	mypid.integral = 0;PID_move(0,0,0,1000,0,0);
	PID_move(0,0,0,1000,0,3);
	
	car_move1(140,370+20,180,180);
//*****************************第二轮**************************//
//****************************调整(亮度等参数)****************************//
	TIM_SetCompare1(TIM3,300);
//********************第二部分，识别物块颜色，按照指定的顺序搬运**************************//
	move_all1(40.0f,1000,200   ,0,2000,200   ,278,13);
	//提前发送0x61
	Serial_GetRxFlag();Serial_SendByte(0x61);
	PID_move(30,0,0,300,0,1);//在粗加工区往前靠
	delay_ms1(100);	
	while(1){
		while(1){
			if(Serial_GetRxFlag() == 1){
				break;  //是wait发送的分解版
			}
		}
		transfer_position_wukuai(0x01,70.0f,0x00);
		car_move_distance(direct_x,direct_y,100,80);//至此第一次定标完成
		if(Serial_RxPacket[8] != 0x34){
			break;
		}else{
			wait_send_shumeipai(0x61,15000);
		}
	}
	move_all(60.0f,1000,200   ,0,2000,200   ,278,13);
	wait_send_shumeipai(0x34,20000);
	ypcode[0]= Serial_RxPacket[0]-0x30;ypcode[1] = Serial_RxPacket[1]-0x30;ypcode[2] = Serial_RxPacket[2]-0x30;
	
	//1.圆台拿走第一个物块
	set_wukuaipingtai_weizhi(0x01);
	yuantai_na(ypcode[0]);
	
	//2.圆台拿走第二个物块
	set_wukuaipingtai_weizhi(0x02);
	yuantai_na1(ypcode[1]);   //达到指定的识别的位置
		//判断物块是否静止
	wait_send_shumeipai(0x62,30000);
	while(1){
		if(Serial_RxPacket[0] == 0x39 && Serial_RxPacket[1] == 0x38){
			break;
		}
	}Serial_RxPacket[0] = 0x30;Serial_RxPacket[1] = 0x30;
	yuantai_na2(ypcode[1]);  //一旦识别到物块静止，抓物块到物块平台上
		
	//3.圆台拿走第三个物块
	set_wukuaipingtai_weizhi(0x03);
	yuantai_na1(ypcode[2]);	 //达到指定的识别的位置
	wait_send_shumeipai(0x63,30000);
	while(1){
		if(Serial_RxPacket[0] == 0x39 && Serial_RxPacket[1] == 0x38){
			break;
		}
	}Serial_RxPacket[0] = 0x30;Serial_RxPacket[1] = 0x30;	
	yuantai_na2(ypcode[2]);	 //一旦识别到物块静止，抓物块到物块平台上

	TIM_SetCompare1(TIM3,0); //关闭补光灯
	//复位到识别的位置
	set_wukuaipingtai_weizhi(0x01);
	move_all1(-120.0f,1000,200   ,0,1000,200   ,278,13);
	set_zhuashou_kai();
//***************************到下一个区域**************************//	
	//到粗加工区
	PID_move(-30,0,0,400,0,1);//回倒车
	delay_ms1(200);
	car_move1(0,400,150,200);
	
	mypid.integral = 0;PID_move(0,0,0,1000,90,0);//在原地旋转
	PID_move(0,0,0,1000,90,3);
	
	car_move1(0,-1700,150,250);//快速通过
	
	mypid.integral = 0;PID_move(0,0,0,1000,-180,0);//在粗加工区原地旋转
	PID_move(0,0,0,1000,-180,3);
//****************************调整(亮度等参数)****************************//
	TIM_SetCompare1(TIM3,300);
//*************************第三部分，色环位置拿并放置**********************//
	while(1){
		wait_send_shumeipai(0x37,20000);//如果发送了信息，但是一直没有得到回应，那么就一直发送
		transfer_position_sehuan(0x01,0x02);
		car_move_distance(direct_x,direct_y,180,150);
		if(Serial_RxPacket[8] != 0x34){
			break;
		}
		//如果返回了没看到的信息，那就微微调整一下，再重复给树莓派发送
		//反之如果返回了看到的信息（也即第九位并非0x34）,则可以退出该程序
	}
	PID_move(0,0,0,1000,-180.0f,4);
	//精细定标
	{
		wait_send_shumeipai(0x38,20000);//如果发送了信息，但是一直没有得到回应，那么就一直发送
		transfer_position_sehuan(0x02,0x02);	
		if(Serial_RxPacket[8] == 0x34){
			car_move_distance(0,0,70,50);//如果没有看到颜色，那就不移动
		}else{	
			car_move_distance(direct_x,direct_y,130,100);//如果看到颜色，那就按照正常的方式移动
		}
	}//至此第二次定标完成
	//物块平台抓物块
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);
	set_zhuashou_he();
	//色环放置第一个
	fangzhi(task[3]);
	set_wukuaipingtai_weizhi(0x02);
	delay_ms1((int)shengjiang_control(33,2000,240));
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);//物块平台抓物块
	set_zhuashou_he();
	
	//色环放置第二个	
	fangzhi(task[4]);
	set_wukuaipingtai_weizhi(0x03);
	delay_ms1((int)shengjiang_control(33,2000,240));
	move_all(-48.0f,1000,200   ,33,2000,200   ,135,13);//物块平台抓物块
	set_zhuashou_he();
	
	//色环放置第二个	
	fangzhi(task[5]);
	set_wukuaipingtai_weizhi(0x01);
	//拿一
	na(task[3]);
	set_wukuaipingtai_weizhi(0x02);
	//拿二
	na(task[4]);
	set_wukuaipingtai_weizhi(0x03);
	//拿三
	na(task[5]);
	//复位到识别的位置
	set_wukuaipingtai_weizhi(0x01);
	move_all1(-120.0f,1000,200   ,0,1000,200   ,278,13);
	set_zhuashou_kai();
//*************************到下一个区域***************************//	
	PID_move(-30,0,0,300,-180,1);//在粗加工区回退
	PID_move(0,0,0,500,-180,3);
	car_move1(140,140+680,180,180);//从绿色出发
	delay_ms1(100);
	
	mypid.integral = 0;PID_move(0,0,0,1000,90,0);
	PID_move(0,0,0,1000,90,3);
	
	car_move1(140,140+620,180,150);
//****************************调整(亮度等参数)****************************//
	TIM_SetCompare1(TIM3,300);
//********************************码垛****************************
	while(1){
		wait_send_shumeipai(0x39,10000);	
		transfer_position_wukuai(0x02,70.0f,0x02);
		car_move_distance(direct_x,direct_y,180,150);//至此第一次定标完成
		delay_ms1(200);
		if(Serial_RxPacket[8] != 0x34){
			break;
		}
	}
	{
		wait_send_shumeipai(0x39,10000);		
		transfer_position_wukuai(0x02,70.0f,0x02);
		if(Serial_RxPacket[8] == 0x34){
			car_move_distance(0,0,70,50);
		}else{	
			car_move_distance(direct_x,direct_y,180,150);//至此第二次定标完成
		}
	}
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);//物块平台抓物块
	set_zhuashou_he();
	//放置第一个
	fangzhi_maduo(task[3]);
	set_wukuaipingtai_weizhi(0x02);
	delay_ms1((int)shengjiang_control(33,2000,240));
	move_all(-48.0f,1000,200   ,33,1000,200   ,135,13);//物块平台抓物块
	set_zhuashou_he();
	
	//放置第二个	
	fangzhi_maduo(task[4]);
	set_wukuaipingtai_weizhi(0x03);
	delay_ms1((int)shengjiang_control(33,2000,240));
	move_all(-48.0f,1000,200   ,33,2000,200   ,135,13);//物块平台抓物块
	set_zhuashou_he();
	
	//放置第三个	
	fangzhi_maduo(task[5]);
	set_wukuaipingtai_weizhi(0x01);
	//回到启停区
	PID_move(-30,0,0,300,90,1);//回退
	car_move1(140,220+650,180,180);//从绿色出发
	move_all1(0.0f,1000,200   ,0,1000,200   ,64.8,13);
	
	mypid.integral = 0;PID_move(0,0,0,1000,0,0);
	PID_move(0,0,0,1000,0,3);
	
	car_move1(140,2000-150,180,180);
	car_move2(-130,0,150,150);//在粗加工区往前靠
	
	jixie_reset();
#endif
}

void fangzhi_green(void)
{
	move_all(-48.0f,1000,200   ,25,1000,200   ,135,13);
	move_all(-20.0f,1000,200   ,25,1000,200   ,135,13);
	move_all(-20.0f,1000,200   ,25,1000,200   ,278,13);
	move_all(-50.0f,1000,200   ,132,1000,200   ,278,13);
	set_zhuashou_kai();
}

void fangzhi_red(void)
{
	move_all(-48.0f,1000,200   ,25,1000,200   ,135,13);
	move_all(3.0f+1.0f,1000,200   ,25,1000,200   ,236.0f,13);
	move_all(3.0f+1.0f,1000,200   ,132,1000,200   ,236.0f,13);
	
	set_zhuashou_kai();
}
void fangzhi_blue(void)
{
	move_all(-48.0f,1000,200   ,25,1000,200   ,135,13);
	move_all(3.0f+5.8f,1000,200   ,25,1000,200   ,317.0f+1.4f,13);
	move_all(3.0f+5.8f,1000,200   ,132,1000,200   ,317.0f+1.4f,13);
	set_zhuashou_kai();
}

void fangzhi(uint8_t task)
{
	if(task == 0x01){
		fangzhi_red();
	}else if(task == 0x02){
		fangzhi_green();
	}else if(task == 0x03){
		fangzhi_blue();
	}
}

void fangzhi_maduo(uint8_t task)
{
	if(task == 0x01){
		move_all(-48.0f,1000,200   ,25,1000,200   ,135,13);
		move_all(3.0f,1000,200   ,25,1000,200   ,236,13);
		move_all(3.0f,1000,200   ,130-70,1000,200   ,236,13);
		set_zhuashou_kai();
	}else if(task == 0x02){
		move_all(-48.0f,1000,200   ,25,1000,200   ,135,13);
		move_all(-48.0f,1000,200   ,25,1000,200   ,180,13);
		move_all(-120.0f,1000,200   ,25,1000,200   ,278,13);
		move_all(-50.0f,1000,200   ,130-70,1000,200   ,278,13);
		set_zhuashou_kai();
	}else if(task == 0x03){
		move_all(-48.0f,1000,200   ,25,1000,200   ,135,13);
		move_all(-48.0f,1000,200   ,25,1000,200   ,180,13);
		move_all(-120.0f,1000,200   ,25,1000,200   ,317,13);
		move_all(3.0f,1000,200   ,130-70,1000,200   ,317,13);
		set_zhuashou_kai();
	}
}

void na_green(void)
{	
	move_all(-10.0f,1000,200   ,130,1000,200   ,278,13);
	move_all(-50.0f,1000,200   ,130,1000,200   ,278,13);
	set_zhuashou_he();
	move_all(-50.0f,1000,200   ,50,1000,220   ,278,13);
	move_all(-48.0f,1000,200   ,25,1000,220   ,135,13);
	move_all(-48.0f,1000,200   ,33,1000,220   ,135,13);
	set_zhuashou_kai();
}

void na_blue(void)
{
	move_all(3.0f,1000,200   ,110,1000,200   ,317,13);
	move_all(3.0f,1000,200   ,130,1000,200   ,317,13);
	set_zhuashou_he();
	move_all(8.0f,1000,200   ,25,1000,220   ,317,13);
	move_all(-48.0f,1000,200   ,25,1000,220   ,135,13);
	move_all(-48.0f,1000,200   ,33,1000,220   ,135,13);
	set_zhuashou_kai();
}

void na_red(void)
{
	move_all(3.0f,1000,200   ,130,1000,200   ,236,13);
	set_zhuashou_he();
	move_all(-50.0f,1000,200   ,50,1000,220   ,236,13);
	move_all(-48.0f,1000,200   ,25,1000,220   ,135,13);
	move_all(-48.0f,1000,200   ,33,1000,220   ,135,13);
	set_zhuashou_kai();
}

void na(uint8_t task)
{
	if(task == 0x01){
		na_red();
	}else if(task == 0x02){
		na_green();
	}else if(task == 0x03){
		na_blue();
	}
}

void yuantai_na(uint8_t task)
{
	if(task == 0x01){
		set_zhuashou_kai();  
		move_all(-100.0f,1000,220   ,50,2000,220   ,278,13);
		set_zhuashou_he();  
		move_all(-48.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,33,2000,220   ,135,13);
		set_zhuashou_kai();	 
	}else if(task == 0x02){
		set_zhuashou_kai();	
		move_all(63.0f,1000,200   ,48,2000,220   ,295,13);
		set_zhuashou_he();   
		move_all(20.0f,1000,200   ,25,2000,220   ,295,13);//**********多一个回收的动作
		move_all(-20.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,33,2000,220   ,135,13);
		set_zhuashou_kai();  
	}else if(task == 0x03){
		set_zhuashou_kai();  
		move_all(63.0f,1000,200   ,48,2000,220   ,261,13);
		set_zhuashou_he();  
		move_all(-48.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,33,2000,220   ,135,13);
		set_zhuashou_kai();  
	}
}

void yuantai_na1(uint8_t task)
{
	if(task == 0x01){
		set_zhuashou_kai();
		move_all(40.0f,1000,200   ,0,2000,200   ,278,13);
	}else if(task == 0x02){
		set_zhuashou_kai();	
		move_all(63.0f,1000,200   ,18,2000,220   ,295,13);
	}else if(task == 0x03){
		set_zhuashou_kai();  
		move_all(63.0f,1000,200   ,18,2000,220   ,261,13);
	}
}

void yuantai_na2(uint8_t task)
{
	if(task == 0x01){
		move_all(-100.0f,1000,220   ,50,2000,220   ,278,13);
		set_zhuashou_he();   
		move_all(-48.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,33,2000,220   ,135,13);
		set_zhuashou_kai();	 
	}else if(task == 0x02){
		move_all(63.0f,1000,200   ,48,2000,220   ,295,13);
		set_zhuashou_he();   
		move_all(20.0f,1000,200   ,25,2000,220   ,295,13);//**********多一个回收的动作
		move_all(-20.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,33,2000,220   ,135,13);
		set_zhuashou_kai();  
	}else if(task == 0x03){
		move_all(63.0f,1000,200   ,48,2000,220   ,261,13);
		set_zhuashou_he();  
		move_all(-48.0f,1000,200   ,25,2000,220   ,135,13);
		move_all(-48.0f,1000,200   ,33,2000,220   ,135,13);
		set_zhuashou_kai();
	}
}


//if task_choose = 0x01,则为检测转盘处的物块圆心
//if task_choose = 0x02,则为检测码垛处的物块圆心
//distance 为物块的高度
//color 为选择定标到哪一个物块的位置 为0表示任何位置都可以
void transfer_position_wukuai(uint8_t task_choose,float distance,uint8_t color)
{
	x_origin = (Serial_RxPacket[0]-0x30)*1000+(Serial_RxPacket[1]-0x30)*100+(Serial_RxPacket[2]-0x30)*10+(Serial_RxPacket[3]-0x30);
	y_origin = (Serial_RxPacket[4]-0x30)*1000+(Serial_RxPacket[5]-0x30)*100+(Serial_RxPacket[6]-0x30)*10+(Serial_RxPacket[7]-0x30);
	float bilichi = 0;
	if(task_choose == 0x01){
		bilichi = -0.0035f*(distance+80.0f) + 1.1967f;
		x_origin = x_origin - 322;
		y_origin = y_origin - 165;
	}else if(task_choose == 0x02){
		bilichi = -0.0033f*(distance) + 1.1655f;
		x_origin = x_origin - 314;
		y_origin = y_origin - 238;
	}
	direct_x = x_origin * bilichi;   
	direct_y = y_origin * bilichi;
	//下面的代码仅仅和颜色有关
	if(color == 2){
		if(Serial_RxPacket[8] == 0x32){
		}else if(Serial_RxPacket[8] == 0x31){
				direct_x = direct_x + distance_wukuai;
		}else if(Serial_RxPacket[8] == 0x33){
				direct_x = direct_x - distance_wukuai;
		}
	}else if(color == 1){
		if(Serial_RxPacket[8] == 0x31){
		}else if(Serial_RxPacket[8] == 0x32){
			direct_x = direct_x - distance_wukuai;
		}else if(Serial_RxPacket[8] == 0x33){
				direct_x = direct_x - 2*distance_wukuai;
		}
	}else if(color == 3){
		if(Serial_RxPacket[8] == 0x33){
		}else if(Serial_RxPacket[8] == 0x32){
			direct_x = direct_x + distance_wukuai;
		}else if(Serial_RxPacket[8] == 0x31){
				direct_x = direct_x + 2*distance_wukuai;
		}
	}
	if(Serial_RxPacket[8] == 0x34){
		direct_y = 5;direct_x = -5;
	}
}


//与任务码相关联，定位色环位置
//fenbian_choose
	//fenbian_choose == 0x01 则为小分辨率
	//fenbian_choose == 0x02 则为大分辨率
//task_num 
	//选择移动到对应颜色的位置 为零表示与颜色无关
void transfer_position_sehuan(uint8_t fenbian_choose,uint8_t task_num)
{   
	x_origin = (Serial_RxPacket[0]-0x30)*1000+(Serial_RxPacket[1]-0x30)*100+(Serial_RxPacket[2]-0x30)*10+(Serial_RxPacket[3]-0x30);
	y_origin = (Serial_RxPacket[4]-0x30)*1000+(Serial_RxPacket[5]-0x30)*100+(Serial_RxPacket[6]-0x30)*10+(Serial_RxPacket[7]-0x30);
	float x_center,y_center;//x加物块右移//y+物块下移
	if(fenbian_choose == 0x01){
		x_center =316;y_center =238;
		x_origin = x_origin - x_center;
		y_origin = y_origin - y_center;
		
		direct_x = x_origin * 1.1805555;    //实际尺寸系数
		direct_y = y_origin * 1.1805555;
	}else if(fenbian_choose == 0x02){
		if(Serial_RxPacket[8] == 0x31){
			x_center = x_center_red;y_center = y_center_red;
		}else if(Serial_RxPacket[8] == 0x32){
			x_center = x_center_green;y_center = y_center_green;
		}else if(Serial_RxPacket[8] == 0x33){
			x_center = x_center_blue;y_center = y_center_blue;
		}
		x_origin = x_origin - x_center;
		y_origin = y_origin - y_center;
	
		direct_x = x_origin * 0.29239766;    //实际尺寸系数
		direct_y = y_origin * 0.29239766;
	}
	
	if(task_num == 2){
		if(Serial_RxPacket[8] == 0x32){
		}else if(Serial_RxPacket[8] == 0x31){
				direct_x = direct_x + distance_wukuai;
		}else if(Serial_RxPacket[8] == 0x33){
				direct_x = direct_x - distance_wukuai;
		}
	}else if(task_num == 1){
		if(Serial_RxPacket[8] == 0x31){
		}else if(Serial_RxPacket[8] == 0x32){
			direct_x = direct_x - distance_wukuai;
		}else if(Serial_RxPacket[8] == 0x33){
				direct_x = direct_x - 2*distance_wukuai;
		}
	}else if(task_num == 3){
		if(Serial_RxPacket[8] == 0x33){
		}else if(Serial_RxPacket[8] == 0x32){
			direct_x = direct_x + distance_wukuai;
		}else if(Serial_RxPacket[8] == 0x31){
				direct_x = direct_x + 2*distance_wukuai;
		}
	}
	if(Serial_RxPacket[8] == 0x34){
				direct_y = 3;direct_x = -3;
	}
}

void jixie_reset(void)
{	
	while(swith_out() == 1){		
		LED_Toggle2();
		delay_ms1(50);
	}
	
	set_zhuashou_kai();
	move_all(0.0f,1000,200   ,0,1000,200   ,68.4,13);
	set_wukuaipingtai_Angle(20.0f,300);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	TIM_Cmd(TIM5, ENABLE); 
	while(1){}
}

/*
	先发送命令，等待一段时间如果还没有返回
	则继续发送，直到收到信息，立刻退出
*/
void wait_send_shumeipai(uint8_t message,uint16_t time)
{	
	Serial_GetRxFlag();//清空标志位
	Serial_SendByte(message);
	while(1){
		if(Serial_GetRxFlag() == 1){
			break;
		}
	}
}

//定时显示OLED和LCD显示屏
void TIM5_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
	{   
		imu_scan(fAcc, fGyro, fAngle);
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		OLED_ShowSignedNum(1,1,fAngle[2],3);
		OLED_ShowChar(1,5,'.');
		OLED_ShowNum(1,6,ABS((fAngle[2]-(int)fAngle[2])*1000),3);
		OLED_ShowSignedNum(2,1,angle,3);
		OLED_ShowChar(2,5,'.');
		OLED_ShowNum(2,6,ABS((angle-(int)angle)*1000),3);
		
	}
}

void clear_ckp(void)
{
	sprintf(tjcstr, "t0.txt=\"  \"");
			HMISends(tjcstr);
	        HMISendb(0xff);//在显示屏上面显示任务
}
