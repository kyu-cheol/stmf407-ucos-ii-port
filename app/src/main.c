#include "includes.h"
#include "app_util.h"

__attribute__((aligned(8)))
static OS_STK AppTaskStartStk[APP_CFG_STARTUP_TASK_STK_SIZE];

__attribute__((aligned(8)))
static OS_STK AppTaskGuidanceStk[APP_CFG_GUIDANCE_TASK_STK_SIZE];

__attribute__((aligned(8)))
static OS_STK AppTaskImuStk[APP_CFG_IMU_TASK_STK_SIZE];

__attribute__((aligned(8)))
static OS_STK UltrasonicSensorStk[APP_CFG_ULTRASONIC_SENSOR_TASK_STK_SIZE];

__attribute__((aligned(8)))
static OS_STK OTATrigTaskStk[OTA_TRIG_TASK_STK_SIZE];

__attribute__((aligned(8)))
static OS_STK AppTaskCommStk[APP_CFG_COMM_TASK_STK_SIZE];

static void AppTaskStart(void *p_arg);
static void MotorSpeedControlTask(void *p_arg);
static void UpdateWheelOdometry(void *p_arg);
static void UltrasonicSensorTask(void *p_arg);
static void OTATrigTask(void *p_arg);
static void AppTaskComm(void *p_arg);

OS_TMR *my_timer;
OS_EVENT *UartMutex = NULL;
OS_EVENT *LeftEncoderMbox;
OS_EVENT *HCSR04DurationMbox;
OS_EVENT *ButtonSem;

// 20ms 제어 주기에 맞춘 dt 설정
#define dt 0.1f

// 속도 제어 게인값 (상황에 맞게 튜닝 필요)
#define P_GAIN_FAST 25.0f
#define I_GAIN_FAST 12.0f
#define D_GAIN_FAST 0.5f

#define P_GAIN_SLOW 15.0f
#define I_GAIN_SLOW 9.0f
#define D_GAIN_SLOW 0.0f

//extern volatile uint32_t enc_pulse_edge_cnt; // EXTI 인터럽트에서 증가하는 펄스 수
volatile int32_t enc_pos_cnt;

volatile float current_rpm = 0.0;            // 현재 RPM
extern volatile int32_t target_rpm;          // 목표 RPM

volatile float realError = 0.0;
volatile float accError = 0.0;
volatile float errorGap = 0.0;

volatile float pControl = 0.0;
volatile float iControl = 0.0;
volatile float dControl = 0.0;

volatile float left_wheel_distance;

extern volatile uint32_t echo_duration;
extern uint8_t target_rpm_update_flag;

extern volatile uint32_t rx_count;
uint8_t g_cmd_vel_buffer[8];

extern float linear_x;
extern float angular_z;
extern uint8_t uart_idle_flag;

//void spi_rx_handler(SPI_x *SPIx, uint8_t data);
void HCSR04_sensor_init(void);

void BSP_Init(void)
{	
	led_setup();
	button_setup();

	timer_init(TIM1, 8400 - 1, 1000 - 1);		// 168000000 2000hz
	timer_start_PWM(TIM1, 1, GPIOE, 9, 500);

	// timer_init(TIM2, 4800 - 1, 1000 - 1);		// 84000000
	// timer_start_PWM(TIM2, 2, GPIOA, 1, 500);

	// gpio_set_exti(GPIOC, 7, GPIO_EXTI_BOTH);	// Encoder A상
	// gpio_set_exti(GPIOC, 9, GPIO_EXTI_BOTH);	// Encoder B상

	timer3_encoder_init();

	wheel_left_init();
	wheel_right_init();

	HCSR04_sensor_init();

	//uart_recv_it_onoff(UART3, 1);
	uart3_dma_rx_init(g_cmd_vel_buffer, sizeof(g_cmd_vel_buffer));
}

void main(void)
{
	OSInit();

	OSTaskCreate((void (*)(void *)) AppTaskStart, \
				 (void *) 0, \
				 (OS_STK *) &AppTaskStartStk[APP_CFG_STARTUP_TASK_STK_SIZE - 1], \
				 (INT8U) APP_CFG_STARTUP_TASK_PRIO);


	OSStart();
}

static void AppTaskStart(void *p_arg)
{
	INT8U err;
	(void)p_arg;

	s_printf("AppTaskStart\r\n");

	// 시스템에서 사용되는 peripherals init
	BSP_Init();

	// Systick 인터럽트 최하위 우선순위 부여후 실행
	// OSStart 이후에 호출되어야 함.
	// task들 생성하기 전에 먼저 켜도 되는건가?
	OS_CPU_SysTickInitFreq(CPU_FREQ);
	
	UartMutex = OSMutexCreate(3u, &err);
	//MissileEventFlags = OSFlagCreate(0x00, &err);
	LeftEncoderMbox = OSMboxCreate((void *)0);
	HCSR04DurationMbox = OSMboxCreate((void *)0);
	ButtonSem = OSSemCreate(0);

	/* 1. Motor Speed Control Task 생성 (FPU 지원) */
    OSTaskCreateExt((void (*)(void *)) MotorSpeedControlTask,
                    (void *) 0,
                    (OS_STK *) &AppTaskGuidanceStk[APP_CFG_GUIDANCE_TASK_STK_SIZE - 1],
                    (INT8U)    APP_CFG_GUIDANCE_TASK_PRIO,
                    (INT16U)   APP_CFG_GUIDANCE_TASK_PRIO, // ID는 보통 우선순위와 동일하게 부여합니다.
                    (OS_STK *) &AppTaskGuidanceStk[0],     // 스택 바닥(최하위) 주소
                    (INT32U)   APP_CFG_GUIDANCE_TASK_STK_SIZE,
                    (void *)   0,
                    (INT16U)  (OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR | OS_TASK_OPT_SAVE_FP)); // ★ FPU 백업 옵션 추가

    /* 2. IMU Task 생성 (FPU 지원) */
    OSTaskCreateExt((void (*)(void *)) UpdateWheelOdometry,
                    (void *) 0,
                    (OS_STK *) &AppTaskImuStk[APP_CFG_IMU_TASK_STK_SIZE - 1],
                    (INT8U)    APP_CFG_IMU_TASK_PRIO,
                    (INT16U)   APP_CFG_IMU_TASK_PRIO,
                    (OS_STK *) &AppTaskImuStk[0],
                    (INT32U)   APP_CFG_IMU_TASK_STK_SIZE,
                    (void *)   0,
                    (INT16U)  (OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR | OS_TASK_OPT_SAVE_FP)); // ★ FPU 백업 옵션 추가

	/* 2. IMU Task 생성 (FPU 지원) */
    OSTaskCreateExt((void (*)(void *)) UltrasonicSensorTask,
                    (void *) 0,
                    (OS_STK *) &UltrasonicSensorStk[APP_CFG_ULTRASONIC_SENSOR_TASK_STK_SIZE - 1],
                    (INT8U)    APP_CFG_ULTRASONIC_SENSOR_TASK_PRIO,
                    (INT16U)   APP_CFG_ULTRASONIC_SENSOR_TASK_PRIO,
                    (OS_STK *) &UltrasonicSensorStk[0],
                    (INT32U)   APP_CFG_ULTRASONIC_SENSOR_TASK_STK_SIZE,
                    (void *)   0,
                    (INT16U)  (OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR | OS_TASK_OPT_SAVE_FP)); // ★ FPU 백업 옵션 추가

	OSTaskCreate((void (*)(void *))OTATrigTask,
    			 (void *)0,
    			 &OTATrigTaskStk[OTA_TRIG_TASK_STK_SIZE - 1],
    			 OTA_TRIG_TASK_PRIO);
	
    /* 3. Communication Task 생성 (FPU 지원) */
    OSTaskCreateExt((void (*)(void *)) AppTaskComm,
                    (void *) 0,
                    (OS_STK *) &AppTaskCommStk[APP_CFG_COMM_TASK_STK_SIZE - 1],
                    (INT8U)    APP_CFG_COMM_TASK_PRIO,
                    (INT16U)   APP_CFG_COMM_TASK_PRIO,
                    (OS_STK *) &AppTaskCommStk[0],
                    (INT32U)   APP_CFG_COMM_TASK_STK_SIZE,
                    (void *)   0,
                    (INT16U)  (OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR | OS_TASK_OPT_SAVE_FP)); // ★ FPU 백업 옵션 추가
				 
	s_printf("uC/OS-II RTOS Initialization complete. Deleting Start Task...\r\n");
	OSTaskDel(OS_PRIO_SELF);
}


static void MotorSpeedControlTask(void *p_arg)
{
	INT32U last_wake_time;
	int32_t local_target_rpm;
	uint32_t pulse_cnt;

	// 하드웨어 타이머 연산용 변수 추가
    uint16_t current_cnt = 0;
    uint16_t last_cnt = 0;
    int16_t encoder_diff = 0;

#if OS_CRITICAL_METHOD == 3
	OS_CPU_SR cpu_sr = 0;
#endif

	last_cnt = (uint16_t)(TIM3->CNT);
	last_wake_time = OSTimeGet();

	/* 100ms 주기로 DC 바퀴의 속도 제어 */
	while (1) {
		//s_printf("[%d]\r\n", target_rpm);

		// 현재 encoder 펄스 수 가져옴
		OS_ENTER_CRITICAL();
		local_target_rpm = target_rpm;
		OS_EXIT_CRITICAL();
		
		current_cnt = (uint16_t)(TIM3->CNT);
        
        // 16비트 정수 차이 연산 후 int16_t 강제 형변환
        // -> 0에서 65535로 튀는 언더플로우나 65535에서 0으로 넘치는 오버플로우가 자동 보정됩니다.
        encoder_diff = (int16_t)(current_cnt - last_cnt);
        last_cnt = current_cnt; // 다음 주기를 위해 현재 값을 저장

        // 음수(역회전)로 들어온 이동량도 양수(펄스 수 변위)로 변경해 줍니다.
        pulse_cnt = (encoder_diff < 0) ? -encoder_diff : encoder_diff;
        
        // 정방향/역방향 부호가 살아있는 누적 위치 데이터가 필요하다면 사용하세요.
		OS_ENTER_CRITICAL();
        enc_pos_cnt += encoder_diff;
		OS_EXIT_CRITICAL();

		// 100ms 동안의 펄스 수로 RPM 계산
		current_rpm = (float)pulse_cnt * 0.2435f; // (60 * pulse_cnt) / (0.1 * 11 * 56 * 4)

		if (local_target_rpm == 0) {
			accError = 0.0f;  // 과거에 쌓인 모든 적분 찌꺼기를 깨끗하게 청소
            realError = 0.0f;
            errorGap = 0.0f;

            pControl = 0.0f;
            iControl = 0.0f;
            dControl = 0.0f;
		}
		else {
			// PID 오차 계산
			errorGap = (float)local_target_rpm - current_rpm - realError;
			realError = (float)local_target_rpm - current_rpm;
			accError += realError * dt;

			// // I항 누적 한계치 설정
			// float max_i_limit = 900.0f / I_GAIN_SPEED; 
        	// if (accError > max_i_limit)  accError = max_i_limit;
        	// if (accError < -max_i_limit) accError = -max_i_limit;

			// PID 제어량 계산
			if (local_target_rpm <= 30) {
				pControl = P_GAIN_SLOW * realError;
				iControl = I_GAIN_SLOW * accError;
				dControl = D_GAIN_SLOW * (errorGap / dt);
			}
			else {
				pControl = P_GAIN_FAST * realError;
				iControl = I_GAIN_FAST * accError;
				dControl = D_GAIN_FAST * (errorGap / dt);
			}
		}

		float amount_of_control = pControl + iControl + dControl;

		if (amount_of_control < 0.0f) {
			amount_of_control = 0.0f;
		}
		if (amount_of_control > 999.0f) {
			amount_of_control = 999.0f;
		}

		// 왼쪽 모터 최종 제어량 업데이트
		TIM1->CCR1 = (uint16_t)amount_of_control;

		// Wheel odometry 계산을 위한 encoder_diff 변수를 mailbox로 post
		static int32_t mailbox_buf;
		mailbox_buf = (int32_t)encoder_diff;

		OSMboxPost(LeftEncoderMbox, (void *)&mailbox_buf);

		TimeDlyUntil(&last_wake_time, 100);
	}
}

// 라즈베리파이 시리얼 전송용 구조체 정의 (바이트 정렬)
typedef struct __attribute__((packed)) {
    uint32_t timestamp;        // 시간 정보 (ms)
    float    pos_x;            // 누적 X (m)
    float    pos_y;            // 누적 Y (m)
    float    theta;            // 누적 각도 (rad)
    float    linear_vel_x;     // 현재 선속도 (m/s)
    float    angular_vel_z;    // 현재 각속도 (rad/s)
} OdomPacket_T;

OdomPacket_T tx_odom_packet;   // 라즈베리파이로 보낼 전송 버퍼

float robot_theta = 0.0f;
float robot_x = 0.0f;
float robot_y = 0.0f;
#define ROBOT_WHEEL_BASE 0.33f

void update_robot_odometry(float left_delta, float right_delta)
{
#if OS_CRITICAL_METHOD == 3
	OS_CPU_SR cpu_sr = 0;
#endif

	// 100ms 동안 정면 방향으로 몇 m 앞으로 전진했는지
	// 100ms 동안의 이전 위치 벡터로 부터 몇 라디안 기울어 졌는지
	float delta_s = (right_delta + left_delta) / 2.0f;
    float delta_theta = (right_delta - left_delta) / ROBOT_WHEEL_BASE;

	// 한 주기가 100ms 이므로 그 기간동안 직진한걸로 간주하는 효과를 감쇄
	float middle_theta = robot_theta + (delta_theta / 2.0f);

	OS_ENTER_CRITICAL();

    // 1. [Pose] 위치 누적 연산
    robot_x += delta_s * cosf(middle_theta);
    robot_y += delta_s * sinf(middle_theta);
    robot_theta += delta_theta;
    
    // 각도 정규화 (-PI ~ +PI)
    if (robot_theta > 3.141592f)  robot_theta -= 2.0f * 3.141592f;
    if (robot_theta < -3.141592f) robot_theta += 2.0f * 3.141592f;

	OS_EXIT_CRITICAL();

    // 2. [Twist] 실시간 속도 연산 (주기 100ms = 0.1s 이므로 0.1로 나눔)
    float current_linear_vel = delta_s / 0.1f;
    float current_angular_vel = delta_theta / 0.1f;

    // 3. [전송 패킷 패킹] 라즈베리파이로 보낼 데이터 최종 취합
	// 차동구동 로봇이므로 linear.y=0, linear.z=0, angular.x=0, angular.y=0
    tx_odom_packet.timestamp     = OSTimeGet(); // 또는 OSTimeGet()
    tx_odom_packet.pos_x         = robot_x;
    tx_odom_packet.pos_y         = robot_y;
    tx_odom_packet.theta         = robot_theta;
    tx_odom_packet.linear_vel_x  = current_linear_vel;
    tx_odom_packet.angular_vel_z = current_angular_vel;
}

static void UpdateWheelOdometry(void *p_arg)
{
	INT8U err;
	//INT32U last_wake_time;
	int32_t *pLeftDiff;

	int32_t left_encoder_delta;	// 100ms 동안의 엔코더 펄스 벡터
	int32_t right_encoder_delta;

	float left_wheel_turns;
	float right_wheel_turns;

	float left_wheel_delta;
	float right_wheel_delta;

	(void)p_arg;	

#if OS_CRITICAL_METHOD == 3
	OS_CPU_SR cpu_sr = 0;
#endif

	//last_wake_time = OSTimeGet();

	/* 100ms 주기로 휠 오도메트리 데이터 출력 */
	while (1) {
		// encoder_diff 변수 값을 받기 위해 waiting
		// 오른쪽 바퀴 pid제어 코드도 완성되면 구조체 포인터로 양쪽 diff 변수를 받아야 함
		pLeftDiff = (int32_t *)OSMboxPend(LeftEncoderMbox, 0, &err);

		if (err == OS_ERR_NONE && pLeftDiff != NULL) {
			left_encoder_delta = *pLeftDiff;

			left_wheel_turns = (float)left_encoder_delta / 2464.0f;
			left_wheel_delta = left_wheel_turns * 2.0f * 3.141592f * 0.065f;

			OS_ENTER_CRITICAL();
			left_wheel_distance += left_wheel_delta;
			OS_EXIT_CRITICAL();

			//update_robot_odometry(left_wheel_delta, right_wheel_delta);
		}

		// mailbox pend API가 blocking 해주므로 Delay가 필요없음
		//TimeDlyUntil(&last_wake_time, 100);
	}
}

void delay_us(uint32_t us)
{
	uint32_t start_val = TIM2->CNT;

	while ((uint16_t)(TIM2->CNT - start_val) < us);
}

#define FILTER_SIZE 5

float apply_median_filter(float new_val) {
    static float buf[FILTER_SIZE] = {0.0f};
    static int idx = 0;
    
    buf[idx] = new_val;
    idx = (idx + 1) % FILTER_SIZE;

    // 버퍼 복사 후 정렬
    float temp[FILTER_SIZE];
    for(int i = 0; i < FILTER_SIZE; i++) temp[i] = buf[i];

    // 단순 버블 정렬 (크기가 작으므로 충분히 빠름)
    for(int i = 0; i < FILTER_SIZE - 1; i++) {
        for(int j = 0; j < FILTER_SIZE - i - 1; j++) {
            if(temp[j] > temp[j+1]) {
                float t = temp[j];
                temp[j] = temp[j+1];
                temp[j+1] = t;
            }
        }
    }

    // 중앙값 반환
    return temp[FILTER_SIZE / 2];
}

void HCSR04_sensor_init(void)
{
	// trig pin setup
	gpio_set_mode(GPIOD, 11, GPIO_MODE_OUTPUT);
	gpio_set_ospeed(GPIOD, 11, GPIO_OSPEED_VH);
	gpio_write_pin(GPIOD, 11, 0);

	// echo pin setup. TIM4_CH1
	gpio_set_mode(GPIOD, 12, GPIO_MODE_AF);
	gpio_set_af(GPIOD, 12, TIM3_5_PIN_AF);

	timer_init(TIM4, 84 - 1, 0xFFFF);

	// echo 신호의 길이를 재기 위한 TIM4 PWM Input Mode 설정
	// TIM4의 채널1과 채널2를 모두 TI1(ch1핀)에 연결하여 같은 신호가 두 채널에 보이게 설정
	// 채널1 : rising edge에서 CNT 레지스터 초기화
	// 채널2 : falling edge에서 CNT 레지스터 값 캡처
	TIM4->CCMR1 &= ~((3 << 0) | (3 << 8));
	TIM4->CCMR1 |= (1 << 0) | (2 << 8);		// CC1S = 01, CC2S = 10

	// CC1E(0), CC1P(1), CC1NP(3), CC2E(4), CC2P(5), CC2NP(7) 비트 영역 초기화
	// 채널1을 rising edge를 감지하도록 설정, 채널2를 falling edge를 감지하도록 설정
	TIM4->CCER &= ~((1 << 0) | (1 << 1) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 7));
	TIM4->CCER |= (1 << 0) | (1 << 4) | (1 << 5);

	// 채널1의 rising edge 트리거가 발생했을 때 CNT를 초기화 하도록 설정
	TIM4->SMCR &= ~((7 << 0) | (7 << 4));
	TIM4->SMCR |= (4 << 0) | (5 << 4);		// SMS = 100, TS = 101

	// 채널2의 falling edge에서 인터럽트가 발생하도록 설정
	TIM4->DIER |= (1 << 2);		// CC2IE set

	// TIM4 인터럽트 활성화
	nvic_irq_enable(NVIC_TIM4_IRQN);
	nvic_irq_setprio(NVIC_TIM4_IRQN, 6);

	// TIM4 카운트 시작
	//TIM4->EGR |= (1 << 0);  // Update Generation (설정값 즉시 반영)
	//TIM4->SR  &= ~(1 << 0); // EGR로 인해 발생하는 UIF(Update Interrupt Flag) 클리어
	//timer_init(TIM4, 84 - 1, 0xFFFF);
	timer_start(TIM4);

	// us delay를 위한 타이머
	timer_init(TIM2, 84 - 1, 0xFFFF);
	timer_start(TIM2);
}

static void UltrasonicSensorTask(void *p_arg)
{
	INT8U err;
	INT32U last_wake_time;
	(void)p_arg;

	uint32_t duration;
	float distance_cm, filtered_distance;
	float prev_valid_dist = 0.0f;

#if OS_CRITICAL_METHOD == 3
	OS_CPU_SR cpu_sr = 0;
#endif

	last_wake_time = OSTimeGet();

	while (1) {
		// Mail box flush
		(void)OSMboxAccept(HCSR04DurationMbox);

		// Trig pin 활성화
		OS_ENTER_CRITICAL();

		gpio_write_pin(GPIOD, 11, 1);
		delay_us(10);
		gpio_write_pin(GPIOD, 11, 0);

		OS_EXIT_CRITICAL();

		// Echo pin 으로 들어오는 duration 측정
		duration = (uint32_t)OSMboxPend(HCSR04DurationMbox, 100, &err);

		if (err == OS_ERR_NONE) {
			distance_cm = (float)duration / 58.0f;

			if (distance_cm < 2.0f || distance_cm > 100.0f) {
				filtered_distance = prev_valid_dist;
			}
			else {
				filtered_distance = apply_median_filter(distance_cm);
				prev_valid_dist = distance_cm;
			}

			if (filtered_distance < 30.f) {
				led_on();
			}
			else {
				led_off();
			}

			//s_printf("%f, %f\r\n", filtered_distance, distance_cm);
		}
		else if (err == OS_ERR_TIMEOUT) {
			s_printf("No Detected\r\n");
		}

		TimeDlyUntil(&last_wake_time, 200);
	}
}

// 1초 이상 신호를 주어야 소프트웨어 리셋이 걸리도록 구현
static void OTATrigTask(void *p_arg)
{
	INT8U err;
	(void)p_arg;

	while (1) {
		OSSemPend(ButtonSem, 0, &err);

    	OSTimeDly(1000);

    	if (gpio_read_pin(GPIOE, 4) == 0)
    	{
       		SoftwareReset();
    	}
	}
}


static void AppTaskComm(void *p_arg)
{
    //INT32U last_wake_time;
    (void)p_arg;
	//rx_count = 0;
    //last_wake_time = OSTimeGet();

    while (1) {
		//s_printf("rpm : %f\r\n", current_rpm);
		//s_printf("distance : %f\r\n", left_wheel_distance);

		//s_printf("target rpm : %d\r\n", target_rpm);
		if (uart_idle_flag) {
			uart_idle_flag = 0;

			s_printf("linear_x : %f\r\nangular_z : %f\r\n\r\n", linear_x, angular_z);
		}
		//s_printf("%d\r\n", rx_count);
		//s_printf("&rx_count = %08X\r\n", (unsigned int)&rx_count);

		//OSTimeDly(1000);
    }
}