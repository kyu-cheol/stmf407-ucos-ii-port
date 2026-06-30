#include "includes.h"
#include "app_util.h"

__attribute__((aligned(8)))
static OS_STK AppTaskStartStk[APP_CFG_STARTUP_TASK_STK_SIZE];

__attribute__((aligned(8)))
static OS_STK AppTaskGuidanceStk[APP_CFG_GUIDANCE_TASK_STK_SIZE];

__attribute__((aligned(8)))
static OS_STK AppTaskImuStk[APP_CFG_IMU_TASK_STK_SIZE];

__attribute__((aligned(8)))
static OS_STK AppTaskCommStk[APP_CFG_COMM_TASK_STK_SIZE];

static void AppTaskStart(void *p_arg);
static void AppTaskGuidance(void *p_arg);
static void AppTaskImu(void *p_arg);
static void AppTaskComm(void *p_arg);

OS_TMR *my_timer;
OS_EVENT *UartMutex = NULL;
OS_FLAG_GRP *MissileEventFlags;

// Missile 발사 준비 완료 flag
#define FLAG_LAUNCH_READY (FLAG_IMU_OK | FLAG_COMM_OK | FLAG_PROP_OK)

void BSP_Init(void)
{	
	led_setup();
	button_setup();
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
	MissileEventFlags = OSFlagCreate(0x00, &err);

	/* Missile 유도 제어 Task */
	OSTaskCreate((void (*)(void *)) AppTaskGuidance, \
				 (void *) 0, \
				 (OS_STK *) &AppTaskGuidanceStk[APP_CFG_GUIDANCE_TASK_STK_SIZE - 1], \
				 (INT8U) APP_CFG_GUIDANCE_TASK_PRIO);

	/* IMU 센서 처리 Task  */
	OSTaskCreate((void (*)(void *)) AppTaskImu, \
				 (void *) 0, \
				 (OS_STK *) &AppTaskImuStk[APP_CFG_IMU_TASK_STK_SIZE - 1], \
				 (INT8U) APP_CFG_IMU_TASK_PRIO);

	/* 지상 통제소와의 통신 담당 Task */
    OSTaskCreate((void (*)(void *)) AppTaskComm, \
				 (void *) 0, \
				 (OS_STK *) &AppTaskCommStk[APP_CFG_COMM_TASK_STK_SIZE - 1], \
				 (INT8U) APP_CFG_COMM_TASK_PRIO);
				 
	s_printf("uC/OS-II RTOS Initialization complete. Deleting Start Task...\r\n");
	OSTaskDel(OS_PRIO_SELF);
}

void MyTimerCallback(void *p_tmr, void *p_arg) {
	led_toggle();
}

static void AppTaskGuidance(void *p_arg)
{
	INT8U err;
	INT32U last_wake_time;

	my_timer = OSTmrCreate(
        0,                      // 초기 지연 시간 (0이면 즉시 시작)
        1000,                   // 주기 (OS_TMR_CFG_TICKS_PER_SEC이 10일 때, 10은 1초를 의미)
        OS_TMR_OPT_PERIODIC,    // 주기적으로 반복 실행 옵션
        MyTimerCallback,        // 만료 시 호출할 함수 등록
        (void *)0,              // 콜백 함수에 넘겨줄 인자
        (INT8U *)"MyTimer",     // 타이머 이름
        &err                    // 에러 코드 반환 변수
    );

	if (err == OS_ERR_NONE) {
		OSTmrStart(my_timer, &err);
	}

	s_printf("[Guidance] 비행 승인 이벤트를 대기합니다...\r\n");

	// IMU 센서, 지상 통제소와의 통신, 추진체 압력 상태가 모두 정상 상태가 될때까지 blocking
	OSFlagPend(
		MissileEventFlags,
		FLAG_LAUNCH_READY,
		OS_FLAG_WAIT_SET_ALL | OS_FLAG_CONSUME,
		0,
		&err
	);

	if (err == OS_ERR_NONE) {
		s_printf("[Guidance] 비행 승인 완료. 유도 알고리즘 가동.\r\n");
	}
	else {
		ASSERT();
	}

	last_wake_time = OSTimeGet();

	/* 10ms 주기로 미사일의 유도 알고리즘 수행 */
	while (1) {
		s_printf("[Guidance]\r\n");

		/*
		 *  미사일의 여러 센서 데이터들을 토대로 복잡한 유도 알고리즘 수행
		 *  10ms의 주기로 현재 상태를 갱신해가며 목표 상태에 도달하도록 제어 수행
		 */

		TimeDlyUntil(&last_wake_time, 10);
	}
}

static void AppTaskImu(void *p_arg)
{
	INT8U err;
	INT32U last_wake_time;
	(void)p_arg;

	volatile float a = 1.23, b = 4.87, result;

	s_printf("[IMU] 자이로/가속도 센서 Calibration Start...\r\n");
	OSTimeDlyHMSM(0, 0, 3, 0);
	s_printf("[IMU] 센서 데이터 갱신 정상 확인.\r\n");

	OSFlagPost(MissileEventFlags, FLAG_IMU_OK, OS_FLAG_SET, &err);

	last_wake_time = OSTimeGet();

	/* 10ms 주기로 IMU 센서 데이터 필터링 처리 */
	while (1) {
		s_printf("[IMU]\r\n");
		result = a * b;
		s_printf("result : %f\r\n", result);

		/* 
		 *  자이로/가속도 센서 값을 DMA가 SPI로 부터 읽어와 전역 버퍼에 기록하고 있다고 가정
		 *  해당 전역 버퍼의 데이터를 칼만, LPF 같은 필터 연산을 통해 노이즈 제거
		 *  노이즈 제거된 최종 센서 데이터를 AppTaskGuidance 태스크에서 사용할 수 있도록 버퍼에 저장
		 */

		TimeDlyUntil(&last_wake_time, 100);
	}
}

static void AppTaskComm(void *p_arg)
{
	INT8U err;
	(void)p_arg;

	OSTimeDlyHMSM(0, 0, 1, 500);
	s_printf("[COMM] 지상 통제소 비행 명령 수신 완료.\r\n");

	OSFlagPost(MissileEventFlags, FLAG_COMM_OK, OS_FLAG_SET, &err);

	/* 100ms 주기로 지상 통제소 <==> 미사일 통신 */
	while (1) {
		s_printf("[COMM]\r\n");

		/* 
		 *  미사일의 현재 위치, 고도, 속도를 지상 통제소로 송신
		 *  지상 통제소에서 보내는 비행 유도 중지, 자폭 등의 명령들을 수신 후 해당 명령에 맞는 동작 수행
		 */

		OSTimeDly(1000);
	}
}