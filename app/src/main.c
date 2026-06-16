#include "includes.h"
#include "nvic.h"

#define  APP_TASK_START_PRIO        5
#define  APP_TASK_START_STK_SIZE  128
__attribute__((aligned(8))) static OS_STK AppTaskStartStk[APP_TASK_START_STK_SIZE];

#define APP_TASK1_PRIO	6
#define APP_TASK1_STK_SIZE 128
__attribute__((aligned(8))) static OS_STK AppTask1Stk[APP_TASK1_STK_SIZE];

#define APP_TASK2_PRIO	7
#define APP_TASK2_STK_SIZE 128
__attribute__((aligned(8))) static OS_STK AppTask2Stk[APP_TASK2_STK_SIZE];

static void AppTaskStart(void *p_arg);
static void AppTask1(void *p_arg);
static void AppTask2(void *p_arg);

OS_TMR *my_timer;
uint32_t test;

void BSP_Init(void) {	
	led_setup();
	button_setup();
}

void main(void) {
	
	//SystemInit();

	OSInit();

	OSTaskCreate((void (*)(void *)) AppTaskStart, \
				 (void *) 0, \
				 (OS_STK *) &AppTaskStartStk[APP_TASK_START_STK_SIZE - 1], \
				 (INT8U) APP_TASK_START_PRIO);


	OSStart();
}

static void AppTaskStart(void *p_arg)
{
	(void)p_arg;

	printf("AppTaskStart\r\n");

	// 해당 태스크에서 사용되는 peripherals init
	BSP_Init();

	//Print_All_Tasks_Info();

	// Systick 인터럽트 최하위 우선순위 부여후 실행
	// OSStart 이후에 호출되어야 함.
	OS_CPU_SysTickInitFreq(CPU_FREQ);

	OSTaskCreate((void (*)(void *)) AppTask1, \
				 (void *) 0, \
				 (OS_STK *) &AppTask1Stk[APP_TASK1_STK_SIZE - 1], \
				 (INT8U) APP_TASK1_PRIO);

	OSTaskCreate((void (*)(void *)) AppTask2, \
				 (void *) 0, \
				 (OS_STK *) &AppTask2Stk[APP_TASK2_STK_SIZE - 1], \
				 (INT8U) APP_TASK2_PRIO);

	while (1) {
		OSTimeDlyHMSM(0, 0, 5, 0);
	}
}

void MyTimerCallback(void *p_tmr, void *p_arg) {
	led_toggle();
}

void AppTask1(void *p_arg)
{
	INT8U err;

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

	while (1) {
		printf("[TASK1]\r\n");

		OSTimeDlyHMSM(0, 0, 1, 0);
	}
}

void AppTask2(void *p_arg)
{
	(void)p_arg;

	while (1) {
		printf("[TASK2]\r\n");

		OSTimeDlyHMSM(0, 0, 1, 0);
	}
}

// Context Switching 발생할 때 마다 call 되는 hook function 
void OSTaskSwHook(void)
{
	//__asm__ volatile ("BKPT");
	//while (1);
}

void Print_All_Tasks_Info(void)
{
    OS_TCB  *p_tcb;
    uint32_t task_count = 0;

    // 1. 순회하는 도중 문맥 전환이 일어나지 않도록 스케줄러 잠금
    OSSchedLock();

    printf("\r\n--- Current Active Tasks List ---\r\n");

    // 2. uC/OS-II 커널의 TCB 시작점부터 탐색 시작
    p_tcb = OSTCBList; 

    while (p_tcb != (OS_TCB *)0) {
        task_count++;
        
        // 태스크의 우선순위(Prio) 출력
        printf("Task [%ld] - Priority: %d", task_count, p_tcb->OSTCBPrio);
        
        // 만약 특정 시스템 태스크이거나 사용자 태스크인 경우 구별 팁
        if (p_tcb->OSTCBPrio == OS_TASK_IDLE_PRIO) {
            printf(" (System Idle Task)\r\n");
        } else if (p_tcb->OSTCBPrio == 61) { // 질문자님의 타이머 태스크 우선순위
            printf(" (System Timer Task)\r\n");
        } else if (p_tcb->OSTCBPrio == 5) {
            printf(" (AppTaskStart)\r\n");
        } else {
            printf(" (User Task)\r\n");
        }

        // 다음 태스크로 이동
        p_tcb = p_tcb->OSTCBNext;
    }

    printf("Total Active Tasks Count: %ld\r\n", task_count);
    printf("---------------------------------\r\n");

    // 3. 조회가 끝났으므로 스케줄러 잠금 해제
    OSSchedUnlock();
}