#include <stdint.h>
#include "timer.h"
#include "exti.h"
#include "spi.h"
#include "dma.h"
#include "includes.h"

uint8_t button_flag;
uint8_t target_rpm_update_flag;
uint8_t uart_idle_flag;

// volatile uint32_t enc_pulse_edge_cnt = 0;
// volatile int32_t enc_pos_cnt = 0;
volatile int32_t target_rpm = 0;

extern void (*spi_rx_callback)(SPI_x *spi, uint8_t data);
extern void (*spi_ovr_callback)(SPI_x *spi, uint8_t data);

extern OS_EVENT *HCSR04DurationMbox;
extern OS_EVENT *ButtonSem;

#define ENCODER_A_INPUT_GPIO_Port GPIOC
#define ENCODER_A_INPUT_Pin 7

#define ENCODER_B_INPUT_GPIO_Port GPIOC
#define ENCODER_B_INPUT_Pin 9

extern uint8_t g_cmd_vel_buffer[8];
float linear_x, angular_z;
volatile uint32_t rx_count = 0;

void isr_tim4(void)
{
	OSIntEnter();

	if (TIM4->SR & (1 << 2)) {
		TIM4->SR &= ~(1 << 2);

		OSMboxPost(HCSR04DurationMbox, (void *)TIM4->CCR2);
	}

	OSIntExit();
}

void isr_exti4(void)
{
	INT8U err;

	OSIntEnter();

	EXTI->PR |= 1 << 4;
	button_flag = 1;

	OSSemPost(ButtonSem);

	//OSFlagPost(MissileEventFlags, FLAG_PROP_OK, OS_FLAG_SET, &err);
	//SoftwareReset();

	OSIntExit();
}


// void isr_exti9_5(void)
// {
// 	OSIntEnter();

// 	// Encoder A Channel EXTI interrupt (PC7)
// 	if (EXTI->PR & (1 << 7)) {
// 		EXTI->PR |= 1 << 7;
// 		//printf("7 Pin Occured\r\n");

// 		enc_pulse_edge_cnt++;

// 		// A channel rising edge
// 		if (gpio_read_pin(ENCODER_A_INPUT_GPIO_Port, ENCODER_A_INPUT_Pin) == 1) {
// 			if (gpio_read_pin(ENCODER_B_INPUT_GPIO_Port, ENCODER_B_INPUT_Pin) == 1) {
// 				enc_pos_cnt++;
// 			}
// 			else {
// 				enc_pos_cnt--;
// 			}
// 		}
// 		else {		// A channel falling edge
// 			if (gpio_read_pin(ENCODER_B_INPUT_GPIO_Port, ENCODER_B_INPUT_Pin) == 0) {
// 				enc_pos_cnt++;
// 			}
// 			else {
// 				enc_pos_cnt--;
// 			}
// 		}
// 	}

// 	// Encoder B Channel EXTI interrupt (PC9)
// 	if (EXTI->PR & (1 << 9)) {
// 		EXTI->PR |= 1 << 9;
// 		//printf("9 Pin Occured\r\n");

// 		enc_pulse_edge_cnt++;

// 		if (gpio_read_pin(ENCODER_B_INPUT_GPIO_Port, ENCODER_B_INPUT_Pin) == 1) {
// 			if (gpio_read_pin(ENCODER_A_INPUT_GPIO_Port, ENCODER_A_INPUT_Pin) == 0) {
// 				enc_pos_cnt++;
// 			}
// 			else {
// 				enc_pos_cnt--;
// 			}
// 		}
// 		else {
// 			if (gpio_read_pin(ENCODER_A_INPUT_GPIO_Port, ENCODER_A_INPUT_Pin) == 1) {
// 				enc_pos_cnt++;
// 			}
// 			else {
// 				enc_pos_cnt--;
// 			}
// 		}
// 	}

// 	OSIntExit();
// }

void isr_spi1(void)
{
	uint32_t sr = SPI1->SR;
	uint8_t recv_data;

	OSIntEnter();

	if (sr & SPI_SR_RX_NOTEMPTY) {
		recv_data = (uint8_t)SPI1->DR;
		
		if (spi_rx_callback) {
			spi_rx_callback(SPI1, recv_data);
		}
	}

	if (sr & SPI_SR_OVR) {
		recv_data = (uint8_t)SPI1->DR;
		(void)SPI1->SR;

		if (spi_ovr_callback) {
			spi_ovr_callback(SPI1, recv_data);
		}
	}

	OSIntExit();
}

void isr_uart3(void)
{
    volatile uint32_t reg;

    OSIntEnter();

    reg = UART3->SR;

	// IDLE 인터럽트
	if (reg & UART_SR_IDLE && UART3->CR1 & UART_CR1_IDLEIE) {
		// IDLE 인터럽트 플래그 clear
		volatile uint32_t dummy = UART3->SR;
		dummy = UART3->DR;
		(void)dummy;

		uint16_t current_ndtr = DMA1->S[1].NDTR;
		uint16_t packet_size = sizeof(g_cmd_vel_buffer);

		uart_idle_flag = 1;

		// DMA의 circular 설정으로 NDTR 레지스터가 reload 되어 있을 수 있음.
		if (current_ndtr == 0 || current_ndtr == packet_size) {
			// 데이터 파싱
			//parse_cmd_vel_packet(g_cmd_vel_buffer);
			memcpy(&linear_x, &g_cmd_vel_buffer[0], sizeof(float));
			memcpy(&angular_z, &g_cmd_vel_buffer[4], sizeof(float));

			rx_count++;
		}
		else {
			// 데이터 유실이 발생한 경우 DMA 카운터 리셋
			DMA1->S[1].CR &= ~DMA_SxCR_EN;
			while (DMA1->S[1].CR & DMA_SxCR_EN);

			DMA1->S[1].NDTR = packet_size;

			DMA1->S[1].CR |= DMA_SxCR_EN;
		}
	}

	// 송신 인터럽트
    if (reg & UART_SR_TXE) {
        ;
    }

    OSIntExit();
}