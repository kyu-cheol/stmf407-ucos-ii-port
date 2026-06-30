#include <stdint.h>
#include "timer.h"
#include "exti.h"
#include "spi.h"
#include "includes.h"

//volatile uint32_t ticks;
uint8_t timer_flag;
uint8_t button_flag;

extern void (*spi_rx_callback)(SPI_x *spi, uint8_t data);
extern void (*spi_ovr_callback)(SPI_x *spi, uint8_t data);

void isr_tim2(void)
{
	static volatile uint32_t tim2_cnt = 0;

	TIM2_UI_FLAG_CLEAR();

	tim2_cnt++;
	timer_flag = 1;
}

void isr_exti4(void)
{
	INT8U err;

	OSIntEnter();

	EXTI->PR |= 1 << 4;
	button_flag = 1;

	OSFlagPost(MissileEventFlags, FLAG_PROP_OK, OS_FLAG_SET, &err);

	OSIntExit();
}

void isr_spi1(void)
{
	uint32_t sr = SPI1->SR;
	uint8_t recv_data;

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
}