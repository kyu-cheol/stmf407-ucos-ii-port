#include <stdint.h>
#include "uart.h"
#include "gpio.h"
#include "system.h"
#include "rcc.h"

#define UART_CR1_UART_ENABLE    (1 << 13)
#define UART_CR1_WORD_LEN       (1 << 12)
#define UART_CR1_PARITY_ENABLED (1 << 10)
#define UART_CR1_PARITY_ODD     (1 << 9)
#define UART_CR1_TX_ENABLE      (1 << 3)
#define UART_CR1_RX_ENABLE      (1 << 2)
#define UART_CR2_STOPBITS       (3 << 12)
#define UART_SR_TXE             (1 << 7)
#define UART_SR_RXNE            (1 << 5)

// PB10, PB11
#define UART3_TX_PIN (10)
#define UART3_RX_PIN (11)

// TX : PB10
// RX : PB11
static void uart3_pin_setup(void)
{
	uint32_t reg;

	RCC_AHB1_CLOCK_ER |= GPIOB_AHB1_CLOCK_ER_VAL;

	// set alternate function mode
	reg = GPIOB->MODE & ~(0x03 << (UART3_RX_PIN * 2));
	GPIOB->MODE = reg | 0x02 << (UART3_RX_PIN * 2);

	reg = GPIOB->MODE & ~(0x03 << (UART3_TX_PIN * 2));
	GPIOB->MODE = reg | 0x02 << (UART3_TX_PIN * 2);

	// select AF7(USART1..3)
	reg = GPIOB->AFRH & ~(0X0F << ((UART3_RX_PIN - 8) * 4));
	GPIOB->AFRH = reg | (UART3_PIN_AF << ((UART3_RX_PIN - 8) * 4));

	reg = GPIOB->AFRH & ~(0X0F << ((UART3_TX_PIN - 8) * 4));
	GPIOB->AFRH = reg | (UART3_PIN_AF << ((UART3_TX_PIN - 8) * 4));
}

void uart_init(UART_x *uart, uint32_t bitrate)
{
	uart3_pin_setup();

	RCC_APB1_CLOCK_ER |= UART3_APB1_CLOCK_ER_VAL;

	uart->CR1 |= UART_CR1_TX_ENABLE;		// trnasmit enable
	uart->BRR = PCLK1 / bitrate;			// bitrate setting
	uart->CR1 &= ~UART_CR1_WORD_LEN;		// select 8bits len
	uart->CR1 &= ~UART_CR1_PARITY_ENABLED;	// parity disable
	uart->CR2 &= ~UART_CR2_STOPBITS;		// select 1 stop bit
	uart->CR1 |= UART_CR1_UART_ENABLE;		// uart enable
}

void uart_write(UART_x *uart, const char *data)
{
	const char *p = data;
	volatile uint32_t reg;

	while (*p) {
		do {
			reg = uart->SR;
		} while ((reg & UART_SR_TXE) == 0);

		uart->DR = *p;
		p++;
	}
}

void uart_deinit(UART_x *uart)
{
	;
}

/* printf stub function */
int _write(int fd, char *ptr, int len)
{
	for (int i = 0; i < len; ++i) {
		while (!(UART3->SR & UART_SR_TXE));

		UART3->DR = (uint8_t)ptr[i];
	}

	return len;
}