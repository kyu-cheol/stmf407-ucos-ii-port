#ifndef __UART_H__
#define __UART_H_

#include <stdint.h>

typedef struct UART_X {
	uint32_t SR;
	uint32_t DR;
	uint32_t BRR;
	uint32_t CR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t GTPR;
} UART_x;

#define UART3_BASE 0x40004800

#define UART3 ((UART_x *)UART3_BASE)

#define UART1_PIN_AF (7)
#define UART2_PIN_AF (7)
#define UART3_PIN_AF (7)

#define UART4_PIN_AF (8)
#define UART5_PIN_AF (8)
#define UART6_PIN_AF (8)

void uart_init(UART_x *uart, uint32_t bitrate);	// "bitrate-8-N-1" 고정
void uart_write(UART_x *uart, const char *data);
void uart_deinit(UART_x *uart);

#endif
