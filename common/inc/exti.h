#ifndef __EXTI_H__
#define __EXTI_H__

#include <stdint.h>

typedef struct EXTI_TYPEDEF {
	uint32_t IMR;
	uint32_t EMR;
	uint32_t RTSR;
	uint32_t FTSR;
	uint32_t SWIER;
	uint32_t PR;
} EXTI_TypeDef;

#define EXTI_BASE  0x40013c00
#define EXTI ((EXTI_TypeDef *)EXTI_BASE)

#define EXTI_CR1 (*(volatile uint32_t *)0x40013808)	// EXTI0 ~ EXTI3
#define EXTI_CR2 (*(volatile uint32_t *)0x4001380c)	// EXTI4 ~ EXTI7
#define EXTI_CR3 (*(volatile uint32_t *)0x40013810)	// EXTI8 ~ EXTI11
#define EXTI_CR4 (*(volatile uint32_t *)0x40013814)	// EXTI12 ~ EXTI15

#define EXTI_CR_A 0
#define EXTI_CR_B 1
#define EXTI_CR_C 2
#define EXTI_CR_D 3
#define EXTI_CR_E 4
#define EXTI_CR_F 5
#define EXTI_CR_G 6
#define EXTI_CR_H 7
#define EXTI_CR_I 8

#endif
