#include "gpio.h"
#include "rcc.h"

void gpio_init(void)
{
    RCC_AHB1_CLOCK_ER |= (GPIOA_AHB1_CLOCK_ER_VAL | GPIOB_AHB1_CLOCK_ER_VAL | \
                          GPIOC_AHB1_CLOCK_ER_VAL | GPIOD_AHB1_CLOCK_ER_VAL | \
                          GPIOE_AHB1_CLOCK_ER_VAL | GPIOF_AHB1_CLOCK_ER_VAL | \
                          GPIOG_AHB1_CLOCK_ER_VAL | GPIOH_AHB1_CLOCK_ER_VAL | \
                          GPIOI_AHB1_CLOCK_ER_VAL);
    
    // RCC 레지스터 동기화
    // 해당 레지스터 값을 load 하여 peripheral write 동작이 완료되었음을 보장
    (void)RCC_AHB1_CLOCK_ER; 
}

void gpio_wrtie_pin(GPIO_Port *gpiox, uint8_t pin, uint8_t pin_state)
{
    if (pin_state == HIGH) {
        gpiox->BSRR = (1 << pin);
    }
    else {
        gpiox->BSRR = (1 << (pin + 16));
    }
}

uint8_t gpio_read_pin(GPIO_Port *gpiox, uint8_t pin)
{
    return (uint8_t)((gpiox->IDR >> pin) & 1);
}

void gpio_set_mode(GPIO_Port *gpiox, uint8_t pin, uint8_t mode)
{
    uint32_t reg = gpiox->MODE;
    reg &= ~(0x03 << (pin * 2));
    reg |= (mode << (pin * 2));
    gpiox->MODE = reg;
}

void gpio_set_ospeed(GPIO_Port *gpiox, uint8_t pin, uint8_t ospeed)
{
    uint32_t reg = gpiox->OSPEED;
    reg &= ~(0x03 << (pin * 2));
    reg |= (ospeed << (pin * 2));
    gpiox->OSPEED = reg;
}

void gpio_set_pupd(GPIO_Port *gpiox, uint8_t pin, uint8_t pupd)
{
    uint32_t reg = gpiox->PUPD;
    reg &= ~(0x03 << (pin * 2));
    reg |= (pupd << (pin * 2));
    gpiox->PUPD = reg;
}

void gpio_set_af(GPIO_Port *gpiox, uint8_t pin, uint8_t af_num)
{
    uint32_t reg;

    if (pin < 8) {
        reg = gpiox->AFRL & (~(0x0f << (pin * 4)));
        gpiox->AFRL = reg | (af_num << (pin * 4));
    }
    else {
        reg = gpiox->AFRH & (~(0x0f << ((pin - 8) * 4)));
        gpiox->AFRH = reg | (af_num << ((pin - 8) * 4));
    }
}