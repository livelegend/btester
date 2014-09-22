#include "stm32f30x.h"
#include "gpio.h"

void gpio_config(GPIO_TypeDef * gpio, uint8_t n, uint8_t mode, uint8_t otype, uint8_t ospeed, uint8_t pull, uint8_t af) {
    
    gpio->MODER = (gpio->MODER & (~(3<<(n*2)))) | (mode<<(n*2));
    if (mode==MODE_GP ||  mode==MODE_AF) {
        gpio->OTYPER  = (gpio->OTYPER  & (~(1<<n)))     | (otype<<n);
        gpio->OSPEEDR = (gpio->OSPEEDR & (~(3<<(n*2)))) | (ospeed<<(n*2));
    }
    gpio->PUPDR = (gpio->PUPDR & (~(3<<(n*2)))) | (pull<<(n*2));
    if (mode==MODE_AF) {
        gpio->AFR[n>>3] = (gpio->AFR[n>>3] & (~(0xF<<((n&7)*4)))) | (af<<((n&7)*4));
    }
    
}
