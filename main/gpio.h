#ifndef __GPIO__H__
#define __GPIO__H__

#define MODE_IN   0
#define MODE_GP   1
#define MODE_AF   2
#define MODE_AN   3

#define OTYPE_PP  0
#define OTYPE_OD  1

#define OSPEED_2  0
#define OSPEED_10 1
#define OSPEED_50 3

#define PULL_NO   0
#define PULL_UP   1
#define PULL_DOWN 2

#define AF_NO     0
#define AF0       0
#define AF1       1
#define AF2       2
#define AF3       3
#define AF4       4
#define AF5       5
#define AF6       6
#define AF7       7
#define AF8       8
#define AF9       9
#define AF10      10
#define AF11      11
#define AF12      12
#define AF13      13
#define AF14      14
#define AF15      15

void gpio_config(GPIO_TypeDef *, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

#endif
