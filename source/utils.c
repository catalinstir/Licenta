#include "utils.h"

void delay(uint32_t cycles) {
    volatile uint32_t i = 0U;

    for (i = 0U; i < cycles; ++i) {
        __asm("NOP");
    }
}

void print_float(float num) {

    float intpart;
    float frac = modff(num, &intpart);
    int integer_part = (int)intpart;
    int decimal_part = (int)(fabsf(frac) * 1000.0f);

    if (num < 0.0f && integer_part == 0) {
        PRINTF("-0.%03d", decimal_part);
    } else if (num < 0.0f) {
        PRINTF("-%d.%03d", abs(integer_part), decimal_part);
    } else {
        PRINTF("%d.%03d", integer_part, decimal_part);
    }
}
