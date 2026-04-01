#include "ultrasonic_sensor.h"

unsigned int Switch_Status(void) {
    if (0 == GPIO_PinRead(BOARD_SWITCH_GPIO, BOARD_SWITCH_GPIO_PIN))
        return 0;
    else
        return 1;
}

unsigned int USonic_GetDistance(void) {
    unsigned int t;
    unsigned int echo;
    unsigned int timeout = 0;
    const unsigned int MAX_TIMEOUT = 100000;
    PRINTF("CITESTE DISTANTA\r\n");
    GPIO_PortToggle(BOARD_TRIG_GPIO, 1u << BOARD_TRIG_GPIO_PIN);

    delay(170);

    GPIO_PortToggle(BOARD_TRIG_GPIO, 1u << BOARD_TRIG_GPIO_PIN);
    timeout = 0;
    do {
        echo = GPIO_PinRead(BOARD_ECHO_GPIO, BOARD_ECHO_GPIO_PIN);
        timeout++;
        if (timeout > MAX_TIMEOUT) {
            PRINTF("TIMEOUT 1: Nu a început ECHO\r\n");
            return 999;
        }

    } while (0 == echo);

    t = 0;
    timeout = 0;
    do {
        echo = GPIO_PinRead(BOARD_ECHO_GPIO, BOARD_ECHO_GPIO_PIN);
        t++;
        timeout++;
        if (timeout > MAX_TIMEOUT) {
            PRINTF("TIMEOUT 2: ECHO prea lung\r\n");
            return 999;
        }
    } while (1 == echo);

    return 44 * t / 10000; // distance(cm) = 343 * t * 256 / 2 / 1000000000;
}

bool verificaObstacol(void) {
    unsigned int d = USonic_GetDistance();
    PRINTF("\r\ndistance=%d \r\n", d);
    return (d < 30);
}
