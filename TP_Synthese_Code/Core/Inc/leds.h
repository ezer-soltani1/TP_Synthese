#ifndef INC_LEDS_H_
#define INC_LEDS_H_

#include "main.h"
#include "shell.h"

typedef struct {
    void (*init)(void);
    void (*write)(uint8_t reg, uint8_t value);
    uint8_t (*read)(uint8_t reg);
    void (*test_first_led)(void);
    void (*chenillard)(void);
    void (*blink_all)(void);
} LED_Driver_t;

extern LED_Driver_t led_driver;

void LED_Driver_Init(LED_Driver_t *driver);
int shell_control_led(h_shell_t *h_shell, int argc, char **argv);
int shell_chenillard(h_shell_t *h_shell, int argc, char **argv);
int shell_blink_all(h_shell_t *h_shell, int argc, char **argv);

#endif
