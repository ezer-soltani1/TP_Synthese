#include "leds.h"
#include "spi.h"
#include "gpio.h"
#include <stdio.h>
#include <stdlib.h>

LED_Driver_t led_driver;

// MCP23S17 Registers and Opcodes
#define MCP_IODIRA    0x00
#define MCP_IODIRB    0x01
#define MCP_GPIOA     0x12
#define MCP_GPIOB     0x13
#define MCP_OLATA     0x14
#define MCP_OLATB     0x15
#define MCP_OPCODE_WRITE 0x40
#define MCP_OPCODE_READ  0x41

// Shadow registers to keep track of LED states
// Default to 0xFF (all LEDs OFF, assuming active-low)
static uint8_t shadow_gpio_a = 0x00;
static uint8_t shadow_gpio_b = 0x00;

static void MCP23S17_Write(uint8_t reg, uint8_t value) {
    uint8_t data[3];
    data[0] = MCP_OPCODE_WRITE;
    data[1] = reg;
    data[2] = value;

    HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, data, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);
}

static uint8_t MCP23S17_Read(uint8_t reg) {
    uint8_t txData[3];
    uint8_t rxData[3];

    txData[0] = MCP_OPCODE_READ;
    txData[1] = reg;
    txData[2] = 0x00; // Dummy byte

    HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi3, txData, rxData, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(VU_nCS_GPIO_Port, VU_nCS_Pin, GPIO_PIN_SET);

    return rxData[2];
}

static void MCP23S17_Init(void) {
    // Hardware Reset
    HAL_GPIO_WritePin(VU_nRESET_GPIO_Port, VU_nRESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(VU_nRESET_GPIO_Port, VU_nRESET_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // Configure Port A and B as outputs (IODIR = 0x00)
    MCP23S17_Write(MCP_IODIRA, 0x00);
    MCP23S17_Write(MCP_IODIRB, 0x00);

    // Set all LEDs OFF by default (active-low) using shadow registers
    shadow_gpio_a = 0xFF;
    shadow_gpio_b = 0xFF;
    MCP23S17_Write(MCP_GPIOA, shadow_gpio_a);
    MCP23S17_Write(MCP_GPIOB, shadow_gpio_b);
}

static void Test_First_LED(void) {
    // Test GPIOA Pin 0
    // Note: This test function bypasses shadow register update for simplicity in test,
    // or it should update them. Let's just blink it.
    MCP23S17_Write(MCP_GPIOA, 0xFE); // LED 0 ON (Active Low)
    HAL_Delay(500);
    MCP23S17_Write(MCP_GPIOA, 0xFF); // LED 0 OFF
}

static void LED_Chenillard(void) {
    // Run through GPIOA
    for (int i = 0; i < 8; i++) {
        MCP23S17_Write(MCP_GPIOA, ~(1 << i));
        HAL_Delay(100);
    }
    MCP23S17_Write(MCP_GPIOA, 0xFF);
    
    // Run through GPIOB
    for (int i = 0; i < 8; i++) {
        MCP23S17_Write(MCP_GPIOB, ~(1 << i));
        HAL_Delay(100);
    }
    MCP23S17_Write(MCP_GPIOB, 0xFF);

    // Restore state from shadow registers after animation
    MCP23S17_Write(MCP_GPIOA, shadow_gpio_a);
    MCP23S17_Write(MCP_GPIOB, shadow_gpio_b);
}

static void Blink_All_LEDs(void) {
    for(int i=0; i<3; i++) {
        MCP23S17_Write(MCP_GPIOA, 0x00); // All ON
        MCP23S17_Write(MCP_GPIOB, 0x00);
        HAL_Delay(200);
        MCP23S17_Write(MCP_GPIOA, 0xFF); // All OFF
        MCP23S17_Write(MCP_GPIOB, 0xFF);
        HAL_Delay(200);
    }
    // Restore state from shadow registers
    MCP23S17_Write(MCP_GPIOA, shadow_gpio_a);
    MCP23S17_Write(MCP_GPIOB, shadow_gpio_b);
}

int shell_control_led(h_shell_t *h_shell, int argc, char **argv) {
    if (argc != 4) {
        int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "Usage: l <port> <pin> <state>\r\n");
        h_shell->drv.transmit(h_shell->print_buffer, size);
        return -1;
    }

    char port = argv[1][0];
    int pin = atoi(argv[2]);
    int state = atoi(argv[3]);

    if ((port != 'A' && port != 'B') || pin < 0 || pin > 7 || (state != 0 && state != 1)) {
        int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "Invalid arguments\r\n");
        h_shell->drv.transmit(h_shell->print_buffer, size);
        return -1;
    }

    uint8_t *shadow_reg;
    uint8_t reg_addr;

    if (port == 'A') {
        shadow_reg = &shadow_gpio_a;
        reg_addr = MCP_GPIOA;
    } else {
        shadow_reg = &shadow_gpio_b;
        reg_addr = MCP_GPIOB;
    }

    // Update shadow register
    if (state == 1) {
        // ON -> Active Low -> Bit = 0
        *shadow_reg &= ~(1 << pin);
    } else {
        // OFF -> Active Low -> Bit = 1
        *shadow_reg |= (1 << pin);
    }

    // Write to hardware
    if(led_driver.write) {
        led_driver.write(reg_addr, *shadow_reg);
    }

    int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "LED %c%d -> %s\r\n", port, pin, state ? "ON" : "OFF");
h_shell->drv.transmit(h_shell->print_buffer, size);

    return 0;
}

int shell_chenillard(h_shell_t *h_shell, int argc, char **argv) {
    int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "Running Chenillard...\r\n");
    h_shell->drv.transmit(h_shell->print_buffer, size);
    
    if (led_driver.chenillard) {
        led_driver.chenillard();
    }
    return 0;
}

int shell_blink_all(h_shell_t *h_shell, int argc, char **argv) {
    int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "Blinking All LEDs...\r\n");
    h_shell->drv.transmit(h_shell->print_buffer, size);

    if (led_driver.blink_all) {
        led_driver.blink_all();
    }
    return 0;
}

void LED_Driver_Init(LED_Driver_t *driver) {
    driver->init = MCP23S17_Init;
    driver->write = MCP23S17_Write;
    driver->read = MCP23S17_Read;
    driver->test_first_led = Test_First_LED;
    driver->chenillard = LED_Chenillard;
    driver->blink_all = Blink_All_LEDs;

    // Perform hardware initialization
    driver->init();
}
