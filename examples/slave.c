/*
 *
 * Modbus test and study for RPi/BPi and UART RS485 module
 *
 * SLAVE side
 *
 */
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <modbus.h>
#include <wiringPi.h>

#define SLAVE_ID            13
#define REG_COUNT_TO_QUERY  5

static int control_pin = -1;
static int rts_debug = 0;

void bpi_rs485_select_control_pin(int phys_pin_num, int debug)
{
    /* this function will select specified physical pin as 
       control pin for DE/~RE terminals of RS485 module and 
       set its initial state to '0', thus putting RS485 module
       into receive mode.
       NOTE: here pin is the physical pin number of the GPIO
       connector */
    wiringPiSetupPhys();
    control_pin = phys_pin_num;
    pinMode(control_pin, OUTPUT);
    digitalWrite(control_pin, 0);
    rts_debug = debug;
}

void bpi_rs485_custom_rts(modbus_t *ctx, int on)
{
    if (control_pin <= 0) {
        fprintf(stderr, "bpi_rs485_custom_rts: invalid control pin number %d\n", control_pin);
        exit(1);
    }
    if (on) {
        /* set control pin state to transmit */
        digitalWrite(control_pin, 1);
    }
    else {
        /* set control pin stat to receive */
        digitalWrite(control_pin, 0);
    }
    if (rts_debug) {
        fprintf(stderr, "\tbpi_rs485_custom_rts: setting RTS pin to %d\n", on);
    }
}

int main(void)
{
    modbus_t *ctx;

    /* nb_bits, nb_input_bits, nb_registers, nb_input_registers */
    modbus_mapping_t *mapping = modbus_mapping_new(0, 1, 30, 2);
    if (!mapping) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n", modbus_strerror(errno));
        exit(1);
    }

    mapping->tab_registers[10] = 10;
    mapping->tab_registers[11] = 11;
    mapping->tab_registers[12] = 12;
    /* Example: set register 13 to integer value 1308 */
    mapping->tab_registers[13] = 1308;
    mapping->tab_registers[14] = 14;

    bpi_rs485_select_control_pin(40, 1); /* use physical pin #40 as DE/~RE control pin */

    /* BPI M2U specifics - UART on GPIO pins 8,10 is accessible via /dev/ttyS2 */
    ctx = modbus_new_rtu("/dev/ttyS2", 57600, 'N', 8, 1);
    if (ctx == NULL) {
        fprintf(stderr, "Unable to create the libmodbus context\n");
        return -1;
    }
    modbus_set_debug(ctx, TRUE);

    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    if (modbus_rtu_set_custom_rts(ctx, &bpi_rs485_custom_rts) == -1) {
        fprintf(stderr, "RTU set custom RTS handler failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
    
    if (modbus_rtu_set_rts(ctx, MODBUS_RTU_RTS_UP) == -1) {
        fprintf(stderr, "RTU set initial RTS state failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
    
    if (modbus_flush(ctx) == -1) {
        fprintf(stderr, "Initial flush failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    fprintf(stdout, "Modbus RTU initialized.\n");

    /* this slave will run at ID = SLAVE_ID */
    modbus_set_slave(ctx, SLAVE_ID);

    uint8_t req[MODBUS_RTU_MAX_ADU_LENGTH]; /* request buffer */
    int len; /* length of the request/response */

    while(1) {
        len = modbus_receive(ctx, req);
        if (len == -1) break;

        len = modbus_reply(ctx, req, len, mapping);
        if (len == -1) break;
    }
    printf("Exit the loop: %s\n", modbus_strerror(errno));

    modbus_mapping_free(mapping);

    modbus_close(ctx);
    modbus_free(ctx);
    return 0;
}