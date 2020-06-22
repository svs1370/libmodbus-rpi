/*
 *
 * Modbus test and study for RPi/BPi and UART RS485 module
 *
 * MASTER side
 *
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <math.h>

#include <modbus.h>
#include <wiringPi.h>

#include <pcf8574.h>  
#include <lcd.h>  

// #define GRAPH_DEBUG  1

#define SLAVE_TO_QUERY      13
#define REG_ADDR_TO_QUERY 4
#define REG_COUNT_TO_QUERY  2

static int baud_rate = 115200; // 9600, 57600, 115200

static int control_pin = -1;
static int rts_debug = 0;

#define TOTAL_GLYPHS 8           // Use all 8 custom chars of LCD display
#define TOTAL_READINGS 40        // Readings are 8 chars * 5 dots per char = 40

unsigned char *glyphs[TOTAL_GLYPHS];      // we will build custom chars in this array
float readings[TOTAL_READINGS];  // this will hold 40 temperature readings
unsigned char stored[TOTAL_READINGS];     // this will indicate if corresponding readings[i] is filled with actual data
int marker, update_counter;
const float TBASE_OFFSET=7.0;
float tDisplay, tCur, tMaxAbs, tMinAbs, tMaxGraph, tMinGraph, graphDivider; //, graphBias, tBaseGraph, ;

#ifdef GRAPH_DEBUG
    #define SLEEP_TIME 2
    #define UPDATE_CYCLE 1          // update graph each 1 cycles, i.e. 1 * 2 sec = 2 sec
    #define DEBUG_READINGS 50
    float debug_dataset[DEBUG_READINGS] = {
        22.0, 23.0, 25.3, 27.0, 27.0, // 0
        28.5, 27.3, 27.0, 26.8, 26.0, // 1
        25.0, 23.0, 22.0, 22.0, 21.0, // 2
        19.0, 16.5, 15.1, 14.8, 14.0, // 3
        13.0, 13.0, 13.0, 10.1, 10.5, // 4
        10.0, 10.5, 10.5, 11.0, 12.0, // 5
        12.5, 12.7, 13.3, 14.1, 15.2, // 6
        16.5, 18.0, 19.8, 20.3, 22.0, // 7
        22.8, 23.5, 25.0, 25.5, 25.5, // 8
        22.0, 23.0, 25.3, 27.0, 27.0, // 9
    };
#else
    #define SLEEP_TIME 20
    // 36 cycles = 12 minutes, 105 cycles = 35 minutes
    // 12 min update gives 8 hrs display, 35 minutes - approx. 24 hrs
    #define UPDATE_CYCLE 105  // update graph each UPDATE_CYCLES cycles, each is SLEEP_TIME
#endif

void bpi_rs485_select_control_pin(int pin_num, int debug)
{
    /* this function will select specified physical pin as 
       control pin for DE/~RE terminals of RS485 module and 
       set its initial state to '0', thus putting RS485 module
       into receive mode.
       NOTE: pin num is interpreted according to the method
       tha is called to initialize wiringPi, e.g.
       - if wiringPiSetupPhys is called, than physical pin # is expected (i.e. 40) 
       - if wiringPiSetup is called, than wiring Pi pin # is expected (i.e. 29) */
    
    control_pin = pin_num;
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

int my_lcd_setup()
{
    int display_id;

    for(int i=0; i<TOTAL_GLYPHS; i++) {
        glyphs[i] = (unsigned char *) malloc(8*sizeof(unsigned char));
        if(glyphs[i] == NULL) {
            printf("my_lcd_setup: failed to allocate memory for custom glyph %d. Exiting.\n", i);
        exit(1);
        }
        memset(glyphs[i], 0, 8);
    }

    pinMode(8,I2C_PIN); // 8
	pinMode(9,I2C_PIN); // 9
		
    if ( pcf8574Setup (100, 0x27) != TRUE ) 
    {
        printf("my_lcd_setup: pcf8574 setup failed. Exiting.\n");
        exit(1);
    }

    // printf("pcf8574Setup - done\n");
    
    for(int i=0;i<8;i++)  // pcf8574 has 8 GPIO lines
        pinMode(100+i,OUTPUT);  
    
    digitalWrite(103,1); /* BL - backlit or blink? */ 
    digitalWrite(101,0); /* RW - set to write */ 
    
    display_id=lcdInit(2,16,4,100,102,104,105,106,107,0,0,0,0);  
    lcdHome(display_id);  
    lcdClear(display_id);

    lcdPosition(display_id, 0, 0);

    // blank all custom chars inside the LCD before proceeding
    for(int pos=0; pos<TOTAL_GLYPHS; pos++) {
        lcdCharDef(display_id, pos, glyphs[pos]);
    }

    return display_id;
}

void build_glyph(int dfd, int glyphnum, int curmark)
{
    int index=0, top_offset=0;

    if( glyphnum < 0 || glyphnum >= TOTAL_GLYPHS) return; 

    memset(glyphs[glyphnum], 0, 8); // blank the selected glyph template

#ifdef GRAPH_DEBUG
    fprintf(stdout, "\tbuild_glyph: num %d: ", glyphnum);
#endif

    for(int pos=0; pos<5; pos++) {
        // index = curmark + glyphnum*5 + pos + 1;
        index = curmark + 1 + glyphnum*5 + pos;
        if( index >= TOTAL_READINGS ) {
            index -= TOTAL_READINGS;
        }
        if( stored[index] == 0x00 ) {
#ifdef GRAPH_DEBUG
            fprintf(stdout, "[%2d]        ; ", index);
#endif            
            continue;
        }
        // new algorithm - top of display (tBaseGraph) is always plus 7 degrees from tCur
        // top_offset = (int)(round((tBaseGraph - readings[index])/graphDivider)); 
        top_offset = (int)(round((tMaxGraph - readings[index])/graphDivider)); // old algorithm - min/max
#ifdef GRAPH_DEBUG
        fprintf(stdout, "[%2d] %2.1f %2d; ", index, readings[index], top_offset);
#endif        
        if (top_offset < 0 ) top_offset=0;
        else if (top_offset > 7) top_offset=7;
        glyphs[glyphnum][top_offset] |= 0x01 << (4-pos);
        lcdCharDef(dfd, glyphnum, glyphs[glyphnum]);
    }

#ifdef GRAPH_DEBUG
    fprintf(stdout, "\n");
#endif

    return;
}

void redraw_graph(int dfd, int curmark) 
{
    if( curmark < 0 || curmark >= TOTAL_READINGS) return;

#ifdef GRAPH_DEBUG
    fprintf(stdout, "\nredraw_graph: mark=%d tCur=%2.1f tMinGraph=%2.1f tMaxGraph=%2.1f\n", 
           curmark, tCur, tMinGraph, tMaxGraph);
#endif

    for(int i=0; i<TOTAL_GLYPHS; i++) {
        build_glyph(dfd, i, curmark);
        lcdPosition(dfd, 8+i, 1);
        lcdPutchar(dfd, i);
    }

    return;
}

int main(void)
{
    int display, numread;
    unsigned int errCount=0;
    unsigned int update_limits = 0x0;
    

    time_t timep;  
    struct tm *ptm;  

    modbus_t *ctx;

    // wiringPiSetupPhys();
    wiringPiSetup();

    display = my_lcd_setup();

    /* use wiring Pi pin # 29 when in wiringPiSetup mode, which is phys pin #40  */
    bpi_rs485_select_control_pin(29, 1); /* use physical pin #40 as DE/~RE control pin when in wiringPiSetupPhys mode */

    /* BPI M2U specifics - UART on GPIO pins 8,10 is accessible via /dev/ttyS2 */
    ctx = modbus_new_rtu("/dev/ttyS2", baud_rate, 'N', 8, 1);
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

    modbus_set_response_timeout(ctx, 1, 0); // set to 1 sec 0 microsec
    
    if (modbus_flush(ctx) == -1) {
        fprintf(stderr, "Initial flush failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    fprintf(stdout, "Modbus RTU initialized.\n");

    modbus_set_slave(ctx, SLAVE_TO_QUERY);

#ifdef GRAPH_DEBUG
    fprintf(stdout, "Modbus Master - in GRAPH_DEBUG mode\n");
#else
    uint16_t read_registers[REG_COUNT_TO_QUERY]; /* make enough */
    /* we will be reading two holding registers
     - the 1st (at index 0) will hold temperature in engineering notation: 227 means +22,7C
     - the 2nd (at index 1) will hold request counter */
    /* Read REG_COUNT_TO_QUERY holding registers starting from address REG_ADDR_TO_QUERY */

    fprintf(stdout, "Modbus Master - query Slave ID %d\n", SLAVE_TO_QUERY);
    fprintf(stdout, "\tBaud rate: %d\n", baud_rate);
    fprintf(stdout, "\tQuery type: read registers. Address %d, num of registers %d\n\n", REG_ADDR_TO_QUERY, REG_COUNT_TO_QUERY);
#endif

    lcdPosition(display, 11, 0);
    lcdPrintf(display, "/");
    lcdPosition(display, 6, 1);
    lcdPrintf(display,"%d", errCount);

    tCur = 0.0;
    tDisplay = 0.0;
    tMinAbs = 99.0;
    tMaxAbs = -99.0;

    tMinGraph = 99.0;
    tMaxGraph = -99.0;
    // graphBias = 4.0;
    // graphDivider = 1.0;
    // tBaseGraph = tCur + graphBias;
    
    marker=(TOTAL_READINGS-1);
    update_counter=UPDATE_CYCLE;

    
#ifdef GRAPH_DEBUG
    for(int debug_runner=0; debug_runner<DEBUG_READINGS; debug_runner++) {
        numread = REG_COUNT_TO_QUERY;
#else
    while(1) {
        numread = modbus_read_registers(ctx, REG_ADDR_TO_QUERY, REG_COUNT_TO_QUERY, read_registers);
#endif

        if (numread != REG_COUNT_TO_QUERY) {
            fprintf(stderr, "Failed to read: %s\n", modbus_strerror(errno));
            errCount++;
            lcdPosition(display, 6, 1);
            lcdPrintf(display, "%d", errCount);
        }
        else {
#ifdef GRAPH_DEBUG
            tCur = debug_dataset[debug_runner];
#else
            tCur = ((float)read_registers[0])/10.0;
#endif

            if ( tCur > tMaxAbs ) {
                tMaxAbs = tCur;
                lcdPosition(display, 12, 0);
                lcdPrintf(display, "%2.1f", tMaxAbs);
            }
            if ( tCur < tMinAbs ) {
                tMinAbs = tCur;
                lcdPosition(display, 7, 0);
                lcdPrintf(display, "%2.1f", tMinAbs);
            }
            

            if ( tDisplay != tCur ) {
                // update current temperature display if temperature changed
                lcdPosition(display, 0, 0);
                lcdPrintf(display, "%2.1f", tCur);
                tDisplay = tCur;
            }

            update_counter++;
            if ( update_counter >= UPDATE_CYCLE ) {
                // rebuild custom characters and redraw graph
                readings[marker] = tDisplay;
                stored[marker] = 0x01;
                // recalculate min/max stored readings
                update_limits = 0x00;
                for(int pos=0; pos<TOTAL_READINGS; pos++) {
                    if (stored[pos] == 0x00) continue;
                    if (readings[pos] > tMaxGraph) {
                        tMaxGraph = readings[pos];
                        update_limits = 0x01;
                    }
                    if (readings[pos] < tMinGraph) {
                        tMinGraph = readings[pos];
                        update_limits = 0x01;
                    }
                }
                if ( update_limits > 0 ) {
                    // rescale mapping of stored range to the LCD capabilities
                    if ( tMaxGraph - tMinGraph < 7) {
                        graphDivider = 1.;
                        // graphBias = 4.;
                    }
                    else {
                        graphDivider = ceil((tMaxGraph - tMinGraph)/7);
                        // graphBias = (tMaxGraph - tMinGraph)/2.;
                    }
                    update_limits = 0x00;
                }
                // tBaseGraph = tCur + graphBias;
                redraw_graph(display, marker);
                marker++;
                if ( marker >= TOTAL_READINGS ) marker=0;
                update_counter = 0;
            }
        }

        // update time display
        time(&timep);  
        ptm=localtime(&timep);  
        lcdPosition(display, 0, 1);  
        lcdPrintf(display,"%02d:%02d",ptm->tm_hour, ptm->tm_min);  

        sleep(SLEEP_TIME);
    }
    
    modbus_close(ctx);
    modbus_free(ctx);
    return 0;
}
