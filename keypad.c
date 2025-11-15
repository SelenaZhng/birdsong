#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"

// VGA graphics library
#include "vga16_graphics_v2.h"
#include "pt_cornell_rp2040_v1_4.h"

// Low-level alarm infrastructure we'll be using
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

//Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0  // 2^32 (a constant)

#define DELAY 20 // 1/Fs (in microseconds)

#define Fs 50000

// the DDS units - core 0
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (400.0*two32)/Fs ;

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size] ;

// Values output to DAC
int DAC_output_0 ;
int DAC_output_1 ;

// Amplitude modulation parameters and variables
fix15 max_amplitude = int2fix15(1) ;    // maximum amplitude
fix15 attack_inc ;                      // rate at which sound ramps up
fix15 decay_inc ;                       // rate at which sound ramps down
fix15 current_amplitude_0 = 0 ;         // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0 ;         // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME             1000
#define DECAY_TIME              1000
#define SUSTAIN_TIME            10000
#define BEEP_DURATION           10500
#define BEEP_REPEAT_INTERVAL    50000

// State machine variables
volatile unsigned int STATE_0 = 0 ;
volatile unsigned int count_0 = 0 ;

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

// Keypad pin configurations
#define BASE_KEYPAD_PIN 9
#define KEYROWS         4
#define NUMKEYS         12

#define LED             25

unsigned int keycodes[12] = {   0x28, 0x11, 0x21, 0x41, 0x12,
                                0x22, 0x42, 0x14, 0x24, 0x44,
                                0x18, 0x48} ;
unsigned int scancodes[4] = {   0x01, 0x02, 0x04, 0x08} ;
unsigned int button = 0x70 ;


char keytext[40];
int prev_key = 0;

//SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define LED      25
#define SPI_PORT spi0

//GPIO for timing the ISR
#define ISR_GPIO 2

#define NOT_PRESSED 0
#define MAYBE_PRESSED 1
#define MAYBE_NOT_PRESSED 2
#define PRESSED 3

volatile int state = NOT_PRESSED;

// Swoop
#define FREQ_B 1740
#define FREQ_K 260
#define SWOOP_N 6500
volatile int swoop_active = 0;
volatile int swoop_index = 0;
static uint32_t swoop_incr[SWOOP_N];

// Chirp
#define CHIRP_B 2000
#define CHIRP_K 0.000184f
#define CHIRP_N 6500
volatile int chirp_active = 0;
volatile int chirp_index = 0;
static uint32_t chirp_incr[CHIRP_N];

// Record
volatile int record_active = 0;
static int record_sequence[100] = {0};
volatile int index_count = 0;

// Play
volatile int play_active = 0;
volatile int play_index = 0;

// Silence 
volatile int silent_active = 0;
volatile int silence_duration_counter = 0;

// This timer ISR is called on core 0
static void alarm_irq(void) {

    // Assert a GPIO when we enter the interruptf
    gpio_put(ISR_GPIO, 1) ;

    // Clear the alarm irq
    hw_clear_bits(&timer_hw->intr, 1u << ALARM_NUM);

    // Reset the alarm register
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

    // static int prev_pressed = 0; 
    if (swoop_active) {

        // DDS phase and sine table lookup
        phase_accum_main_0 += swoop_incr[swoop_index]  ;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0>>24])) + 2048 ;
       // printf("%d/n", phase_accum_main_0);
        
        // Ramp up amplitude
        if (swoop_index < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
        }
        // Ramp down amplitude
        else if (swoop_index > SWOOP_N - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        swoop_index +=1;
        count_0 += 1 ;

        if (swoop_index >= SWOOP_N) {
            swoop_active = 0;
            swoop_index = 0;
            // count_0 = 0;
            current_amplitude_0 = 0;
        }   
    } else {
        // current_amplitude_0 = 0;
        // count_0 = 0;
        DAC_output_0 = 2048;
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0x0FFF));
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);
    }
    
    if (chirp_active){
        // DDS phase and sine table lookup
        phase_accum_main_0 += chirp_incr[chirp_index]  ;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
        sin_table[phase_accum_main_0>>24])) + 2048 ;
            
        // Ramp up amplitude
        if (chirp_index < ATTACK_TIME) {
            current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
        }
        // Ramp down amplitude
        else if (chirp_index > CHIRP_N - DECAY_TIME) {
            current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        chirp_index +=1;
        count_0 += 1 ;

        if (chirp_index >= CHIRP_N) {
            chirp_active = 0;
            chirp_index = 0;
            // count_0 = 0;
            current_amplitude_0 = 0;
        } 
    } else {
        // silence when neither is active
        DAC_output_0 = 2048;
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0x0FFF));
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);
    }

    if (silent_active) {
        silence_duration_counter += DELAY;

        if (silence_duration_counter >= 130000) { 
            silent_active = 0;
            silence_duration_counter = 0;
        }a

        DAC_output_0 = 2048;
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0x0FFF));
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1);
    }
    
    // De-assert the GPIO when we leave the interrupt
    gpio_put(ISR_GPIO, 0);
}

// This thread runs on core 0
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;

    // Some variables
    static int i ;
    static uint32_t keypad ;

    while(1) {

        gpio_put(LED, !gpio_get(LED)) ;

        // Scan the keypad!
        for (i=0; i<KEYROWS; i++) {
            // Set a row high
            gpio_put_masked((0xF << BASE_KEYPAD_PIN),
                            (scancodes[i] << BASE_KEYPAD_PIN)) ;
            // Small delay required
            sleep_us(1) ; 
            // Read the keycode
            keypad = ((gpio_get_all() >> BASE_KEYPAD_PIN) & 0x7F) ;
            // Break if button(s) are pressed
            if (keypad & button) break ;
        }
        // If we found a button . . .
        if (keypad & button) {
            // Look for a valid keycode.
            for (i=0; i<NUMKEYS; i++) {
                if (keypad == keycodes[i]) break ;
            }
            // If we don't find one, report invalid keycode
            if (i==NUMKEYS) (i = -1) ;
        }
        // Otherwise, indicate invalid/non-pressed buttons
        else (i=-1) ;

        static int possible = -1;
        
        switch (state) {
            case NOT_PRESSED:   
                if (i != -1){
                    possible = i;
                    state = MAYBE_PRESSED;
                } else {
                    state = NOT_PRESSED;
                }
               break;
            case MAYBE_PRESSED:
                if (i == possible){
                    state = PRESSED;

                    if (possible == 1){ //key 1
                        chirp_active = 0;
                        swoop_active = 1;
                        swoop_index = 0;
                        current_amplitude_0 = 0;
                        phase_accum_main_0 = 0;
                        silent_active =0;
                        if (record_active && index_count <100) {
                            record_sequence[index_count] = 1;
                            index_count++;
                        } else if (index_count == 100) {
                            index_count = 0;
                            record_active = 0;
                        }
                    } else if (possible == 2){ //key 2
                        swoop_active = 0;
                        chirp_active = 1;
                        chirp_index  = 0;
                        phase_accum_main_0 = 0;
                        current_amplitude_0 = 0;
                        silent_active =0;
                        if (record_active && index_count < 100) {
                            record_sequence[index_count] = 2;
                            index_count++;
                        } else if (index_count == 100) {
                            index_count = 0;
                            record_active = 0;
                        }
                    } else if (possible == 3) {
                        silent_active = 1;
                        if (record_active && index_count < 100) {
                            record_sequence[index_count] = 3;
                            index_count++;
                        } else if (index_count == 100) {
                            index_count = 0;
                            record_active = 0;
                        }
                    } else if (possible == 10) { // record button (*)
                        record_active = !record_active;
                        if (record_active) {
                            memset(record_sequence, 0, sizeof record_sequence);
                            index_count = 0;
                        }
                    } else if (possible == 11) { // play button (#)
                       if (index_count > 0){
                        play_active = 1;
                        play_index = 0;
                        record_active = 0;
                       }
                    }
                } else {
                    state = NOT_PRESSED;
                }
                break;
            case MAYBE_NOT_PRESSED:
                if (i == possible) {
                    state = PRESSED;
                } else {
                    state = NOT_PRESSED;
                }
                break;
            case PRESSED:
                // make it beep
                if (i != possible){
                    state = MAYBE_NOT_PRESSED;
                } else {
                    state = PRESSED;
                }
                break;
        }
        
        // play mode FSM
        if (play_active) {
            if (!swoop_active && !chirp_active && !silent_active) {
                if (play_index < index_count) {
                    int code = record_sequence[play_index++];

                    phase_accum_main_0 = 0;
                    current_amplitude_0 = 0;

                    if (code == 1) {
                        chirp_active = 0;
                        swoop_active = 1;
                        swoop_index  = 0;
                    }
                    else if (code == 2) {
                        swoop_active = 0;
                        chirp_active = 1;
                        chirp_index  = 0;
                    }
                    else if (code == 3) {
                        silent_active = 1;
                        silence_duration_counter = 0;
                    }
                }
                else {
                    play_active = 0;
                    play_index  = 0;
                }
            }
        }

        static int last_idx = -1;
        if (record_active && index_count != last_idx){ 
            last_idx = index_count;
        }

        static int last_rec = -1;  // remember previous value

        if (record_active != last_rec) {
            last_rec = record_active;   // update memory
        }

        static int last_play_active = -1;
        static int last_play_index  = -1;

        if (play_active != last_play_active || play_index != last_play_index) {
            last_play_active = play_active;
            last_play_index  = play_index;
        }

        
        // Write key to VGA
        if (i != prev_key) {
            prev_key = i ;
            fillRect(250, 20, 176, 30, RED); // red box
            sprintf(keytext, "%d", i) ;
            setCursor(250, 20) ;
            setTextSize(2) ;
            writeString(keytext) ;
        }

        PT_YIELD_usec(30000) ;
    }
    // Indicate thread end
    PT_END(pt) ;
}

int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Initialize the VGA screen
    initVGA() ;

    // Draw some filled rectangles
    fillRect(64, 0, 176, 50, BLUE); // blue box
    fillRect(250, 0, 176, 50, RED); // red box
    fillRect(435, 0, 176, 50, GREEN); // green box

    // Write some text
    setTextColor(WHITE) ;
    setCursor(65, 0) ;
    setTextSize(1) ;
    writeString("Raspberry Pi Pico") ;
    setCursor(65, 10) ;
    writeString("Keypad demo") ;
    setCursor(65, 20) ;
    writeString("Hunter Adams") ;
    setCursor(65, 30) ;
    writeString("vha3@cornell.edu") ;
    setCursor(250, 0) ;
    setTextSize(2) ;
    writeString("Key Pressed:") ;

     // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC) ;
    gpio_set_dir(LDAC, GPIO_OUT) ;
    gpio_put(LDAC, 0) ;

    // Setup the ISR-timing GPIO
    gpio_init(ISR_GPIO) ;
    gpio_set_dir(ISR_GPIO, GPIO_OUT);
    gpio_put(ISR_GPIO, 0) ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;


    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME)) ;
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME)) ;

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = float2fix15(2047*sin((float)ii*6.283/(float)sine_table_size));
    }

    for (int n = 0; n < SWOOP_N; n++) {
        float swoop_freq = FREQ_B + FREQ_K * sin((M_PI * n) / SWOOP_N );
        swoop_incr[n] = (uint32_t)( swoop_freq * two32 / Fs );
    }
    
    for (int n = 0; n < CHIRP_N; n++) {
        float chirp_freq = ((0.000184)*(n*n)) + 2000;
        chirp_incr[n] = (uint32_t)( chirp_freq * two32 / Fs );
    }


    // Enable the interrupt for the alarm (we're using Alarm 0)
    hw_set_bits(&timer_hw->inte, 1u << ALARM_NUM) ;
    // Associate an interrupt handler with the ALARM_IRQ
    irq_set_exclusive_handler(ALARM_IRQ, alarm_irq) ;
    // Enable the alarm interrupt
    irq_set_enabled(ALARM_IRQ, true) ;
    // Write the lower 32 bits of the target time to the alarm register, arming it.
    timer_hw->alarm[ALARM_NUM] = timer_hw->timerawl + DELAY ;

    ////////////////// KEYPAD INITS ///////////////////////
    // Initialize the keypad GPIO's
    gpio_init_mask((0x7F << BASE_KEYPAD_PIN)) ;
    // Set row-pins to output
    gpio_set_dir_out_masked((0xF << BASE_KEYPAD_PIN)) ;
    // Set all output pins to low
    gpio_put_masked((0xF << BASE_KEYPAD_PIN), (0x0 << BASE_KEYPAD_PIN)) ;
    // Turn on pulldown resistors for column pins (on by default)
    gpio_pull_down((BASE_KEYPAD_PIN + 4)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 5)) ;
    gpio_pull_down((BASE_KEYPAD_PIN + 6)) ;

    // Add core 0 threads
    pt_add_thread(protothread_core_0) ;

    // Start scheduling core 0 threads
    pt_schedule_start ;

}
