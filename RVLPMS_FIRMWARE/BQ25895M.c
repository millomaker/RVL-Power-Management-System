/*
 * File:   main.c
 * Author: Gunnar
 *
 * Created on September 13, 2019, 9:32 PM
 */

#include <xc.h>
#include <stdint.h>
#include <pic16f15324.h>
#include "PICCONFIG.h"
#include "PWM.h"
#include "PPS.h"
#include "I2C.h"
#include "BQ25895M.h"
#include "ADC.h"
#include "LED_INTERFACE.h"
#include "time.h"

void PIC_SETUP(){
    
    CPUDOZEbits.IDLEN = 0;      //clear idle mode
    VREGCON = 0b00000011;       //low power sleep mode
    
    //IO setup
    TRISA = 0xFF;
    TRISC = 0xFF;
    ANSELA = 0;
    ANSELC = 0;
    ANSELAbits.ANSA4 = 1;   //thermistor
    WPUA = 0b00100000;      //WPU on RA5 BTN
    
    //TIMER1 setup
    T1CONbits.CKPS = 0b00;  //1:1 prescale
    T1CONbits.nSYNC = 0;
    T1CONbits.RD16 = 1;     //16-bit read
    T1GCONbits.GE = 0;      //Gate OFF
    T1CLK = 0b00000100;     //CLK is LFINTOSC
    TMR1 = TMR1_RST;  
    PIE4bits.TMR1IE = 1;    //enable tmr1 interrupt     
    T1CONbits.ON = 1;
    
    //IOC setup
    PIE0bits.IOCIE = 1;     //enable IOC module
    IOCAP = 0b00100000;     //PSEDGE enable bits
    IOCAN = 0b00100000;     //NEGEDGE enable bits
    IOCAF = 0x00;
    
    INTCONbits.GIE = 1;     //enable active interrupts
    INTCONbits.PEIE = 1;    //enable peripheral interrupts
}

uint8_t SYS_ENABLE = 0;         //regulator enable state
uint8_t FiveVolt_ENABLE = 0;    //BengeAdd; 5V regulator state (when SYS_ENABLE is off)
uint32_t btn_time_start = 0;    //timer for debouncing button
uint8_t pwr_btn_temp = 0;
uint8_t pwr_btn_temp_prev = 0;
uint8_t btn_state = 0;  //0: not pressed 1: being debounced 2: pressed
uint32_t btn_time_pressed = 0;  //timer for how long button has been held
uint8_t btn_high_edge = 1;      //goes high when button is released
uint8_t btn_press_count = 0;    //how many times button has been pressed in short time interval
uint32_t btn_press_timer = 0;   //timer for determining double press
uint8_t btn_long_edge = 0;  //goes high when btn is long press, resets when btn is released

void interrupt ISR(){   //Benge : Interrupt Service Routine

    if(TMR1IF) {    //Benge : Timer 1 Interrupt Flag bit, interrupt when overflow http://ww1.microchip.com/downloads/en/devicedoc/31012a.pdf 
        TMR1IF = 0;
        TMR1 = TMR1_RST;
        timer_counter++;
    }
    
    if(IOCAF5) {
        IOCAF5 = 0;
    }

    if(IOCAF1) {    //BengeAdd, If RA1 triggered the interrupt (INT pin on the BQ25895)
        IOCAF1 = 0;
    }
} 

void thermal_protection(){    
    //calculating setpoint = 255 / [(Therm_resistance(at temp) / 10,000 ) + 1]
    //For temp = 75C, setpoint = 222
    if(readADC(ADCRA4) >= 222) {
        SYS_ENABLE = 0;
        TRISCbits.TRISC5 = 1;       //turn off regulators
        TRISAbits.TRISA2 = 1;       //BengeAdd turn off 5V regulator
        BQ_Write(0x09, 0b01100100); //Force BATFET off 
    }    
}
/*
void ps2_on() {
    //turn on ps2
    __delay_ms(1200);
    TRISAbits.TRISA2 = 0;   //ps2 reset output
    aux = 0;                //ps2 reset low
    __delay_ms(200);
    TRISAbits.TRISA2 = 1;   //ps2 reset float 
}
*/

void main() {
            
    PIC_SETUP();
    
    //clear i2c bus if SDA held low
    I2C_bus_reset();
    
    //Initialize I2C Master
    PPS_unlock();
    SSP1DATPPS = 0x11;  //SDA INPUT
    RC1PPS = 0x16;      //SDA OUTPUT
    SSP1CLKPPS = 0x10;  //SDA INPUT
    RC0PPS = 0x15;      //SCL OUTPUT
    PPS_lock();
    I2C_Master_Init(350000);   
    
    BQ_CONFIG_INIT();
    BQ_INIT();
    
    PWM_INIT();

    while(1) {

        CLRWDT();   //Benge : Clear Watchdog Timer
        
        //debouncing the power button
        pwr_btn_temp = pwr_btn;
        if(pwr_btn_temp ^ pwr_btn_temp_prev) {  //if curr/prev values are not equal, then it is not debounced
            btn_time_start = get_time();
            btn_state = 1;
        }  
        if(timer_diff(btn_time_start) > 4) {
            if(!pwr_btn_temp) { //btn is pressed, on first edge, grab the time
                if(btn_state != 2) btn_time_pressed = get_time();
                btn_state = 2;      
            }
            else {   //btn is not pressed
                btn_state = 0;
            }
        }
        pwr_btn_temp_prev = pwr_btn_temp;
             
        //power_button state machine
        if(btn_state == 2) {
            if(timer_diff(btn_time_pressed) > 84 && btn_long_edge == 0) {
                SYS_ENABLE = !SYS_ENABLE;
                if(SYS_ENABLE) {
                    TRISCbits.TRISC5 = 0;           //Benge : this sets the PORTC,5 pin as output; https://www.quora.com/What-is-the-difference-between-PORTCbits-RC7-0-and-TRISCbits-RC7-0-in-PIC18F4550
                    TRISAbits.TRISA2 = 0;           //BengeAdd; ; Turn ON 5V reg
                    enable = 1;                     //turn on regulators   
                    enable5v = 1;                   //BengeAdd
                }
                else {
                    TRISCbits.TRISC5 = 1;           //turn off regulators; Benge : this sets the PORTC,5 pin as input
                    TRISAbits.TRISA2 = 1;           //BengeAdd; Turn Off 5V reg
                    if(mode == 3) {                 //shipping mode
                        BQ_Write(0x09, 0b01100100); //Force BATFET off 
                    }
                }
                btn_long_edge = 1;
            }
            btn_high_edge = 0;
        }
        if(btn_state == 0) {
            if(btn_high_edge == 0) {
                if(SYS_ENABLE && timer_diff(btn_time_pressed)<=50){ //short press
                    if(btn_press_count == 0) btn_press_timer = get_time();
                    btn_press_count++;
                    if(btn_press_count == 2) {  //double press
                        if(!VBUS_CHRG_STATE[1]) mode++; //only increment mode when not charging
                    }                    
                }
            }
            btn_high_edge = 1;
            btn_long_edge = 0;
        }
        if(timer_diff(btn_press_timer) > 50) btn_press_count = 0;   //reset double press counter
        
        BQ_UPDATE();
        thermal_protection();
        
        //WORKING :) ; but the LED doesn't power up after powering up the PMS when charging.
        //but if I unplug the led power up.
        if (!SYS_ENABLE) {
            if (VBUSV_STATE[1]>=0x12 && VBUSV_STATE[1]<=0x1A) { //BengeAdd; If sys not enable & Vbus >= 4.4V & Vbus <= 5.3V (connected to USB on a computer); Don't forget the 2.6V Offset
                TRISAbits.TRISA2 = 0;
                enable5v = 1;
                FiveVolt_ENABLE=!FiveVolt_ENABLE;
            }
            else if (FiveVolt_ENABLE && VBUSV_STATE[1]<0x12) { //BengeAdd; If 5V is enable and VBUS < 4.4V
                TRISAbits.TRISA2 = 1;
                FiveVolt_ENABLE=!FiveVolt_ENABLE;
            }
        }
        
        //if battery is low, revert to mode 2 to warn user
        //.02V/bit, 2.304V offset. bit value = [(desired cutoff voltage) - 2.304] / .02V/bit
        if(BATTERY_VOLTAGE <= 50) mode = 2;
        
        //setting the led interface
        if(VBUS_CHRG_STATE[1] == 0) {
            if(SYS_ENABLE) {
                led_modes();
            }
        }
        else {
            chrg_led();
        }
        
        //power consumption putting pic to sleep
        if(!FiveVolt_ENABLE && !SYS_ENABLE && VBUS_CHRG_STATE[1]==0 && btn_state==0 && BQ_adc_state==0) {
            PWM_power_down();
            CLRWDT();
            SLEEP();    
            RESET();
        }
        
    }  
}
