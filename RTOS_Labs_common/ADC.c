// *************ADC.c**************
// EE445M/EE380L.6 Labs 1, 2, Lab 3, and Lab 4 
// mid-level ADC functions
// you are allowed to call functions in the low level ADCSWTrigger driver
// 
// Runs on LM4F120/TM4C123
// Jonathan W. Valvano Jan 5, 2020, valvano@mail.utexas.edu
#include <stdint.h>
#include "vware/ADCSWTrigger.h"
#include "vware/tm4c123gh6pm.h"


// channelNum (0 to 11) specifies which pin is sampled with sequencer 3
// software start
// return with error 1, if channelNum>11, 
// otherwise initialize ADC and return 0 (success)
int ADC_Init(uint32_t channelNum){
    // put your Lab 1 code here

    int rv = 0;
    if(channelNum > 11) {
        rv = 1;
        goto exit;
    }

    SYSCTL_RCGCADC_R |= 0x0001;   // 1) activate ADC0
    SYSCTL_RCGCGPIO_R |= 0x10;    // 2) activate clock for Port E
    while((SYSCTL_PRGPIO_R&0x10) != 0x10){};  // 3 for stabilization

    /*
     *  Channel         IO
     *  =====================
     *     0            PE3
     *     1            PE2
     *     2            PE1
     *     3            PE0
     *     4            PD3
     *     5            PD2
     *     6            PD1
     *     7            PD0
     *     8            PE5
     *     9            PE4
     *     10           PB4
     *     11           PB5
     */

    uint8_t pinNum;
    if(channelNum <= 3) {
        pinNum = 3 - channelNum;

        GPIO_PORTE_DIR_R    &= ~(1u << pinNum);     //  make input
        GPIO_PORTE_AFSEL_R  |=  (1u << pinNum);     //  enable alternate function
        GPIO_PORTE_DEN_R    &= ~(1u << pinNum);     //  disable digital I/O
        GPIO_PORTE_AMSEL_R  |=  (1u << pinNum);     //  enable analog functionality
    } else if(channelNum <= 7) {
        pinNum = 7 - channelNum;

        GPIO_PORTD_DIR_R    &= ~(1u << pinNum);     //  make input
        GPIO_PORTD_AFSEL_R  |=  (1u << pinNum);     //  enable alternate function
        GPIO_PORTD_DEN_R    &= ~(1u << pinNum);     //  disable digital I/O
        GPIO_PORTD_AMSEL_R  |=  (1u << pinNum);     //  enable analog functionality
    } else if(channelNum <= 9) {
        pinNum = 13 - channelNum;

        GPIO_PORTE_DIR_R    &= ~(1u << pinNum);     //  make input
        GPIO_PORTE_AFSEL_R  |=  (1u << pinNum);     //  enable alternate function
        GPIO_PORTE_DEN_R    &= ~(1u << pinNum);     //  disable digital I/O
        GPIO_PORTE_AMSEL_R  |=  (1u << pinNum);     //  enable analog functionality
    } else if(channelNum <= 11) {
        pinNum = channelNum - 6;

        GPIO_PORTB_DIR_R    &= ~(1u << pinNum);     //  make input
        GPIO_PORTB_AFSEL_R  |=  (1u << pinNum);     //  enable alternate function
        GPIO_PORTB_DEN_R    &= ~(1u << pinNum);     //  disable digital I/O
        GPIO_PORTB_AMSEL_R  |=  (1u << pinNum);     //  enable analog functionality
    }

    while((SYSCTL_PRADC_R&0x0001) != 0x0001){}; // good code, but not implemented in simulator
    ADC0_PC_R &= ~0xF;
    ADC0_PC_R |= 0x1;             // 8) configure for 125K samples/sec
    ADC0_SSPRI_R = 0x0123;        // 9) Sequencer 3 is highest priority
    ADC0_ACTSS_R &= ~0x0008;      // 10) disable sample sequencer 3
    ADC0_EMUX_R &= ~0xF000;       // 11) seq3 is software trigger
    ADC0_SSMUX3_R &= ~0x000F;

    ADC0_SSMUX3_R += channelNum;  // 12) set channel

    ADC0_SSCTL3_R = 0x0006;       // 13) no TS0 D0, yes IE0 END0
    ADC0_IM_R &= ~0x0008;         // 14) disable SS3 interrupts
    ADC0_ACTSS_R |= 0x0008;       // 15) enable sample sequencer 3

exit:
    return rv;
}

// software start sequencer 3 and return 12 bit ADC result
uint32_t ADC_In(void){
    // put your Lab 1 code here

    uint32_t result;
    ADC0_PSSI_R = 0x0008;            // 1) initiate SS3
    while((ADC0_RIS_R&0x08)==0){};   // 2) wait for conversion done

    result = ADC0_SSFIFO3_R&0xFFF;   // 3) read result

    ADC0_ISC_R = 0x0008;             // 4) acknowledge completion

    return result;
}

float ADC_In_Voltage(void) {
    return ADC_In() * 3.3 / 4096.;
}
