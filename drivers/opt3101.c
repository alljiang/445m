#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "vware/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "RTOS/OS.h"
#include "utils/utils.h"
#include "drivers/opt3101.h"

/*
 *         L2   R0
 *      L1         R1
 *   L0               R2
 *
 *   Address: 0b
 *
 *   OPT3101 Left
 *      I2C3_SCL: PD0
 *      I2C3_SDA: PD1
 *           RST: PD6
 *
 *   OPT3101 Right
 *      I2C0_SCL: PB2
 *      I2C0_SDA: PB3
 *           RST: PD7
 */

void
OPT3101_Initialize() {
#ifdef OPT3101_L
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);

    GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), true);

    // Reset for > 30 us
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0);
    delayMicroseconds(50);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6);
#endif

#ifdef OPT3101_R
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    // Reset for > 30 us
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0);
    delayMicroseconds(50);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_PIN_7);
#endif


    // Write the NUM_SUB_FRAMES and NUM_AVG_SUB_FRAMES registers to set the device to operate at the required sample rate

    // Select the maximum ambient current to be supported by writing IAMB_MAX_SEL

    // Write illumination DAC currents ILLUM_DAC_L_TX0 and ILLUM_DAC_H_TX0

    // Program the adaptive HDR thresholds: HDR_THR_LOW and HDR_THR_HIGH.

    // Load all the calibration settings: illumination crosstalk, phase offset, phase temperature coefficient, and phase ambient coefficient

    // Enable on-chip temperature conversion: EN_TEMP_CONV = 1

    // Write I2C host settings to read the external temperature sensor if it is present in the system. Register settings are listed in Table 26

    // Enable the timing generator by setting TG_EN = 1

    // Perform internal crosstalk correction by making INT_XTALK_CALIB = 1, followed by INT_XTALK_CALIB = 0.
}

void OPT3101_write8(int I2C_base, uint8_t reg, uint8_t data) {
    I2CMasterSlaveAddrSet(I2C_base, OPT3101_I2C_ADDRESS, false);

    I2CMasterDataPut(I2C_base, reg);
    I2CMasterControl(I2C_base, I2C_MASTER_CMD_BURST_SEND_START);

    I2CMasterDataPut(I2C_base, data);
    I2CMasterControl(I2C_base, I2C_MASTER_CMD_BURST_SEND_FINISH);

    while(I2CMasterBusy(I2C_base));
}

int8_t OPT3101_receive8(int I2C_base, uint8_t reg) {
    I2CMasterSlaveAddrSet(I2C_base, OPT3101_I2C_ADDRESS, false);
    I2CMasterDataPut(I2C_base, reg);
    I2CMasterControl(I2C_base, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C_base));

    I2CMasterSlaveAddrSet(I2C_base, OPT3101_I2C_ADDRESS, true);
    I2CMasterControl(I2C_base, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(I2CMasterBusy(I2C_base));
    return I2CMasterDataGet(I2C_base);
}

int16_t OPT3101_receive16(int I2C_base, uint8_t high_addr, uint8_t low_addr) {
    uint8_t low = I2C_receive8(I2C_base, low_addr);
    uint8_t high = I2C_receive8(I2C_base, high_addr);

    return (high << 8) | low;
}

void
OPT3101_Update() {
    writeReg(0x00, 0x000000);

#ifdef OPT3101_L
    uint32_t reg08 = readReg(0x08);
    phase = reg08 & 0xFFFF;
    distanceMillimeters = (int32_t)phase * 14990 >> 16;
#endif

#ifdef OPT3101_R
#endif
}

float
OPT3101_GetDistance() {

}
