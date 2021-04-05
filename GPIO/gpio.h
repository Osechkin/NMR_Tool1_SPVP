#ifndef GPIO_H_
#define GPIO_H_


#include "../Common/OMAPL138_global.h"
#include "hw_types.h"


#define GP_0							(0)
#define GP_1							(1)
#define GP_2							(2)
#define GP_3							(3)
#define GP_4							(4)
#define GP_5							(5)
#define GP_6							(6)
#define GP_7							(7)
#define GP_8							(8)
#define GP_9							(9)
#define GP_10							(10)
#define GP_11							(11)
#define GP_12							(12)
#define GP_13							(13)
#define GP_14							(14)
#define GP_15							(15)



#define CSL_GPIO_DIR_DIR_OUT            (0x00000000u)
#define CSL_GPIO_DIR_DIR_IN             (0x00000001u)


#define CSL_GPIO_STATE_LOW				(0)
#define CSL_GPIO_STATE_HIGH				(1)


#define GPIO_FAL_ONLY					(0)
#define GPIO_RIS_ONLY					(1)
#define GPIO_FAL_AND_RIS				(3)


#define GPIO_UNKNOWN_STATE				(0)
#define GPIO_HIGH_STATE					(1)
#define GPIO_FALL_STATE					(2)
#define GPIO_LOW_STATE					(3)
#define GPIO_RISE_STATE					(4)

#define GPIO_IN_DATA(n)					(0x20 + (0x28 * n))


/*
static volatile unsigned char new_GP0p1 = 1;
static volatile unsigned char old_GP0p1 = 1;
static volatile unsigned int new_GP0p2 = 1;
static volatile unsigned int old_GP0p2 = 1;
static volatile unsigned int new_GP0p3 = 1;
static volatile unsigned int old_GP0p3 = 1;
*/


void gpioPowerOn(CSL_PscRegsOvly psc1Regs);
void enableGPIOPinMux_Bank0(int *pins, int count, CSL_SyscfgRegsOvly sysRegs);
void configureGPIOPins_Bank0(int *pins, int *pin_dirs, int count, CSL_GpioRegsOvly gpioRegs);
void writeGPIOPin_Bank0(int pin, unsigned int state, CSL_GpioRegsOvly gpioRegs);
unsigned int readGPIOPin_Bank0(int pin, CSL_GpioRegsOvly gpioRegs);
unsigned int readGPIO_Bank0(CSL_GpioRegsOvly gpioRegs);
void configureGPIOInterrupts_Bank0(int *pins, int *int_states, int count, CSL_GpioRegsOvly gpioRegs);
void mapGPIOInterrupt_Bank0(int intc, CSL_DspintcRegsOvly intcRegs);

int GPIOPinRead(unsigned int baseAdd, unsigned int pinNumber);
unsigned int GPIOBankRead(unsigned int baseAdd, unsigned int bankNumber);


#endif // GPIO_H_
