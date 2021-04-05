/*
 * gpio.c
 *
 *  Created on: 19.10.2016
 *      Author: Osechkin
 */

#include <cstdio>

#include "gpio.h"



void enableGPIOPinMux_Bank0(int *pins, int count, CSL_SyscfgRegsOvly sysRegs)
{
	/* Key to be written to enable the pin mux registers for write            */
	sysRegs->KICK0R = 0x83e70b13;
	sysRegs->KICK1R = 0x95A4F1E0;

	int i;
	for (i = 0; i < count; i++)
	{
	 	int pin_number = pins[i];
	   	switch (pin_number)
	   	{
	   	case GP_0:		sysRegs->PINMUX1 = ((CSL_SYSCFG_PINMUX1_PINMUX1_31_28_GPIO0_0) << (CSL_SYSCFG_PINMUX1_PINMUX1_31_28_SHIFT)); break;	// enable the pinmux for the GPIO bank 0 pin 0
	   	case GP_1:		sysRegs->PINMUX1 = ((CSL_SYSCFG_PINMUX1_PINMUX1_27_24_GPIO0_1) << (CSL_SYSCFG_PINMUX1_PINMUX1_27_24_SHIFT)); break;	// enable the pinmux for the GPIO bank 0 pin 1
	   	case GP_2:		sysRegs->PINMUX1 = ((CSL_SYSCFG_PINMUX1_PINMUX1_23_20_GPIO0_2) << (CSL_SYSCFG_PINMUX1_PINMUX1_23_20_SHIFT)); break;	// enable the pinmux for the GPIO bank 0 pin 2
	   	case GP_3:		sysRegs->PINMUX1 = ((CSL_SYSCFG_PINMUX1_PINMUX1_19_16_GPIO0_3) << (CSL_SYSCFG_PINMUX1_PINMUX1_19_16_SHIFT)); break;	// enable the pinmux for the GPIO bank 0 pin 3
	   	case GP_4:		sysRegs->PINMUX1 = ((CSL_SYSCFG_PINMUX1_PINMUX1_15_12_GPIO0_4) << (CSL_SYSCFG_PINMUX1_PINMUX1_15_12_SHIFT)); break;	// enable the pinmux for the GPIO bank 0 pin 4
	   	case GP_5:		sysRegs->PINMUX1 = ((CSL_SYSCFG_PINMUX1_PINMUX1_11_8_GPIO0_5) << (CSL_SYSCFG_PINMUX1_PINMUX1_11_8_SHIFT)); break;	// enable the pinmux for the GPIO bank 0 pin 5
	   	case GP_6:		sysRegs->PINMUX1 = ((CSL_SYSCFG_PINMUX1_PINMUX1_7_4_GPIO0_6) << (CSL_SYSCFG_PINMUX1_PINMUX1_7_4_SHIFT)); break;		// enable the pinmux for the GPIO bank 0 pin 6
	   	case GP_7:		sysRegs->PINMUX1 = ((CSL_SYSCFG_PINMUX1_PINMUX1_3_0_GPIO0_7) << (CSL_SYSCFG_PINMUX1_PINMUX1_3_0_SHIFT)); break;		// enable the pinmux for the GPIO bank 0 pin 7
	   	case GP_8:		sysRegs->PINMUX0 = ((CSL_SYSCFG_PINMUX0_PINMUX0_31_28_GPIO0_8) << (CSL_SYSCFG_PINMUX0_PINMUX0_31_28_SHIFT)); break;	// enable the pinmux for the GPIO bank 0 pin 8
	   	case GP_9:		sysRegs->PINMUX0 = ((CSL_SYSCFG_PINMUX0_PINMUX0_27_24_GPIO0_9) << (CSL_SYSCFG_PINMUX0_PINMUX0_27_24_SHIFT)); break;	// enable the pinmux for the GPIO bank 0 pin 9
	   	case GP_10:		sysRegs->PINMUX0 = ((CSL_SYSCFG_PINMUX0_PINMUX0_23_20_GPIO0_10) << (CSL_SYSCFG_PINMUX0_PINMUX0_23_20_SHIFT)); break;// enable the pinmux for the GPIO bank 0 pin 10
	   	case GP_11:		sysRegs->PINMUX0 = ((CSL_SYSCFG_PINMUX0_PINMUX0_19_16_GPIO0_11) << (CSL_SYSCFG_PINMUX0_PINMUX0_19_16_SHIFT)); break;// enable the pinmux for the GPIO bank 0 pin 11
	   	case GP_12:		sysRegs->PINMUX0 = ((CSL_SYSCFG_PINMUX0_PINMUX0_15_12_GPIO0_12) << (CSL_SYSCFG_PINMUX0_PINMUX0_15_12_SHIFT)); break;// enable the pinmux for the GPIO bank 0 pin 12
	   	case GP_13:		sysRegs->PINMUX0 = ((CSL_SYSCFG_PINMUX0_PINMUX0_11_8_GPIO0_13) << (CSL_SYSCFG_PINMUX0_PINMUX0_11_8_SHIFT)); break;	// enable the pinmux for the GPIO bank 0 pin 13
	   	case GP_14:		sysRegs->PINMUX0 = ((CSL_SYSCFG_PINMUX0_PINMUX0_7_4_GPIO0_14) << (CSL_SYSCFG_PINMUX0_PINMUX0_7_4_SHIFT)); break;	// enable the pinmux for the GPIO bank 0 pin 14
	   	case GP_15:		sysRegs->PINMUX0 = ((CSL_SYSCFG_PINMUX0_PINMUX0_3_0_GPIO0_15) << (CSL_SYSCFG_PINMUX0_PINMUX0_3_0_SHIFT)); break;	// enable the pinmux for the GPIO bank 0 pin 15
	   	default: break;
	   	}
	   }

	/* lock the pinmux registers                                              */
	sysRegs->KICK0R = 0x00000000;
	sysRegs->KICK1R = 0x00000000;

	printf("GPIO enabled in the pin mux registers for Bank0\n");
}


void configureGPIOPins_Bank0(int *pins, int *pin_dirs, int count, CSL_GpioRegsOvly gpioRegs)
{
	int i;
	for (i = 0; i < count; i++)
	{
		//if (pin_dirs[i] == 0) pin_dirs[i] = CSL_GPIO_DIR_DIR_OUT;
		//else pin_dirs[i] = CSL_GPIO_DIR_DIR_IN;

		switch (pins[i])
		{
		case GP_0:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR0, pin_dirs[i]);	break;
		case GP_1:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR1, pin_dirs[i]);	break;
		case GP_2:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR2, pin_dirs[i]);	break;
		case GP_3:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR3, pin_dirs[i]);	break;
		case GP_4:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR4, pin_dirs[i]);	break;
		case GP_5:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR5, pin_dirs[i]);	break;
		case GP_6:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR6, pin_dirs[i]);	break;
		case GP_7:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR7, pin_dirs[i]);	break;
		case GP_8:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR8, pin_dirs[i]);	break;
		case GP_9:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR9, pin_dirs[i]);	break;
		case GP_10:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR10, pin_dirs[i]);	break;
		case GP_11:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR11, pin_dirs[i]);	break;
		case GP_12:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR12, pin_dirs[i]);	break;
		case GP_13:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR13, pin_dirs[i]);	break;
		case GP_14:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR14, pin_dirs[i]);	break;
		case GP_15:		CSL_FINS(gpioRegs->BANK[0].DIR,	GPIO_DIR_DIR15, pin_dirs[i]);	break;
		default: break;
		}
	}

	printf("GPIO pins in Bank0 were configured for input/output\n");
}


void writeGPIOPin_Bank0(int pin, unsigned int state, CSL_GpioRegsOvly gpioRegs)
{
	if (state == 0) state = CSL_GPIO_STATE_LOW;
	else state = CSL_GPIO_STATE_HIGH;

	switch (pin)
	{
	case 0:		CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT0, state);	break;
	case 1:		CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT1, state);	break;
	case 2:		CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT2, state);	break;
	case 3:		CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT3, state);	break;
	case 4:		CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT4, state);	break;
	case 5:		CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT5, state);	break;
	case 6:		CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT6, state);	break;
	case 7:		CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT7, state);	break;
	case 8:		CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT8, state);	break;
	case 9:		CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT9, state);	break;
	case 10:	CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT10, state);	break;
	case 11:	CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT11, state);	break;
	case 12:	CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT12, state);	break;
	case 13:	CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT13, state);	break;
	case 14:	CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT14, state);	break;
	case 15:	CSL_FINS(gpioRegs->BANK[0].OUT_DATA, GPIO_OUT_DATA_OUT15, state);	break;
	default: break;
	}
}


unsigned int readGPIOPin_Bank0(int pin, CSL_GpioRegsOvly gpioRegs)
{
	unsigned int state = 0;

	switch (pin)
	{
	case 0:		state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN0);	break;
	case 1:		state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN1);	break;
	case 2:		state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN2);	break;
	case 3:		state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN3);	break;
	case 4:		state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN4);	break;
	case 5:		state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN5);	break;
	case 6:		state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN6);	break;
	case 7:		state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN7);	break;
	case 8:		state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN8);	break;
	case 9:		state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN9);	break;
	case 10:	state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN10);	break;
	case 11:	state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN11);	break;
	case 12:	state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN12);	break;
	case 13:	state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN13);	break;
	case 14:	state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN14);	break;
	case 15:	state = CSL_FEXT(gpioRegs->BANK[0].IN_DATA, GPIO_IN_DATA_IN15);	break;
	default: break;
	}

	return state;
}


unsigned int readGPIO_Bank0(CSL_GpioRegsOvly gpioRegs)
{
	unsigned int pin = 0;
	pin = gpioRegs->BANK[0].IN_DATA;
	return pin;
}


/*
 * \brief    Function to power on the GPIO module in the power sleep controller.
 * \param    None
 * \return   None
 *  Note: This function causes the program to abort in case it is unable to enable the GPIO module.
 */
void gpioPowerOn(CSL_PscRegsOvly psc1Regs)
{
    volatile Uint32 pscTimeoutCount = 10240u;
    Uint32          temp            = 0;

    /* we will now power on the GPIO module in the PSC.                       *
     * Configure the GPIO Module to Enable state                              */
    psc1Regs->MDCTL[CSL_PSC_GPIO] = ((psc1Regs->MDCTL[CSL_PSC_GPIO] & 0xFFFFFFE0) | CSL_PSC_MDSTAT_STATE_ENABLE);

    /* Kick start the Enable Command                                          */
    temp = psc1Regs->PTCMD;
    temp = ((temp & CSL_PSC_PTCMD_GO0_MASK) | (CSL_PSC_PTCMD_GO0_SET << CSL_PSC_PTCMD_GO0_SHIFT));

    psc1Regs->PTCMD |= temp;

    /* Wait for the power state transition to occur                           */
    while (((psc1Regs->PTSTAT & (CSL_PSC_PTSTAT_GOSTAT0_IN_TRANSITION)) != 0) && (pscTimeoutCount>0))
    {
        pscTimeoutCount--;
    }

    /* Check if PSC state transition timed out                                */
    if (0 == pscTimeoutCount)
    {
        printf("GPIO PSC transition to ON state timed out\n");
    }
    else
    {
        printf("GPIO enabled in PSC\n");
    }
}


void configureGPIOInterrupts_Bank0(int *pins, int *int_states, int count, CSL_GpioRegsOvly gpioRegs)
{
	/* Enable GPIO Bank interrupt for bank 0                                  */
	CSL_FINST(gpioRegs->BINTEN, GPIO_BINTEN_EN0, ENABLE);

	int i;
	for (i = 0; i < count; i++)
	{
		switch (pins[i])
		{
		case 0:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL0, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS0, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS0, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL0, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL0, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS0, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 1:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL1, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS1, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS1, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL1, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL1, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS1, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 2:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL2, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS2, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS2, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL2, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL2, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS2, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 3:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL3, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS3, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS3, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL3, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL3, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS3, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 4:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL4, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS4, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS4, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL4, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL4, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS4, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 5:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL5, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS5, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS5, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL5, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL5, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS5, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 6:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL6, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS6, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS6, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL6, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL6, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS6, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 7:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL7, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS7, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS7, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL7, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL7, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS7, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 8:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL8, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS8, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS8, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL8, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL8, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS8, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 9:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL9, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS9, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS9, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL9, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL9, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS9, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 10:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL10, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS10, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS10, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL10, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL10, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS10, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 11:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL11, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS11, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS11, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL11, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL11, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS11, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 12:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL12, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS12, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS12, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL12, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL12, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS12, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 13:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL13, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS13, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS13, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL13, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL13, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS13, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 14:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL14, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS14, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS14, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL14, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL14, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS14, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		case 15:
		{
			if (int_states[i] == GPIO_FAL_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL15, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_RIS_TRIG, GPIO_CLR_RIS_TRIG_CLRRIS15, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			else if (int_states[i] == GPIO_RIS_ONLY)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS15, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].CLR_FAL_TRIG, GPIO_CLR_FAL_TRIG_CLRFAL15, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
			}
			else if (int_states[i] == GPIO_FAL_AND_RIS)
			{
				CSL_FINS(gpioRegs->BANK[0].SET_FAL_TRIG, GPIO_SET_FAL_TRIG_SETFAL15, CSL_GPIO_SET_FAL_TRIG_SETFAL_ENABLE);
				CSL_FINS(gpioRegs->BANK[0].SET_RIS_TRIG, GPIO_SET_RIS_TRIG_SETRIS15, CSL_GPIO_SET_RIS_TRIG_SETRIS_ENABLE);
			}
			break;
		}
		default: break;
		}
	}

	printf("Enable GPIO interrupt (Bank0) for rising/falling/both\n");
}


void mapGPIOInterrupt_Bank0(int intc, CSL_DspintcRegsOvly intcRegs)
{
	switch (intc)
	{
	case 4:		CSL_FINS(intcRegs->INTMUX1, DSPINTC_INTMUX1_INTSEL4, CSL_INTC_EVENTID_GPIO_BNK0_INT);	break;	// map GPIO0 event to cpu int4
	case 5:		CSL_FINS(intcRegs->INTMUX1, DSPINTC_INTMUX1_INTSEL5, CSL_INTC_EVENTID_GPIO_BNK0_INT);	break;	// map GPIO0 event to cpu int5
	case 6:		CSL_FINS(intcRegs->INTMUX1, DSPINTC_INTMUX1_INTSEL6, CSL_INTC_EVENTID_GPIO_BNK0_INT);	break;	// map GPIO0 event to cpu int6
	case 7:		CSL_FINS(intcRegs->INTMUX1, DSPINTC_INTMUX1_INTSEL7, CSL_INTC_EVENTID_GPIO_BNK0_INT);	break;	// map GPIO0 event to cpu int7
	case 8:		CSL_FINS(intcRegs->INTMUX2, DSPINTC_INTMUX2_INTSEL8, CSL_INTC_EVENTID_GPIO_BNK0_INT);	break;	// map GPIO0 event to cpu int8
	case 9:		CSL_FINS(intcRegs->INTMUX2, DSPINTC_INTMUX2_INTSEL9, CSL_INTC_EVENTID_GPIO_BNK0_INT);	break;	// map GPIO0 event to cpu int9
	case 10:	CSL_FINS(intcRegs->INTMUX2, DSPINTC_INTMUX2_INTSEL10, CSL_INTC_EVENTID_GPIO_BNK0_INT);	break;	// map GPIO0 event to cpu int10
	case 11:	CSL_FINS(intcRegs->INTMUX2, DSPINTC_INTMUX2_INTSEL11, CSL_INTC_EVENTID_GPIO_BNK0_INT);	break;	// map GPIO0 event to cpu int11
	case 12:	CSL_FINS(intcRegs->INTMUX3, DSPINTC_INTMUX3_INTSEL12, CSL_INTC_EVENTID_GPIO_BNK0_INT);	break;	// map GPIO0 event to cpu int12
	case 13:	CSL_FINS(intcRegs->INTMUX3, DSPINTC_INTMUX3_INTSEL13, CSL_INTC_EVENTID_GPIO_BNK0_INT);	break;	// map GPIO0 event to cpu int13
	case 14:	CSL_FINS(intcRegs->INTMUX3, DSPINTC_INTMUX3_INTSEL14, CSL_INTC_EVENTID_GPIO_BNK0_INT);	break;	// map GPIO0 event to cpu int14
	case 15:	CSL_FINS(intcRegs->INTMUX3, DSPINTC_INTMUX3_INTSEL15, CSL_INTC_EVENTID_GPIO_BNK0_INT);	break;	// map GPIO0 event to cpu int15
	default: break;
	}

	printf("GPIO0 events (Bank0) were mapped to CPU interrupts\n");
}


/**
 * \brief    This function reads the value(logic level) of an input or an
 *           output pin.
 *
 * \param    baseAdd     The memory address of the GPIO instance being used.
 * \param    pinNumber   The serial number of the GPIO pin.
 *                       The 144 GPIO pins have serial numbers from 1 to 144.
 *
 * \return   This returns the value present on the specified pin. This returns
 *           one of the following values:
 *           1> GPIO_PIN_LOW, if the value on the pin is logic 0.
 *           2> GPIO_PIN_HIGH, if the value on the pin is logic 1.
 *
 * \note     Using this function, we can read the values of both input and
 *           output pins.
 */
int GPIOPinRead(unsigned int baseAdd, unsigned int pinNumber)
{
    unsigned int regNumber = 0;
    unsigned int pinOffset = 0;
    unsigned int val = 0;

    /*
    ** Each register contains settings for each pin of two banks. The 32 bits
    ** represent 16 pins each from the banks. Thus the register number must be
    ** calculated based on 32 pins boundary.
    */

    regNumber = (pinNumber - 1)/32;

    /*
    ** In every register the least significant bits starts with a GPIO number on
    ** a boundary of 32. Thus the pin offset must be calculated based on 32
    ** pins boundary. Ex: 'pinNumber' of 1 corresponds to bit 0 in
    ** 'register_name01'.
    */

    pinOffset = (pinNumber - 1) % 32;

    val = HWREG(baseAdd + GPIO_IN_DATA(regNumber)) & (1 << pinOffset);

    return (val >> pinOffset);
}


/**
 * \brief    This function reads the value of an input or an
 *           output bank.
 *
 * \param    baseAdd     The memory address of the GPIO instance being used.
 * \param    bankNumber  The serial number of the GPIO bank. *
 *
 * \return   This returns the value present on the specified bank pins from 0 to 31.
 *
 * \note     Using this function, we can read the values of both input and
 *           output pins.
 */
unsigned int GPIOBankRead(unsigned int baseAdd, unsigned int bankNumber)
{
	unsigned int val = 0;

	/*
	** Each register contains settings for each pin of two banks. The 32 bits
	** represent 16 pins each from the banks. Thus the register number must be
	** calculated based on 32 pins boundary.
	*/

	val = HWREG(baseAdd + GPIO_IN_DATA(bankNumber));

	return val;
}
