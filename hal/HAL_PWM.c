// File:  HAL_PWM.c
// Author: JSpicer
// Date:  10/04/17
// Purpose: PWM utilities
// Hardware:  TM4C123 Tiva board

#include <stdint.h>
#include <assert.h>
#include "HAL.h"
#include "tm4c123gh6pm.h"

// These are base IO addresses of the PWM modules.
#define PWM0_REG_BASE 	((volatile uint32_t *)0x40028000)
#define PWM1_REG_BASE		((volatile uint32_t *)0x40029000)

// This structure represents the registers associated with a PWM module.
//	It will be overlayed on top of IO memory so that the structure fields
//	map to the registers.  (See the datasheet for field/register descriptions).
typedef struct {
	uint32_t  CTL;
	uint32_t  SYNC;
	uint32_t  ENABLE;
	uint32_t  INVERT;
	uint32_t  FAULT;
	uint32_t  INTEN;
	uint32_t  RIS;
	uint32_t  ISC;
	uint32_t  STATUS;
	uint32_t  FAULTVAL;
	uint32_t  ENUPD;
	uint32_t  RESERVED[5];
	uint32_t  _0_CTL;
	uint32_t  _0_INTEN;
	uint32_t  _0_RIS;
	uint32_t  _0_ISC;
	uint32_t  _0_LOAD;
	uint32_t  _0_COUNT;
	uint32_t  _0_CMPA;
	uint32_t  _0_CMPB;
	uint32_t  _0_GENA;
	uint32_t  _0_GENB;
	uint32_t  _0_DBCTL;
	uint32_t  _0_DBRISE;
	uint32_t  _0_DBFALL;
	uint32_t  _0_FLTSRC0;
	uint32_t  _0_FLTSRC1;
	uint32_t  _0_MINFLTPER;
	uint32_t  _1_CTL;
	uint32_t  _1_INTEN;
	uint32_t  _1_RIS;
	uint32_t  _1_ISC;
	uint32_t  _1_LOAD;
	uint32_t  _1_COUNT;
	uint32_t  _1_CMPA;
	uint32_t  _1_CMPB;
	uint32_t  _1_GENA;
	uint32_t  _1_GENB;
	uint32_t  _1_DBCTL;
	uint32_t  _1_DBRISE;
	uint32_t  _1_DBFALL;
	uint32_t  _1_FLTSRC0;
	uint32_t  _1_FLTSRC1;
	uint32_t  _1_MINFLTPER;
	uint32_t  _2_CTL;
	uint32_t  _2_INTEN;
	uint32_t  _2_RIS;
	uint32_t  _2_ISC;
	uint32_t  _2_LOAD;
	uint32_t  _2_COUNT;
	uint32_t  _2_CMPA;
	uint32_t  _2_CMPB;
	uint32_t  _2_GENA;
	uint32_t  _2_GENB;
	uint32_t  _2_DBCTL;
	uint32_t  _2_DBRISE;
	uint32_t  _2_DBFALL;
	uint32_t  _2_FLTSRC0;
	uint32_t  _2_FLTSRC1;
	uint32_t  _2_MINFLTPER;
	uint32_t  _3_CTL;
	uint32_t  _3_INTEN;
	uint32_t  _3_RIS;
	uint32_t  _3_ISC;
	uint32_t  _3_LOAD;
	uint32_t  _3_COUNT;
	uint32_t  _3_CMPA;
	uint32_t  _3_CMPB;
	uint32_t  _3_GENA;
	uint32_t  _3_GENB;
	uint32_t  _3_DBCTL;
	uint32_t  _3_DBRISE;
	uint32_t  _3_DBFALL;
	uint32_t  _3_FLTSRC0;
	uint32_t  _3_FLTSRC1;
	uint32_t  _3_MINFLTPER;
	uint32_t  RESERVED1[432];
	uint32_t  _0_FLTSEN;
	uint32_t  _0_FLTSTAT0;
	uint32_t  _0_FLTSTAT1;
	uint32_t  RESERVED2[29];
	uint32_t  _1_FLTSEN;
	uint32_t  _1_FLTSTAT0;
	uint32_t  _1_FLTSTAT1;
	uint32_t  RESERVED3[30];
	uint32_t  _2_FLTSTAT0;
	uint32_t  _2_FLTSTAT1;
	uint32_t  RESERVED4[30];
	uint32_t  _3_FLTSTAT0;
	uint32_t  _3_FLTSTAT1;
	uint32_t  RESERVED5[397];
	uint32_t  PP;
} PWMRegs_t;

// This array is a look table to resolve the PWM module name to its base address.
const volatile uint32_t * PWMBaseAddress[] = {
	PWM0_REG_BASE,
	PWM1_REG_BASE,
};


void PWM_SetClockDivisor(uint8_t divisor)
{
	
	// Must enable the PWM clock BEFORE setting the divisor, otherwise a 
	//	fault exception will get raised while configuring the PWM channel.
	SYSCTL_RCGCPWM_R |= 1;
	
	// Find the right-most, non-zero bit.
	uint8_t value = 0;
	for (; value < 8 && !(divisor & 0x1); value++, divisor >>= 1); 
	
	if (value > 0) {
		// Clear then set the PWMDIV field.  The divisor bits are D19-D17
		SYSCTL_RCC_R &= ~SYSCTL_RCC_PWMDIV_M;
		SYSCTL_RCC_R |= ((uint32_t)(value - 1) << 17);

		// Enable the PWM clock divisor.
		SYSCTL_RCC_R |= SYSCTL_RCC_USEPWMDIV;
	}
	else {
		// Disable the PWMclock divisor.  PWM modules will now be fed the system clock.
		SYSCTL_RCC_R &= ~SYSCTL_RCC_USEPWMDIV;	
	}
}

//----------------------- PWM_Configure --------------------------
// Configure the specified PWM module/channel with a given period 
//	and duty cycle.
// Inputs:  module - the PWM module name.
//          channel - the PWM channel name.
//          period - the pulse period.
//          duty - the number of bus cycles for initial duty cycle 
// Outputs:  none.
void PWM_Configure(PWMModule module, PWMChannel channel, uint16_t period, uint16_t duty)
{
	// Overlay the PWM register structure on top of the PWMx memory region...
	volatile PWMRegs_t* pwm = (volatile PWMRegs_t*)PWMBaseAddress[module];
	
	switch (module) {
		
		case PWMModule0:
			
			SYSCTL_RCGCPWM_R |= 1;
	
			switch (channel) {
				
				// Generator 0
				case PWM0:	// PB6
					
					// Set PCTL field for PB6 to 0x4 for PWM.
					GPIO_EnableAltDigital(PORTB, 0x40, 0x4);
				
					// Disable the generator.
					pwm->_0_CTL = 0;
				
					// Drive PWM0 HIGH when when counter matches comparator A while counting down 
					//	and drive LOW when counter reaches zero.
					pwm->_0_GENA =  PWM_0_GENA_ACTCMPAD_ONE | PWM_0_GENA_ACTLOAD_ZERO;
				
					// Set the count preset which will determine the period.
					pwm->_0_LOAD = period - 1;
				
					// Set the duty cycle count.
					pwm->_0_CMPA = duty - 1;
				
					// Enable the generator, i.e. start the timer.
					pwm->_0_CTL |= 0x01; 

					break;
				
				// Generator 1
				case PWM3:	// PB5
					
					// Set PCTL field for PB5 to 0x4 for PWM.
					GPIO_EnableAltDigital(PORTB, 0x20, 0x4);
				
					// Disable the generator.
					pwm->_1_CTL = 0;
				
					// Drive PWM3 HIGH when when counter matches comparator A while counting down 
					//	and drive LOW when counter reaches zero.
					pwm->_1_GENB =  PWM_1_GENB_ACTCMPAD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
				
					// Set the count preset which will determine the period.
					pwm->_1_LOAD = period - 1;
				
					// Set the duty cycle count.
					pwm->_1_CMPA = duty - 1;
				
					// Enable the generator, i.e. start the timer.
					pwm->_1_CTL |= 0x01; 

					break;
				
				// Generator 2
				case PWM4: // PE4
					
					// Set PCTL field for PE4 to 0x4 for PWM.
					GPIO_EnableAltDigital(PORTE, 0x10, 0x4);
				
					// Disable the generator.
					pwm->_2_CTL = 0;
				
					// Drive PWM4 HIGH when when counter matches comparator A while counting down 
					//	and drive LOW when counter reaches zero.
					pwm->_2_GENA =  PWM_2_GENA_ACTCMPAD_ONE | PWM_2_GENA_ACTLOAD_ZERO;
				
					// Set the count preset which will determine the period.
					pwm->_2_LOAD = period - 1;
				
					// Set the duty cycle count.
					pwm->_2_CMPA = duty - 1;
				
					// Enable the generator, i.e. start the timer.
					pwm->_2_CTL |= 0x01; 

					break;
				
				default:
					abort();  // Enable other channels as needed.
					break;
				
			};
				
			break;
		
		case PWMModule1:
			
			SYSCTL_RCGCPWM_R |= 2;
			
			switch (channel) {
				
				// Generator 1
				case PWM3:	// PA7
					
					// Set PCTL field for PB5 to 0x5 for PWM3.
					GPIO_EnableAltDigital(PORTA, 0x80, 0x5);
				
					// Disable the generator.
					pwm->_1_CTL = 0;
				
					// Drive PWM3 HIGH when when counter matches comparator A while counting down 
					//	and drive LOW when counter reaches zero.
					pwm->_1_GENB =  PWM_1_GENB_ACTCMPAD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
				
					// Set the count preset which will determine the period.
					pwm->_1_LOAD = period - 1;
				
					// Set the duty cycle count.
					pwm->_1_CMPA = duty - 1;
				
					// Enable the generator, i.e. start the timer.
					pwm->_1_CTL |= 0x01; 

					break;
				
				default:
					abort();  // Enable other channels as needed.
					break;
				
			};
				
		
		
		
			break;
		
	};

}

void PWM_Enable(PWMModule module, PWMChannel channel)
{
	// Overlay the PWM register structure on top of the PWMx memory region...
	volatile PWMRegs_t* pwm = (volatile PWMRegs_t*)PWMBaseAddress[module];
	pwm->ENABLE |= (0x1 << (int)channel);
}
	
void PWM_Disable(PWMModule module, PWMChannel channel)
{
	// Overlay the PWM register structure on top of the PWMx memory region...
	volatile PWMRegs_t* pwm = (volatile PWMRegs_t*)PWMBaseAddress[module];
	pwm->ENABLE &= ~(0x1 << (int)channel);
}


//----------------------- PWM_SetDuty --------------------------
// Set the duty cycle of an enabled PWM module/channel
// Inputs:  module - the PWM module name.
//          channel - the PWM channel name.
//          duty - the number of bus cycles for the duty cycle.
// Outputs:  none.
void PWM_SetDuty(PWMModule module, PWMChannel channel, uint16_t duty)
{
	// Overlay the PWM register structure on top of the PWMx memory region...
	volatile PWMRegs_t* pwm = (volatile PWMRegs_t*)PWMBaseAddress[module];
	
	switch (module) {
		
		case PWMModule0:
			
			switch (channel) {
				
				case PWM3:	// PB5
					// Set the duty cycle count.
					pwm->_1_CMPA = duty - 1;
					break;
				
				case PWM4: // PE4
					// Set the duty cycle count.
					pwm->_2_CMPA = duty - 1;
					break;
				
				default:
					// TODO:  add other channels as needed.
					abort();
					break;
				
			}
			break;
			
		default:
			// TODO;  add other modules as needed
			abort();
			break;
	}
}

