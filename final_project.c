//*****************************************************************************
// 
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "final_project.h"
#include "Strings.h"
#include <hw_types.h>
#include <hw_gpio.h>
#include <sysctl.h>
#include <pin_map.h>
#include <rom_map.h>
#include <gpio.h>
#include <tm4c123gh6pm.h>
#include <uart.h>
#include <interrupt.h>
#include <MPU9250_reg.h>
#include <HAL.h>
#include <FreeRTOS.h>
#include <task.h>
#include "midi.h"
#include "hw_memmap.h"
#include <semphr.h>
#include <timers.h>
#include <math.h>

//*****************************************************************************

void I2C3_Init(void);
char I2C3_Wr(int slaveAddr, char memAddr, char data);
char I2C3_Rd(int slaveAddr, char memAddr, int byteCount, char* data);
void Delay(unsigned long counter);
void UART_Transmitter(unsigned char data);
void printstring(char *str);
void MPU9250_Init(void);
void UART_init(void);
static void PlayNote(const MidiNoteEvent_t* note, PWMModule pwmModule, PWMChannel pwmChannel);
void SwitchHandler(uint32_t pinMap);
int IsPlaying(void);
void vSwitchTask(void *pvParameters);
void vTrackTask(void *pvParameters);
void vSensorTask(void *pvParameters);
static int InitHardware(void);
static int motion_detect(void);
void print_readings(void);
void init_vals(void);

uint32_t SystemCoreClock;

static SemaphoreHandle_t armSemaphore_;
static SemaphoreHandle_t trackSemaphore_;
static SemaphoreHandle_t sensorSemaphore_;
static uint8_t switchPressed_;

volatile uint32_t avg_accX;
volatile uint32_t avg_accY;
volatile uint32_t avg_accZ;

const double threshold = 1.5;

extern MidiFile_t midi_tune;

typedef struct {
	TaskHandle_t taskHandle;
	PWMModule pwmModule; 
	PWMChannel pwmChannel;
	const MidiTrack_t* track;
	uint8_t isPlaying;
} TrackParams_t;

// These are the track task creation parameters.
static TrackParams_t trackParams_[4] = { 
	{ NULL, PWMModule0, PWM0, NULL, 0 },
	{ NULL, PWMModule0, PWM3, NULL, 0 },
	{ NULL, PWMModule0, PWM4, NULL, 0 },
	{ NULL, PWMModule1, PWM3, NULL, 0 }
};

char msg[20];

int main(void)
{
	InitHardware();
	I2C3_Init();
	MPU9250_Init();
	UART_init();
	init_vals();
	
	armSemaphore_ = xSemaphoreCreateBinary();
	sensorSemaphore_ = xSemaphoreCreateBinary();
	trackSemaphore_ = xSemaphoreCreateCounting(4, 0);
	
	if (armSemaphore_ && trackSemaphore_ && sensorSemaphore_)
	{
		// The switch task will have a higher priority than the track tasks.
		xTaskCreate(vSwitchTask, "Switch Task", 100, NULL, 3, NULL);
		
		xTaskCreate(vSensorTask, "Sensor Task", 100, NULL, 2, NULL);
		
		for (int i = 0; i < midi_tune.numTracks && i < 4; i++ )
		{		
			trackParams_[i].track = &midi_tune.tracks[i];
			xTaskCreate(vTrackTask, "Track Task", 100, &trackParams_[i], 1, &trackParams_[i].taskHandle);
		}
		// Startup of the FreeRTOS scheduler.  The program should block here.  
		vTaskStartScheduler();
	}
	
//	while(1)
//	{
//		if(motion_detect())
//		{
//			sprintf(msg, "Motion detected\n\r");
//			printstring(msg);
//		}
//		else
//		{
//			sprintf(msg, "No motion detected\n\r");
//			printstring(msg);
//		}
//		
//		print_readings();
//	}
}

void init_vals(void)
{
	char sensorRaw[6];
	int accX, accY, accZ;
	int sum_accX, sum_accY, sum_accZ;
	
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
	
	for(int i = 0; i < 10; i++)
	{
		I2C3_Rd(0x68,MPUREG_ACCEL_XOUT_H, 6, sensorRaw);
		accX = (int)( (sensorRaw[0] << 8 ) | sensorRaw[1] );
		accY = (int)( (sensorRaw[2] << 8 ) | sensorRaw[3] );
		accZ = (int)( (sensorRaw[4] << 8 ) | sensorRaw[5] );
		
		sum_accX += accX;
		sum_accY += accY;
		sum_accZ += accZ;
		Delay(100);
	}
	
	avg_accX = sum_accX / 10;
	avg_accY = sum_accY / 10;
	avg_accZ = sum_accZ / 10;
	
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
}

static int motion_detect(void)
{
	char accRaw[6];
	int accX, accY, accZ;
	
	I2C3_Rd(0x68,MPUREG_ACCEL_XOUT_H, 6, accRaw);
	accX = (int)( (accRaw[0] << 8 ) | accRaw[1] );
	accY = (int)( (accRaw[2] << 8 ) | accRaw[3] );
	accZ = (int)( (accRaw[4] << 8 ) | accRaw[5] );
	Delay(10);
	
	if((accX > avg_accX*threshold) || (accY > avg_accY*threshold) || (accZ > avg_accZ*threshold))
		return 1;
	else
		return 0;
}

void print_readings(void)
{
	int  accX, accY, accZ;
	char sensordata[14];
		
	I2C3_Rd(0x68,MPUREG_ACCEL_XOUT_H, 6, sensordata);
	accX = (int)( (sensordata[0] << 8 ) | sensordata[1] );
	accY = (int)( (sensordata[2] << 8 ) | sensordata[3] );
	accZ = (int)( (sensordata[4] << 8 ) | sensordata[5] );
	Delay(100);
	
	sprintf(msg,"AccX  = %d \n\r",accX);
		printstring(msg);
	sprintf(msg,"AccY  = %d \n\r",accY);
		printstring(msg);
	sprintf(msg,"AccZ  = %d \n\r",accZ);
		printstring(msg);
}

void InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
}

void MPU9250_Init(void)
{
	I2C3_Wr(0x68,MPUREG_SMPLRT_DIV, 0x07);
	I2C3_Wr(0x68,MPUREG_PWR_MGMT_1,  0x01);
	I2C3_Wr(0x68,MPUREG_CONFIG, 0x00);
	I2C3_Wr(0x68,MPUREG_ACCEL_CONFIG,0x00); 
	I2C3_Wr(0x68,MPUREG_GYRO_CONFIG,0x18);
	I2C3_Wr(0x68,MPUREG_INT_ENABLE, 0x01);
}

void UART_init(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	
//	IntPrioritySet(INT_UART0, 0x01); 		// configure UART0 interrupt priority as 1

	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
	(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	
//	IntEnable(INT_UART0);
	MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

void I2C3_Init(void)
{
	SYSCTL_RCGCGPIO_R |= 0x00000008 ; // Enable the clock for port D
	SYSCTL_RCGCI2C_R   |= 0x00000008 ; // Enable the clock for I2C 3
	GPIO_PORTD_DEN_R |= 0x03; // Assert DEN for port D
	// Configure Port D pins 0 and 1 as I2C 3
	GPIO_PORTD_AFSEL_R |= 0x00000003 ;
	GPIO_PORTD_PCTL_R |= 0x00000033 ;
	GPIO_PORTD_ODR_R |= 0x00000002 ; // SDA (PD1 ) pin as open drain
	I2C3_MCR_R  = 0x0010 ; // Enable I2C 3 master function
	/* Configure I2C 3 clock frequency
	(1 + TIME_PERIOD ) = SYS_CLK /(2*
	( SCL_LP + SCL_HP ) * I2C_CLK_Freq )
	TIME_PERIOD = 16 ,000 ,000/(2(6+4) *100000) - 1 = 7 */
	I2C3_MTPR_R   = 0x07 ;
}

/* Wait until I2C master is not busy and return error code */
/* If there is no error, return 0 */
static int I2C_wait_till_done(void)
{
    while(I2C3_MCS_R & 1) {}   /* wait until I2C master is not busy */
		return I2C3_MCS_R & 0xE; /* return I2C error code */
}

/* Write one byte only */
/* byte write: S-(saddr+w)-ACK-maddr-ACK-data-ACK-P */
char I2C3_Wr(int slaveAddr, char memAddr, char data)
{
    char error;

    /* send slave address and starting address */
    I2C3_MSA_R = slaveAddr << 1;
    I2C3_MDR_R = memAddr;
    I2C3_MCS_R = 3;                      /* S-(saddr+w)-ACK-maddr-ACK */

    error = I2C_wait_till_done();       /* wait until write is complete */
    if (error) return error;

    /* send data */
    I2C3_MDR_R = data;
    I2C3_MCS_R = 5;                      /* -data-ACK-P */
    error = I2C_wait_till_done();       /* wait until write is complete */
	
    while(I2C3_MCS_R & 0x40);            /* wait until bus is not busy */
    error = I2C3_MCS_R & 0xE;
    if (error) return error;

    return 0;       /* no error */
}

char I2C3_Rd(int slaveAddr, char memAddr, int byteCount, char* data)
{
    char error;
    
    if (byteCount <= 0)
        return '\0';         /* no read was performed */

    /* send slave address and starting address */
    I2C3_MSA_R = slaveAddr << 1;
    I2C3_MDR_R = memAddr;
    I2C3_MCS_R = 3;       /* S-(saddr+w)-ACK-maddr-ACK */
    error = I2C_wait_till_done();
    if (error)
        return error;

    /* to change bus from write to read, send restart with slave addr */
    I2C3_MSA_R = (slaveAddr << 1) + 1;   /* restart: -R-(saddr+r)-ACK */

    if (byteCount == 1)             /* if last byte, don't ack */
        I2C3_MCS_R = 7;              /* -data-NACK-P */
    else                            /* else ack */
        I2C3_MCS_R = 0xB;            /* -data-ACK- */
    error = I2C_wait_till_done();
    if (error) return error;

    *data++ = I2C3_MDR_R;            /* store the data received */

    if (--byteCount == 0)           /* if single byte read, done */
    {
        while(I2C3_MCS_R  & 0x40) {}    /* wait until bus is not busy */
        return 0;       /* no error */
    }
 
    /* read the rest of the bytes */
    while (byteCount > 1)
    {
        I2C3_MCS_R = 9;              /* -data-ACK- */
        error = I2C_wait_till_done();
        if (error) return error;
        byteCount--;
        *data++ = I2C3_MDR_R;        /* store data received */
    }

    I2C3_MCS_R = 5;                  /* -data-NACK-P */
    error = I2C_wait_till_done();
    *data = I2C3_MDR_R;              /* store data received */
    while(I2C3_MCS_R & 0x40);        /* wait until bus is not busy */
    
    return 0;       /* no error */
}
		
void UART_Transmitter(unsigned char data)  
{
    while((UART0_FR_R & (1<<5)) != 0);  /* wait until Tx buffer not full */
	UART0_DR_R = data;                  /* before giving it another byte */
}

void printstring(char *str)
{
  while(*str)
	{
		UART_Transmitter(*(str++));
	}
}
void Delay(unsigned long counter)
{
	unsigned long i = 0;
	
	for(i=0; i< counter*10000; i++);
}

static void PlayNote(const MidiNoteEvent_t* note, PWMModule pwmModule, PWMChannel pwmChannel)
{
	// A zero note or velocity means silence.
	if (note->key == 0 || note->velocity == 0)
	{
		PWM_Disable(pwmModule, pwmChannel);
	}
	else
	{
		// For each note we create 50% duty cycle.
		uint16_t period = Midi_NotePwmPeriods[note->key];
		uint16_t duty = period / 2;
		
		PWM_Configure(pwmModule, pwmChannel, period, duty);
		PWM_Enable(pwmModule, pwmChannel);
	}
}

void SwitchHandler(uint32_t pinMap)
{
	// Disable interrupts for both switches.
	GPIO_DisarmInterrupt(&PINDEF(PORTF, (PinName_t)(PIN0 | PIN4)));
	
	// Record the switch state for the switch task.
	switchPressed_ = (uint8_t)pinMap;
	
	
	if (switchPressed_ == GPIO_PIN_4)
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3^GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1));
	}
	else if (switchPressed_ == GPIO_PIN_0)
	{
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1^GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3));
	}
	
	
	// This will attempt a wake the higher priority SwitchTask and continue
	//	execution there.
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	// Give the semaphore and unblock the SwitchTask.
//	xSemaphoreGiveFromISR(armSemaphore_, &xHigherPriorityTaskWoken);
	xSemaphoreGiveFromISR(sensorSemaphore_, &xHigherPriorityTaskWoken);
	
	// If the SwitchTask was successfully woken, then yield execution to it
	//	and go there now (instead of changing context to another task).
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int IsPlaying()
{
	// If any one track is still playing, then the entire tune is playing.
	uint8_t isPlaying = 0;
	for (int i = 0; i < midi_tune.numTracks; i++ )
	{		
		isPlaying |= trackParams_[i].isPlaying;
	}
	return isPlaying;
}

void vSwitchTask(void *pvParameters)
{
	TickType_t debounceDelay = pdMS_TO_TICKS(250);
	
	for (;;)
	{
		// Block until the switch ISR has signaled a switch press event...
		BaseType_t taken = xSemaphoreTake(armSemaphore_, portMAX_DELAY);
		
		if (taken == pdPASS)
		{
			// Has SW1 (Play/Stop) been pressed?
			if (switchPressed_ & PIN4)
			{
				xSemaphoreGive(sensorSemaphore_);
				
				// Unblock the track tasks or abort them to play/stop.
//				if (!IsPlaying())
//				{
//					for (int i = 0; i < midi_tune.numTracks; i++ )
//					{		
//						trackParams_[i].isPlaying = 1;
//						xSemaphoreGive(trackSemaphore_);
//					}
//				}
//				else
//				{
//					for (int i = 0; i < midi_tune.numTracks; i++ )
//					{	
//						trackParams_[i].isPlaying = 0;
//						xTaskAbortDelay(trackParams_[i].taskHandle);
//					}
//				}			
			}
			else if (switchPressed_ & PIN0)
			{
				for (int i = 0; i < midi_tune.numTracks; i++ )
				{	
					trackParams_[i].isPlaying = 0;
					xTaskAbortDelay(trackParams_[i].taskHandle);
				}
			}
		
			// Wait a bit to debounce the switch.
			vTaskDelay(debounceDelay);
	
			// Rearm interrupts for both switches.
			GPIO_RearmInterrupt(&PINDEF(PORTF, (PinName_t)(PIN0 | PIN4)));
		}
	}
}

void vTrackTask(void *pvParameters)
{
	TrackParams_t* params = (TrackParams_t*)pvParameters;

	for (;;)
	{
		// Block until its time to play...
		BaseType_t taken = xSemaphoreTake(trackSemaphore_, portMAX_DELAY);
		
		if (taken == pdPASS)
		{
			TickType_t xLastWakeTime = xTaskGetTickCount();
			
			const MidiNoteEvent_t* notes = params->track->notes;
			
			// Play each note.  The notes sequence will end with -1 or when playing 
			//	has been stopped (task aborted).
			for (int noteIndex = 0; notes[noteIndex].deltaTime != -1; noteIndex++)
			{
				int deltaTime = notes[noteIndex].deltaTime;
				if (deltaTime != 0)
				{
					// Wait until its time to run the event 
					vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(deltaTime));	
					
					// Stop playing if the track has been aborted.
					if (!params->isPlaying)
					{
						break;
					}
				}
					
				// Mask out the channel bits (lower nibble).
				uint8_t status = 0xF0 & notes[noteIndex].status;
				switch (status)
				{
					case NOTE_ON:
						PlayNote(&notes[noteIndex], params->pwmModule, params->pwmChannel);
						break;

					case NOTE_OFF:
					default:
						PWM_Disable(params->pwmModule, params->pwmChannel);
						break;
				}
			}
			
			// Turn off the sound.
			PWM_Disable(params->pwmModule, params->pwmChannel);
			
			// Indicate that the track has finished.
			// TODO:  Changing isPlaying here could cause a race condition.  Might want
			//	to synchronize access with the SwitchTask.
			params->isPlaying = 0;
			
//			xSemaphoreGive(xSemaphoreCreateBinary());
			xTaskCreate(vSwitchTask, "Switch Task", 100, NULL, 3, NULL);
		}
	}
}

void vSensorTask(void *pvParameters)
{
	for (;;)
	{
		BaseType_t taken = xSemaphoreTake(sensorSemaphore_, portMAX_DELAY);
		if (taken == pdPASS)
		{

			while(!motion_detect()){}
				
			for (int i = 0; i < midi_tune.numTracks; i++ )
			{		
				trackParams_[i].isPlaying = 1;
				xSemaphoreGive(trackSemaphore_);
			}
		}
	}
}

// Initialize the hardware and peripherals...
static int InitHardware(void)
{
	__disable_irq();
	
	PLL_Init80MHz();

	// Must store the frequency in SystemCoreClock for FreeRTOS to use.
	SystemCoreClock = PLL_GetBusClockFreq();

	// These are the digital intputs for the onboard buttons.
	GPIO_EnableDI(PORTF, PIN0 | PIN4, PULL_UP);
	
	// Enable interrupts for SW1 & SW2.
	GPIO_EnableInterrupt(&PINDEF(PORTF, PIN0), 7, INT_TRIGGER_FALLING_EDGE, SwitchHandler);
	GPIO_EnableInterrupt(&PINDEF(PORTF, PIN4), 7, INT_TRIGGER_FALLING_EDGE, SwitchHandler);
	
	// We are going to slow down the PWM clock frequency in order to play notes
	// in the neighborhood of middle-C.
	PWM_SetClockDivisor(64);
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
	
	__enable_irq();
	
	return 0;
}
