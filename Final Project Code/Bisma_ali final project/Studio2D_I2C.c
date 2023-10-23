/* Studio W7-1 Project Code
		Sample code provided for studio to demonstrate I2C function and debugging 
		Uses modified I2C code by Valvano.

		Written by Tom Doyle
		Last Updated: March 4, 2020
		
		Last Updated: March 13, 2023
		Yaser M. Haddara
		Integrated I2C and UART to identify ToF sensor and display to RealTerm
		
		
		Bisma Ali 
		400369507 
*/



#include <stdint.h>
#include <stdio.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"
#include "i2c0.h" //modified version of Valvano i2c0.c  
#include "uart.h"
#include "VL53L1X_api.h"
#include "onboardLEDs.h"

#define dev 0x29





void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // Activate the clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};        // Allow time for clock to stabilize 
	GPIO_PORTM_DIR_R = 0b00000000;       								      // Make PM0 an input (0) 
  GPIO_PORTM_DEN_R = 0b00000001;														// Enable Digital I/O on PMO
	GPIO_PORTM_AFSEL_R &= ~0x01;     								
	GPIO_PORTM_AMSEL_R &= ~0x01;    
	return;
}

void PortH_Init(void){
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;					// activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){}	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0x0F;        									// make PH0 to PH3 output
	GPIO_PORTH_AFSEL_R &= ~0xFF;     									// disable alt funct
	GPIO_PORTH_DEN_R |= 0x0F;        									// enable digital I/O on PH0 to PH3
	GPIO_PORTH_AMSEL_R &= ~0xFF;     									// disable analog functionality on PH0 to PH3	

	return;
}




void RotateCCW_steps(uint32_t steps, uint32_t delay){ 
	
	uint32_t i;
	for(i = 0; i < steps; i++) {
		//depending on state of motor, sets motor to next state going CCW
		uint8_t data = GPIO_PORTH_DATA_R & 0x0F;
		
		if (data == 0x03) {
			GPIO_PORTH_DATA_R = 0x09;
			SysTick_Wait10ms(delay);
		}
		else if (data == 0x06) {
			GPIO_PORTH_DATA_R = 0x03;
			SysTick_Wait10ms(delay);
		}
		else if (data == 0x09) {
			GPIO_PORTH_DATA_R = 0x0C;
			SysTick_Wait10ms(delay);
		}
		else if (data == 0x0C) {
			GPIO_PORTH_DATA_R = 0x06;
			SysTick_Wait10ms(delay);
		}
		else {													
			GPIO_PORTH_DATA_R = 0x03;
			SysTick_Wait10ms(delay);
		}
	}

	return;
}


void RotateCW_steps(uint32_t steps, uint32_t delay){
	
	uint32_t i;
	for(i = 0; i < steps; i++) {

		uint8_t data = GPIO_PORTH_DATA_R & 0x0F;
		
		if (data == 0x03) {
			GPIO_PORTH_DATA_R = 0x06;
			SysTick_Wait10ms(delay);
		}
		else if (data == 0x06) {
			GPIO_PORTH_DATA_R = 0x0C;
			SysTick_Wait10ms(delay);
		}
		else if (data == 0x09) {
			GPIO_PORTH_DATA_R = 0x03;
			SysTick_Wait10ms(delay);
		}
		else if (data == 0x0C) {
			GPIO_PORTH_DATA_R = 0x09;
			SysTick_Wait10ms(delay);
		}
		else {													
			GPIO_PORTH_DATA_R = 0x03;
			SysTick_Wait10ms(delay);
		}
	}

	return;
}

void ControlRotation(int dir, int Spins){ 
	
	if(dir == 0){ 
		for (int i = 0; i< Spins; i++){ 
		RotateCW_steps(1,1);
		} 
	}else if(dir == 1){ 
		for (int i = 0; i< Spins; i++){ 
		RotateCCW_steps(1,1);
		} 
	}
}


void returnhome(int location, int dir){ 
	int direction = 0; 
	if (dir == 0) direction = 1; 
	ControlRotation(direction,location);  
} 

int main(void) {

	int status = 0;
	uint8_t modelID, modelType;
	uint16_t bothType;
	PLL_Init();
	SysTick_Init();
	I2C_Init();
	UART_Init();
	onboardLEDs_Init();
	//FlashLED2(1);
	PortH_Init();
	PortM_Init();	
	//PortN_Init();

	
	UART_printf("Studio 7D - Identify ToF Sensor:\r\n");
	
	
	VL53L1_RdByte(0x29, 0x010F, &modelID);
	SysTick_Wait10ms(50);
	VL53L1_RdByte(0x29, 0x0110, &modelType);
	SysTick_Wait10ms(50);
	VL53L1_RdWord(0x29, 0x010F, &bothType);
	SysTick_Wait10ms(50);
	
	sprintf(printf_buffer, "Model ID = 0x%x, Model Type = 0x%x, Both Type = 0x%x\r\n\n", modelID, modelType, bothType);
	UART_printf(printf_buffer); //USE FOR MILESTONE ONE
	
	uint8_t byteData, sensorState=0, i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	
	int current_state = 0; // on (1) or off (0) 
	int Distancefromhome = 0; // number of 4-step increments travelled
	int direction = 0; 
	//GPIO_PORTN_DATA_R = 0x0; // start with LED off
	
		
	/* Those basic I2C read functions can be used to check your own I2C functions */
		// hello world!
		

		

	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"2DX ToF Final Project %d\r\n",mynumber);
	UART_printf(printf_buffer);
	
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);

	UART_printf(printf_buffer);



	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }

	//FlashAllLEDs();
	//UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInititalize", status);
	
	status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging
	
	sprintf(printf_buffer,"communicationready ");

	UART_printf(printf_buffer);

	
	
	// ACTIVE HIGH LOGIC (button press detected when PM0 is driven LOW)
	
	while(1){
		
		
		if((GPIO_PORTM_DATA_R&0b00000001)==0){ // button 0 pressed
			current_state ^= 1;

			FlashLED2(10); 
			if (Distancefromhome != 0){ 
				returnhome(Distancefromhome,direction); 
			}
			Distancefromhome = 0; // reinitialize Distancefromhome here (turn on/off resets it)
			while((GPIO_PORTM_DATA_R != 0x1)){}; // wait for button release
			SysTick_Wait10ms(1); // short delay

				SysTick_Wait10ms(1);

		}

	
	if(current_state==1){ // it ON, so we wanna spin motor according to config
		

			ControlRotation(direction, 1);
			Distancefromhome ++;
					
if (Distancefromhome% 32 == 0){ 
		//wait until the ToF sensor's data is ready
	  while (dataReady == 0){
		  status = VL53L1X_CheckForDataReady(dev, &dataReady);
          //FlashLED3(1);
          VL53L1_WaitMs(dev, 5);
	  }
		dataReady = 0;
	  
		//read the data values from ToF sensor
		status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
	  status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
		status = VL53L1X_GetSignalRate(dev, &SignalRate);
		status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
		status = VL53L1X_GetSpadNb(dev, &SpadNum);
    
		//FlashLED4(1);

	  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
		
		sprintf(printf_buffer,"%u \r\n", Distance);

		UART_printf(printf_buffer);
	  SysTick_Wait10ms(50);
  
	
}
	if(Distancefromhome == 2048){
			current_state=0;
			direction ^= 1; 
		 Distancefromhome = 0; 
		
		}

	}

}

}
