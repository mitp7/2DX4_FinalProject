/*
	Name: Mit Patel
	Student Number: 400308098
	MacID: patem97
	Bus Speed: 40MHz
	LED: D1
*/

#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"


#define I2C_MCS_ACK             0x00000008             // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008             // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004             // Acknowledge Address
#define I2C_MCS_STOP            0x00000004             // Generate STOP
#define I2C_MCS_START           0x00000002             // Generate START
#define I2C_MCS_ERROR           0x00000002             // Error
#define I2C_MCS_RUN             0x00000001             // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001             // I2C Busy
#define I2C_MCR_MFE             0x00000010             // I2C Master Function Enable
#define MAXRETRIES              5                      // number of receive attempts before giving up

// Initialized port M for active low push button (Stop)
void PortM_Init(void){
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;				     // Activate clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};	 // Allow time for clock to stabilize
	GPIO_PORTM_DIR_R |= 0x00	;        								   // Making PM0 an input  
  GPIO_PORTM_AFSEL_R &= ~0xFF;     								     // Disable alt funct on PN0
  GPIO_PORTM_DEN_R |= 0xFF;        								     // Enable digital I/O on PN0
  GPIO_PORTM_AMSEL_R &= ~0xFF;     								     // Disable analog functionality on PN0		
	return;
}

// Initialized port J for on board start button
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					   // Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	   // Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    										   // Make PJ1 in 
  GPIO_PORTJ_DEN_R |= 0x02;     										   // Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 					    		 // Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;									   		 // Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;									    			 //	Enable weak pull up resistor
}

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           		 // Activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          	 // Activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};							 // Ready?
  GPIO_PORTB_AFSEL_R |= 0x0C;           							 // 3) Enable alt funct on PB2,3       0b00001100
  GPIO_PORTB_ODR_R |= 0x08;             							 // 4) Enable open drain on PB3 only
  GPIO_PORTB_DEN_R |= 0x0C;             							 // 5) Enable digital I/O on PB2,3
  // GPIO_PORTB_AMSEL_R &= ~0x0C;          						 // 7) Disable analog functionality on PB2,3

                                                       // 6) Configure PB2,3 as I2C
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
  I2C0_MCR_R = I2C_MCR_MFE;                      		   // 9) Master function enable
  I2C0_MTPR_R = 0b0000000000000101000000000111011;     // 8) Configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
  // I2C0_MTPR_R = 0x3B;                               // 8) Configure for 100 kbps clock
        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
  //Use PortG0
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;             // Activate clock for Port N
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // Allow time for clock to stabilize
  GPIO_PORTG_DIR_R &= 0x00;                            // Make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                         // Disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                            // Enable digital I/O on PG0
  GPIO_PORTG_AMSEL_R &= ~0x01;                         // Disable analog functionality on PN0
  return;
}

// Intialize port H for the stepper motor
void PortH_Init(void){
	//Use PortH pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;				     // Activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	   // Allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0xFF;        								     // Make PH0 out (PH0 built-in LED1)
  GPIO_PORTH_AFSEL_R &= ~0xFF;     								     // Disable alt funct on PH0
  GPIO_PORTH_DEN_R |= 0xFF;        								     // Enable digital I/O on PH0
																									     // Configure PH1 as GPIO
                                                       // GPIO_PORTH_PCTL_R = (GPIO_PORTH_PCTL_R&0xFFFFFF0F)+0x00000000;
  GPIO_PORTH_AMSEL_R &= ~0xFF;     								     // Disable analog functionality on PH0		
	return;
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortL_Init(void){
  //Use Port M
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;             // Activate clock for Port N
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};    // Allow time for clock to stabilize
  GPIO_PORTL_DIR_R &= 0x04;                            // Make PG0 in (HiZ)
  GPIO_PORTL_DIR_R |= 0x04;                            // Make PG0 in (HiZ)
		
	GPIO_PORTL_AFSEL_R &= ~0x07;                         // Disable alt funct on PG0
  GPIO_PORTL_DEN_R |= 0x07;                            // Enable digital I/O on PG0
  GPIO_PORTL_AMSEL_R &= ~0x07;                         // Disable analog functionality on PN0
  return;
}

	
//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                          // Make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                   // PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                         // Make PG0 input (HiZ)
    
}

void clockwise(){
		for(int i=0; i<64; i++){						               // 360 degrees is 512 -> 45 degrees is 64 steps
			GPIO_PORTH_DATA_R = 0b00001001;			             
			SysTick_Wait10ms(1);								
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait10ms(1);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait10ms(1);
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait10ms(1);
	}
}

void counterClockwise(){
		for(int i=0; i<64; i++){					
			GPIO_PORTH_DATA_R = 0b00001100;		
			SysTick_Wait10ms(1);							
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait10ms(1);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait10ms(1);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait10ms(1);
		}
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			// Address of the ToF sensor as an I2C slave peripheral
int status = 0;
int direction = 0;        // Change direction of motor


int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	uint8_t id;
	uint8_t type;
	uint16_t both;

	// Initialized all of the ports and clock
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH_Init();
	PortJ_Init();
	PortM_Init();

	//PortL_Init();
	
	//for(int i = 0; i < 5000000; i++){ //1+ PL2, 1-gnd
		//GPIO_PORTL_DATA_R ^= 0b00000100;
		//SysTick_Wait10ms(1);
	//}

	// Booting ToF chip
	while(sensorState==0){
		// Function below is used to check that the sensor has booted, while loop to ensure the sensor finishes booting before the first I2C access
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	//UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	// This function has to be called to enable the ranging and start measurements
  status = VL53L1X_StartRanging(dev);  


	// Tracks number of scans
	int loopCounter = 0;

	// While loop used to iterate to 3 measurements
	while(loopCounter < 3){
		
		// Conditional statment used to detect a button press (PJ1 onboard button)
		if(GPIO_PORTJ_DATA_R == 0b00000000){
			loopCounter++;
			
			// Loop to iterate 8 times, covering 45 degrees each on the motor and makes 8 distance measurements
			for(int i = 0; i < 8; i++) {
				
				// Conditional used to detect stop button 
				if((GPIO_PORTM_DATA_R&0b00000001)==0){
					// Arbitrary value to break out of while loop
					loopCounter = 10;
          break;
        }else{
        }
				
				// Wait until the ToF sensor's data is ready
				while (dataReady == 0){
					
					// Recieves notification that ranging data are ready
					status = VL53L1X_CheckForDataReady(dev, &dataReady);
					
					// Flash led D1
					FlashLED1(1); 
					VL53L1_WaitMs(dev, 5);
				}
				
				dataReady = 0;
				// Recieves the data and stores in the Distance variable
				status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value

				// Rotates the motor
				if(direction %2 ==0){
					clockwise();	
				}else {
					counterClockwise(); 
				}
				
				status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
				// Print the resulted readings to UART
				sprintf(printf_buffer,"%u\n",Distance);
				UART_printf(printf_buffer);
				SysTick_Wait10ms(50);
			}
		}else{
			SysTick_Wait10ms(50);
		}
		direction++;
	}
	VL53L1X_StopRanging(dev);
  while(1) {}
}

