/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "UTILS.h"
#include<stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define game1 49
#define game2 50

//defines for the melody generation
#define len 3 //the number of notes
#define pulses_half_second 2000 //0.5s, duration of each note

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

void ON_LED1();
void OFF_LED2();

void ON_LED2();
void OFF_LED2();

void OFF_LEDS(); //turn off all LEDs
//void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
uint8_t function_random(int upper_boundary, int seed);
void TIM4_IRQHandler(void);
void write_message(unsigned char* message, unsigned char message_length);

void delay(unsigned int time);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Common variables for both games
unsigned char game_choice;
unsigned char recieved[7]="      ";
unsigned char text[6];

//Common variables for both games : time measurement variables
unsigned short start_time_g1, present_time_g2; //initial time for the game1 (moment when LED turned on), present time for game2 (when the counter reaches zero)
int time_player1;	//duration of pressing for player1
int time_player2;	//duration of pressing for player2
uint8_t random_time;  //random time to be used in game 1 or 2

//Game 2 variables
unsigned char i ,End_Of_Melody;
unsigned int potent_value;
unsigned int difficulty;  //level of difficulty to be changed by the user
char sign;
//GAME 2 : melody variables
uint8_t count, state, change;
uint16_t *music; 		//pointer to change which melody is being played
uint16_t melody1[len]={4000,2000,1000};  // OCTAVE 3 (DO 125Hz, SI 250Hz) OCTAVE 4(SI 500Hz)
uint16_t melody2[len]={1000,1000,2000};  // OCTAVE 4  (SI 500Hz,SI 500Hz) OCTAVE 3 (DO 125Hz)

//Ultra sound variables
unsigned char new_measurement =0;	// variable for the question "has a new measuremnt been captured"
int techo1,techo;		//for the echo pulse duration measurement
						//techo1 (rising edge, aka initial time),
						// techo duration of the pulse
unsigned char has_10us_passed=0;  //variable for the time measurement using TIM2
								   //has_10us_passed=0, 10 us has passed
								   //has_10us_passed=1, 100ms has passed

int counter=0;
const float speedOfSound = 0.0343/2;
void ON_LED1() {
//the LED is in  PA12
	GPIOA->BSRR |= (1 << 12);
}

void OFF_LED1() {
	GPIOA->BSRR |= (1 << 28);
}

void ON_LED2() {
//the LED is in PC10
	GPIOC->BSRR |= (1 << 10);
}
void OFF_LED2() {
	GPIOC->BSRR |= (1 << 10) << 16;
}

void OFF_LEDS(){
	OFF_LED1() ;
	OFF_LED2() ;
}

// generate a random number between 1 and  upper_boundary
uint8_t function_random(int upper_boundary, int seed) {

	uint8_t randNumber;

	//initialise the seed
	srand(seed);

	randNumber=(rand() % (upper_boundary-1+ 1))+1;

	return randNumber;
}

void EXTI15_10_IRQHandler(void) {
	uint32_t readings = EXTI->PR; //reading the state register of exxti
	uint32_t button_user_pressed = readings & (1 << 13); //verify bit 13

	if (button_user_pressed == (1 << 13)) {
		if (game_choice == game1)
			game_choice = game2;
		else
			game_choice = game1;

		EXTI->PR |= (1 << 0); // clear flag
	}
}

////////////////// Handler for milestone 2
void TIM4_IRQHandler(void) {

	/// !!! DISCLAIMER !!! : In this part we do not deal with the antirebounce effect since
	///						we consider that the time to have access to the handler is enough for that

	uint32_t readings = TIM4->SR; //retreiving the value of the interruption
	// The following lines are a step to be sure that the flage that rose is associated with the right channel
	uint32_t button_1_edge = readings & (1 << 1); //button_1_edge has the condition if button1 has changed value
	uint32_t button_2_edge = readings & (1 << 2); //button_2_edge has the condition if button2 has changed value


	///////handling block for the player 1
	if (button_1_edge == (1 << 1)) // If the event is CH1, button PB6 pressed (rising edge)
	{
		time_player1 = TIM4->CCR1;	// save the time when the button was pressed
		TIM4->SR &= ~(0x0002);
	}

	///////handling block for the player 2
	if (button_2_edge == (1 << 2)) // If the event is CH2,  button PB7 pressed (rising edge)
	{
		time_player2 = TIM4->CCR2;	// save the time when the button was pressed
		TIM4->SR &= ~(0x0004);
	}

	TIM4->SR = 0x0000; // Clear all flags (although only CH1 is expected and already cleared)
}

////////////////// Handlers for milestone 3

void TIM2_IRQHandler(void)
{
	uint32_t readings= TIM2->SR;
	uint32_t channel1_irq = readings & (1 << 1);
	uint32_t channel2_irq = readings & (1 << 2);
	uint32_t channel3_irq = readings & (1 << 3);

	if (channel1_irq ==(1<<1)){	//Melody mode ?
		TIM2->SR=0;
		TIM2->CNT=0;
	}

	if (channel2_irq==(1<<2)){		//Ultrasound mode (counting base) ?
		if (has_10us_passed==1){	//10us has passed
			TIM2->SR&=~(1<<(2));
			GPIOA->BSRR |= ((1 << 1)<<16); //clear pin PA1
			TIM2->CCR2+=50000;		//Update CCR2 to count until 100 ms
			has_10us_passed=0;
		}
		else{					//
			TIM2->SR&=~(1<<(2));
			GPIOA->BSRR |= (1 << 1); //set pin PA1
			TIM2->CCR2+=5;		//Update CCR2 to count until 10 us
			has_10us_passed=1;
		}
	}
	if(channel3_irq==(1<<3)){   //Ultrasound mode (measurement) ?

		if ((GPIOB->IDR&(1<<1*10))!=0){	// If rising edge
			techo1=TIM2->CCR3;			// time when the pulse starts
		}
		else{							//falling edge
			techo=TIM2->CCR3-techo1;	//techo=pulse duration
			if (techo<0) techo+= 0x0FFFF;
			new_measurement=1;
			//counter++;
		}
		TIM2->SR&=~(1<<(3)); //PB10
	}

}


//handler for the duration of the tone
void TIM3_IRQHandler(void)
{
	uint32_t readings= TIM3->SR;
	//if the duration of the tone has passed in channel 2
	uint32_t channel2_irq = readings & (1 << 2);
	if (channel2_irq==(1 << 2)&&((sign=='+')||(sign=='-')))// check if CH2 triggered and whether we are in
										   // the melody block code
										   // count==0 only in that block and the one where
										   // the melody has finished
	{
		TIM3->SR=0;
		count++; 						//increment the frequency to be generated
		if(count>=len)
		{
			count=0; 					//if we've reached the end of the melody reset the counter
			End_Of_Melody=1;
		}
		TIM2->CCR1=music[count];		//Initialise the TIM2 to play the following melody
		TIM2->CNT=0; 					//reset counters of TIM2 AND TIM3
		TIM3->CNT=0;
	}
	//TIM3->SR=0;
	TIM3->SR=0;
}

void write_message(unsigned char* message, unsigned char message_length){
	unsigned char j=0;
	for (j=0;j< message_length;j++){
		HAL_UART_Transmit(&huart2, message+j, 1, 10000);
	}
}
void delay(unsigned int time){ //delay function using timer 9 (time in ms)
	TIM9->CCR1 = time;
	TIM9->CNT=0;
	TIM9->CR1 |= 0x0001;
	while ((TIM9->SR&0x0002)==0);
	TIM9->SR = 0;
	TIM9->CR1 &= ~(0x0001);

}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  recieved[0]=0;
  HAL_UART_Receive_IT(&huart2, recieved,1);

	//Conifgure the user button PC13 digital input 00
	GPIOC->MODER &= ~(1 << (13 * 2 + 1)); //coloco 0
	GPIOC->MODER &= ~(1 << (13 * 2)); //coloco 0

	//////////////////////////////////////////// Configuration of leds
	//PA12 as digital output . LED1
	GPIOA->MODER &= ~(1 << (12 * 2 + 1)); //coloco 0
	GPIOA->MODER |= (1 << (12 * 2)); //coloco 1

	//PC10 as digital output . LED2 !!!HAD TO CHANGE IT, PD2 DOES NOT FUNCTION
	GPIOC->MODER &= ~(1 << (10 * 2 + 1)); //coloco 0
	GPIOC->MODER |= (1 << (10 * 2)); //coloco 1
	/* !!!!! SECOND MILESTONE CONFIGURATION */

	////////////configuration of the pins
	//Configure PB6 in  AF configuration 10
	GPIOB->MODER |= 1 << (2 * 6 + 1);
	GPIOB->MODER &= ~(1 << (2 * 6));
	GPIOB->AFR[0] |= (0x02 << (6 * 4)); //AFR[0] to associate PB6 with AF2 (TIM4,CH1)

	//Configure PB7 in  AF configuration 10
	GPIOB->MODER |= 1 << (2 * 7 + 1);
	GPIOB->MODER &= ~(1 << (2 * 7));
	GPIOB->AFR[0] |= (0x02 << (7 * 4)); //AFR[0] to associate PB7 with AF2 (TIM4,CH2)

	//Configure PB6 in PULLDOWN (10)
	GPIOB->PUPDR &= ~(1 << (6 * 2));
	GPIOB->PUPDR |= (1 << (6 * 2 + 1));
	//Configure PB7 in PULLDOWN (10)
	GPIOB->PUPDR &= ~(1 << (7 * 2));
	GPIOB->PUPDR |= (1 << (7 * 2 + 1));

	////////////////////////Configuration of the timer 4 (used for capturing the times of the players) //

	TIM4->CR1 = 0;  					//CEN =0 counter disabled,

	TIM4->CR2 = 0; 					//always zero in this class
	TIM4->SMCR = 0;					//always zero in this class
	TIM4->DIER |= 0x000E; // interrupt for channel 1,2,3 enabled (CC3IE=1,CC2IE=1,CC1IE=1) (ch1,2<=>player1,2)(ch3<=>echo signal measurement)
	TIM4->CCMR1 |= 0x0101; 			//Setting channel 1 & 2 as TIC, CCyS = 01


	TIM4->CCER |= 0x0033;		// rising edge interrupt enabled (ch1 &ch2), capture enabled
								// Capture enabled CCyE=1

	TIM4->CNT = 0;					//Initial value of the counter
	TIM4->PSC = 31999;				//Tu= 1ms
	TIM4->ARR = 0xFFFF;	// Recommendation, in this case we needn't a value of reset, we measure time
	//Counter enabling


	TIM4->EGR |= 0x0001;				// UG =1 Clear and refresh the counter
	TIM4->SR = 0;						//clearing the counter flags

	// Interrupts enabling TIM4_IRQHandler at NVIC (position 30)
	NVIC->ISER[0] = (1 << 30);



	//configuring the interruption of the pin PC13 with rising edge
	SYSCFG->EXTICR[3] &= ~(0x0F << (1 * 4));
	SYSCFG->EXTICR[3] |= (2 << (1 * 4));

	EXTI->RTSR |= (1 << 13); // activate rising edge PC13
	EXTI->FTSR &= ~(1 << 13); // deactivate falling edge PC13
	EXTI->IMR |= (1 << 13); // unmasking the interrupt request
	NVIC->ISER[1] |= (1 << (40 - 32)); //NVIC enabling the exti 15_10
	/////////////////////////////////////Configuration of the timer used for generating melody PA5 (AF01) TIM2 CH1

	//Channel 2  is going to be used to measure time TOC without output (5us and 100ms)
	//CHANNEL 3  is going to be used for measuring the duration of the signal echo (PB10,AF01)
	// PIN PA1 is going to be used to generate the output trigger
	//Configure PA1 as digital output 01
	GPIOA->MODER&=~(1<<(1*2+1));
	GPIOA->MODER|=(1<<(1*2));
	//Configure PA5 as an alternate function
	GPIOA->MODER&=~(1<<(5*2));
	GPIOA->MODER|=(1<<(5*2+1));
	// Configure PA5 to AF01 (0001)
	GPIOA->AFR[0]|=(1<<(5*4));
	//Configure PB10 as an alternate function to read techo
	GPIOB->MODER&=~(1<<(10*2));
	GPIOB->MODER|=(1<<(10*2+1));
	//Configure PB10 to AF01 (0001)
	GPIOB->AFR[1]|=(1<<((2*4)));
	//Configure the timer
	TIM2->CR1 = 0; 					// ARPE = 0 -> Not PWM, it is TOC
									// CEN = 0; Counter off
	TIM2->CR2 = 0; 					// Always 0x0000 in this subject
	TIM2->SMCR = 0;					// Always 0x0000 in this subject
	// Setting up the counter functionality : PSC,CNT,ARR
	TIM2->PSC=61; 					//tu = 2 us
	TIM2->CNT = 0;  				//Initial value for CNT
	TIM2->ARR = 0xFFFF; 			// Recommended value =FFFF
	//IRQ SELECTON
	TIM2->DIER = 0x000E;			//IRQ enabled only for CH1, CC1IE=1; CH2 CC2IE=1 and CH3 CC3IE=1
	TIM2->CCMR1 = 0x0030;			//OC1M=011 toggle
									//CC1S=00 TOC,OC1E=0,OC1PE=0 (not PWM,TOC)
									//CC2S=00 TOC, OC2M=000 timing base
	TIM2->CCMR2 |= 0x0001; 			//Setting channel 3 as TIC, CC3S = 01
	TIM2->CCR2=5;					//10us
	TIM2->CCER = 0x0B00;			//CC1NP=CC1P=0(TOC),CC1E=1, output enabled
									//CC2NP=CC2P=0(TOC),CC2E=0, output disabled
									//CC3NP=CC3P=1(TIC)rising and falling edge interrupt enabled (CH3), capture enabled CC3E =1
	TIM2->EGR |= 0x0001;			//UG=1 Update event
	TIM2->SR = 0;					// Clear all flags
	NVIC->ISER[0] |= (1 << 28);		//NVIC enabling the TIM2 GLOBAL IRQ

	/////////////////////////timer for the delays

	// Internal clock selection: CR1, CR2, SMRC
	TIM9->CR1 = 0; 			// ARPE = 0 -> Not PWM, it is TOC
							// CEN = 0; Counter off
	TIM9->CR2 = 0; 			// Always 0x0000 in this subject
	TIM9->SMCR = 0;			// Always 0x0000 in this subject
	// Setting up the counter functionality : PSC,CNT,ARR,and CCR1
	TIM9->PSC = 31999;		//Time unit = 1 ms
	TIM9->CNT = 0;			//Initial value for CNT
	TIM9->ARR = 0xFFFF;		// Recommended value =FFFF
	TIM9->CCR1 = 3000;		// 3 seconds (preload value since we are in a toc configuration
	//IRQ SELECTON
	TIM9->DIER = 0x0004;	//IRQ enabled only for AND CH2, CC1IE=1,CC2IE=1
	// Counter output mode
	TIM9->CCMR1 = 0x0000;	//CC1S=CC2S=0 (TOC)
							//OC1M=OC2M = 000 (no external output)
							//OC1PE=OC2PE =0 (no preload)

	TIM9->CCER = 0;			//CC1P=CC2P= 0 always for TOC
							//CC1E=CC2E =0 (no output)

	// Enabling the counter

	TIM9->CR1 |= 1;			//CEN=1 starting the counter
	TIM9->EGR |= 1;			//UG=1 Update event
	TIM9->SR = 0;			// Clear all flags



	////////////////////////////////////Configuration of the timer used as a counter TIM3 CH1*

	// Internal clock selection: CR1, CR2, SMRC
	TIM3->CR1 = 0; 			// ARPE = 0 -> Not PWM, it is TOC
							// CEN = 0; Counter off
	TIM3->CR2 = 0; 			// Always 0x0000 in this subject
	TIM3->SMCR = 0;			// Always 0x0000 in this subject
	// Setting up the counter functionality : PSC,CNT,ARR,and CCR1
	TIM3->PSC = 31999;		//Time unit = 1 ms
	TIM3->CNT = 0;			//Initial value for CNT
	TIM3->ARR = 0xFFFF;		// Recommended value =FFFF
	TIM3->CCR1 = 30000;		// 3 seconds (preload value since we are in a toc configuration
	//IRQ SELECTON
	TIM3->DIER = 0x0004;	//IRQ enabled only for CH2,CC2IE=1
	// Counter output mode
	TIM3->CCMR1 = 0x0000;	//CC1S=CC2S=0 (TOC)
							//OC1M=OC2M = 000 (no external output)
							//OC1PE=OC2PE =0 (no preload)

	TIM3->CCER = 0;			//CC1P=CC2P= 0 always for TOC
							//CC1E=CC2E =0 (no output)
	NVIC->ISER[0] |= (1 << 29);		//NVIC enabling the TIM3 GLOBAL IRQ


	// Enabling the counter

	TIM3->CR1 |= 1;			//CEN=1 starting the counter
	TIM3->EGR |= 1;			//UG=1 Update event
	TIM3->SR = 0;			// Clear all flags

	///////////////Configuration of the ADC

	//PA4 as analog
	GPIOA->MODER |= (1 << (4 * 2 + 1));
	GPIOA->MODER |= (1 << (4 * 2));

	ADC1->CR2 &= ~(1 << (1 * 1));	//ADON = 0 (ADC powerd off)
	ADC1->CR1 = 0x00000000;	// OVRIE =0 (overrun IRQ disabled)
							//00 resolution = 12 bits
							// 0-Scan disabled
							// 0- EOCIE : end of conversion interrupt

	ADC1->CR2 = (1 << (10 * 1));// EOCS =1 (EOC to be activated after each conversion)
								//DELS = 000 (no delay)
								//CONT =0 (single conversion)
	ADC1->SQR1 = 0x00000000;	// 1 channel in the sequence
	ADC1->SQR5 = 0x00000004;	// Channel is AIN4
	ADC1->CR2 |= 0x00000001;	//ADON = 1 (ADC powered on)



	//////////////////////////////////////////// Initialisation of variable


	game_choice = game1;
	difficulty=1000;
	End_Of_Melody=0;
	recieved[0]=0;
	has_10us_passed=1;
	/* USER CODE END SysInit */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		delay(3000);
		//if needed turn off all LEDS
		OFF_LEDS();
		TIM2->CR1 |= 1;			//CEN=1 starting the counter
		GPIOA->BSRR |= (1 << 1);

		write_message((unsigned char*)"\r\n Welcome, pass your hand alongside the Ultrasonic module\r\n", sizeof("\r\n Welcome, pass your hand alongside the Ultrasonic module\r\n"));

		delay(2000);
		counter+=0;
		while (new_measurement==0);
		new_measurement=0;
		has_10us_passed=0;
		float distance = (techo*2* 100) / 5882;//from clock cycles to microseconds
		printf ("\r\n Distance measured %f cm \r\n",distance);

		delay(1000);
		if (1<=distance&&distance<=7){ // start the game


						//WAIT 3 SECONDS
						delay(3000);

						unsigned char message[]="\n Welcome to the game of reflexes, select: game 1 REACTION TIME (press 1), game 2 COUNTDOWN (press 2)\r\n";

						write_message(message, sizeof(message));
						while(recieved[0]==0);


						// PRESS the user button here
						game_choice=recieved[0];

						recieved[0]=0;

						switch (game_choice) {
							case game1: //GAME 1



									write_message((unsigned char*)"\r\n game 1 \r\n", sizeof("\r\n game 1 \r\n"));


										TIM4->CR1 |= 0x0001;			//CEN=1 counter ENabled
										recieved[0]=0;

										//initialisation phase
										time_player2 =0;
										time_player1=0;

										random_time = function_random(5,(int)TIM4->CNT);  // random time between 1 and 5 ms
										//Waiting time
										delay(random_time*1000);

										start_time_g1 = TIM4->CNT;		//get the present time

										ON_LED1();

										//Wait 1s
										delay(1000);

										OFF_LED1();

										//Deciding which player has won
										if (time_player1 || time_player2) {	//First detect if at least one of them has pressed the button
											if (time_player1) {
											// To resolve the sign issue
												if (time_player1 > start_time_g1) {
													time_player1 -= start_time_g1;//If the player 1 has pressed, determine after how long has he/she pressed
												} else {
													time_player1 = start_time_g1- time_player1;
												}
											}
											if (time_player2) {
											// To resolve the sign issue
												if (time_player2 > start_time_g1) {
													time_player2 -= start_time_g1;//If the player 2 has pressed, determine after how long has he/she pressed
												} else {
													time_player2 = start_time_g1- time_player2;
												}
											}
											if (((time_player1 > time_player2)&&(TIM4->CCR2))||((time_player2 > time_player1)&&(!(TIM4->CCR1)))) {//If the time after which player 1 has pressed the button is longer
																							  //the condition TIM4->CCR2 is added as a guarantee that the player 2
																							  //has pressed his/her button (avoid the situation where time_player2=0)
												ON_LED2();
												Bin2Ascii(time_player2, &text[0]);
												write_message((unsigned char*)"2 ",sizeof("2 "));
												write_message(text,sizeof(text));
												write_message((unsigned char*)"\n\r",sizeof("\n\r") );


												}
											if (((time_player2 > time_player1)&&(TIM4->CCR1))||((time_player1 > time_player2)&&(!(TIM4->CCR2)))){//If the time after which player 2 has pressed the button is longer
																							  //the condition TIM4->CCR2 is added as a guarantee that the player 2
																							  //has pressed his/her button (avoid the situation where time_player1=0)
												ON_LED1();


												Bin2Ascii(time_player1, &text[0]);
												write_message((unsigned char*)"1 ",sizeof("1 "));
												write_message(text,sizeof(text));
												write_message((unsigned char*)"\n\r",sizeof("\n\r") );



											}
										}
										else {
											write_message((unsigned char*)"\r\nEND of the game no winner no loser\r\n",sizeof("\r\nEND of the game no winner no loser\r\n") );
	//

										}



									break;
							case game2: //GAME 2

								write_message((unsigned char*)"\r\n game 2 \r\n",sizeof("\r\n game 2 \r\n") );


									//initialisation
									time_player2 =0;
									time_player1=0;

									write_message((unsigned char*)"\r\nPlease choose the difficulty level \r\n",sizeof("\r\nPlease choose the difficulty level \r\n") );
									write_message((unsigned char*)"You have 2 seconds to do so \r\n",sizeof("You have 2 seconds to do so \r\n") );


									//wait 2 seconds for user interface easiness
									delay(2000);

									write_message((unsigned char*)"\t Easy (countdown 2s) \r\n",sizeof("\t Easy (countdown 2s) \r\n") );
									write_message((unsigned char*)"Potentiometer value [0;1330]\r\n",sizeof("Potentiometer value [0;1330]\r\n") );

									write_message((unsigned char*)"\t Normal (countdown 1s) \r\n",sizeof("\t Normal (countdown 1s) \r\n") );
									write_message((unsigned char*)"Potentiometer value [1330;2600]\r\n",sizeof("Potentiometer value [1330;2600]\r\n") );


									write_message((unsigned char*)"\t Hard (countdown 0.5s) \r\n",sizeof("\t Hard (countdown 0.5s) \r\n") );
									write_message((unsigned char*)"Potentiometer value [2600;4000]\r\n",sizeof("Potentiometer value [2600;4000]\r\n") );


									//wait 2 seconds
									delay(2000);


									 // start conversion
									 while ((ADC1->SR&(1<<(6*1)))==0); //While ADONS = 0, i.e, ADC is not ready
									 // to convert, I wait
									 ADC1->CR2|=(1<<30*1);   		  //When ADONS = 1, I start conversion
									 //(SWSTART=1)
									 // Wait till conversion is finished
									 while ((ADC1->SR&0x0002)==0);   // If EOC = 0, i.e., the conversion is not
									 // finished, I wait
									 potent_value=ADC1->DR;			  // When EOC=1, I take the result and store it in
									 // variable called value
									 //Convert result to string
									 Bin2Ascii(potent_value,text);

									 write_message((unsigned char*)"potentiometer value :",sizeof("potentiometer value :") );
									 write_message((unsigned char*)text,sizeof(text) );
									 write_message((unsigned char*)"\n\r",sizeof("\n\r") );


									 if (potent_value <1330){
										 write_message((unsigned char*)"You have chosen the easy level \r\n",sizeof("You have chosen the easy level \r\n") );

										 //configure the timer,
										 difficulty=2000;

									 }

									 if(1330<=potent_value && potent_value<2600  ){
										 write_message((unsigned char*)"You have chosen the normal level \r\n",sizeof("You have chosen the normal level \r\n") );

										 //configure the timer,
										 difficulty=1000;

									 }

									 if(2600<=potent_value && potent_value<=4095  ){
										 write_message((unsigned char*)"You have chosen the Hard level \r\n",sizeof("You have chosen the Hard level \r\n") );

										 //configure the timer,
										  difficulty=500;

									 }


									random_time = function_random(3,(int)TIM4->CNT); // random time between 1 and 3, in this casE it'll be the moment
																					// starting from which the countdown message will disappear
									//configure the difficulty

									delay(difficulty);// input difficulty

									TIM4->CR1 |= 0x0001;			//CEN=1 counter ENabled

									unsigned char k=57;		//57 is the ascii code of the char 9, no code ascii for 10
									for (i =10 ; i> 0; i--) {

										if (i==10){
											write_message((unsigned char*)("10\n\r"),sizeof("10\n\r") );
											delay(difficulty);

										}
										if (i <= random_time) {
											write_message((unsigned char*)" \n\r",sizeof(" \n\r") );
											delay(difficulty);

										} else if (i!=10) {
											write_message((unsigned char*)(&k),sizeof(k) ); // the respective values of k and i are the same from when i=9
											write_message((unsigned char*)"\n\r",sizeof("\n\r") );
											k--;		//decreasing its value until it reaches 49 (ascii code of 1)

											delay(difficulty);

										}
										if (i==1){
											//Store the value of the present time
											present_time_g2 = TIM4->CNT;
//											delay(difficulty);
										}
										//Wait 1 se before next iteration

									}

									// Waiting 2 seconds after the end of the countdown
									delay(2000);

									//Deciding which player has won
									if (time_player1 || time_player2) {	//First detect if at least one of them has pressed the button
										if (time_player1) {
											// To resolve the sign issue
											if (time_player1 > present_time_g2) {//pressed after the counter reaches zero
												time_player1 -= present_time_g2;//If the player 1 has pressed, determine after how long has he/she pressed
												sign='+';
											} else {  //pressed before the counter reaches zero
												time_player1 = present_time_g2 - time_player1;
												sign='-';
											}
										}
										if (time_player2) {
											// To resolve the sign issue
											if (time_player2 > present_time_g2) { //pressed after the counter reaches zero
												time_player2 -= present_time_g2;//If the player 2 has pressed, determine after how long has he/she pressed
												sign='+';
											} else {   //pressed before the counter reaches zero
												time_player2 = present_time_g2- time_player2;
												sign='-';
											}
										}
										if (((time_player1 > time_player2)&&(time_player2))||((time_player2 > time_player1)&&(!(time_player1)))) {//If the time after which player 1 has pressed the button is longer
																																					  //the condition time_player2 is added as a guarantee that the player 2
																																					  //has pressed his/her button (avoid the situation where time_player2=0)
											ON_LED2();
											Bin2Ascii(time_player2, &text[0]);
											write_message((unsigned char*)"2 ",sizeof("2 ")); //print that the player 2 has won
											write_message((unsigned char*)(&sign),sizeof(sign));
											write_message(text,sizeof(text));
											write_message((unsigned char*)"\n\r",sizeof("\n\r") );


										}
										if (((time_player2 > time_player1)&&(time_player1))||((time_player1 > time_player2)&&(!(time_player2)))){//If the time after which player 2 has pressed the button is longer
																																					  //the condition time_player1 is added as a guarantee that the player 1
																																					  //has pressed his/her button (avoid the situation where time_player1=0)
											ON_LED1();
											Bin2Ascii(time_player1, &text[0]);
											write_message((unsigned char*)"1 ",sizeof("1 ")); //print that the player 1 has won
											write_message((unsigned char*)(&sign),sizeof(sign) );
											write_message(text,sizeof(text));
											write_message((unsigned char*)"\n\r",sizeof("\n\r") );


										}


										//generating melody depending on whether the button was pressed before or after the end of the countdown
										//PA5 timer2 channel 1
										TIM3->CNT=0;
										TIM3->CCR1 = 0;
										TIM2->CCER|=(1<<(0));//enabling hardware output
										delay(500);
										if (sign =='-' ){ //melody 1 is the melody when we press the button before the end of the countdown

												music=melody1;
												count=0;
												TIM2->CCR1=music[count];
												TIM2->CR1=1;
												TIM3->CCR2=pulses_half_second;
												TIM3->CR1=1;


										}
										else if (sign =='+'){

												music=melody2;
												count=0;
												TIM2->CCR1=music[count];
												TIM2->CR1=1;
												TIM3->CCR2=pulses_half_second;
												TIM3->CR1=1;

										}
										while(End_Of_Melody!=1);
										delay(500);
										TIM2->CCER&=~(1<<(0));//disabling hardware output
										TIM2->CR1=0;
										TIM3->CR1=0;

									}
									else {
										write_message((unsigned char*)"END of the game no winner no loser\r\n",sizeof("END of the game no winner no loser\r\n") );

									}



										break;

						}


	}
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim9, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_UART_Receive_IT(huart,recieved,1);
	if (recieved[0]=='X'){
		NVIC_SystemReset();	//software reset
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
			/* User can add his own implementation to report the HAL error return state */
			__disable_irq();
			while (1) {
			}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

