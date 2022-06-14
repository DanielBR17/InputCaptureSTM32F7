#include "stm32f746xx.h"
#include "stm32f7xx.h"
#include "UART_STM32F7.h"


typedef struct{
	
	volatile uint32_t  t[2];
	volatile char edge_tim;
	volatile int  width;
	volatile double freq;
	
}TIM_IC;

typedef struct{
	
	TIM_IC timer_ic;
	GPIO_TypeDef* SFR05_port;
	int SFR05_pin;
	
	float dist_m;
	float dist_cm;
	float dist_ft;
	float dist_in;
	
}SRF05;

TIM_IC Ultrasonido;
SRF05 sensor;
double refClock;
char buff[20], ic_ready=0;
float dist_cm_lin;
volatile int32_t TimeDelay;
char flag=0;
int time=0;
void TIM2_IRQHandler(void);
void TIM2_InputCapture(void);
void PLL(void);
void TIM3_delay(void);
__INLINE void delay_us(uint32_t nTime);

int main(void)
{
	
	PLL();
	SysTick_Config(SystemCoreClock/1000);
	
	Ultrasonido.edge_tim = 0;
	TIM2_InputCapture();
	TIM3_delay();
	
	UART_Init(UART4);
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= (1UL<<2*6);
	GPIOA->OSPEEDR |= (1UL<<2*6);
	GPIOA->PUPDR |= (2UL<<2*6);
	GPIOA->OTYPER &=~ (1UL<<6);
	GPIOA->ODR &=~ 1UL<<6;
	
	while(1)
	{
		
		if(flag == 1 && ic_ready == 1)
		{
			flag=0;
			ic_ready=0;
			GPIOA->ODR |= 1UL<<6;
			delay_us(10);
			GPIOA->ODR &=~ 1UL<<6;	
			
			sprintf(buff,"%.2f\r\n",sensor.dist_cm);
			UART_Transmit(UART4,buff);

		}
		
	}
}

void TIM2_IRQHandler(void)
{
//	CLEAR_BIT(TIM2->SR, TIM_SR_UIF);
	
	if(READ_BIT(TIM2->SR, TIM_SR_CC1IF))
	{
		CLEAR_BIT(TIM2->SR, TIM_SR_CC1IF);
		
		if(Ultrasonido.edge_tim == 0)
		{
			Ultrasonido.edge_tim = 1;
			Ultrasonido.t[0] = 	TIM2->CCR1;
			
			/*	01: inverted/falling edge		*/
			TIM2->CCER |= 1UL<<TIM_CCER_CC1P_Pos;
		}
		else if(Ultrasonido.edge_tim == 1)
		{
			Ultrasonido.edge_tim = 0;
			Ultrasonido.t[1] = 	TIM2->CCR1;
			TIM2->CNT = 0;
			
			if(Ultrasonido.t[1] > Ultrasonido.t[0])
			{
				Ultrasonido.width = Ultrasonido.t[1] - Ultrasonido.t[0];
				
			}
			else if (Ultrasonido.t[0] > Ultrasonido.t[1])
			{
				Ultrasonido.width = (0xFFFF-Ultrasonido.t[0])+Ultrasonido.t[1];
			}
			
			sensor.dist_cm = (float)Ultrasonido.width* 0.034f/2.0f;		
			/*	00: noninverted/rising edge	*/
			TIM2->CCER &=~ TIM_CCER_CC1P_Msk;
			
			ic_ready=1;

		}
	}
}

void SysTick_Handler(void)
{
	time++;
	if(time>100){
		time=0;
		flag=1;
	}

}

void TIM3_IRQHandler(void)
{
	CLEAR_BIT(TIM3->SR, TIM_SR_UIF);

	if(TimeDelay > 0)
	{
		TimeDelay--;
	}
		
}

__INLINE void delay_us(uint32_t nTime)
{
	TimeDelay = nTime;
	while(TimeDelay != 0);
}

void TIM3_delay(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	TIM3->CR1 &=~ TIM_CR1_CEN;
	
	TIM3->PSC = 1-1;
	TIM3->ARR = 168-1;
	TIM3->DIER |= TIM_DIER_UIE;
	TIM3->EGR |= TIM_EGR_UG;
	
	NVIC_SetPriority(TIM3_IRQn,0);
	NVIC_EnableIRQ(TIM3_IRQn);
	
	TIM3->CR1 |= TIM_CR1_CEN;
}

void TIM2_InputCapture(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	TIM2->CR1 &=~ TIM_CR1_CEN;
//	TIM2->PSC = 16-1;
	TIM2->PSC = 84-1;
	TIM2->ARR = 0xFFFF;
	
		
	/*	Bits 1:0 CC1S: Capture/Compare 1 selection 
			01: CC1 channel is configured as input, IC1 is mapped on TI1	*/
	TIM2->CCMR1 |= 1UL << TIM_CCMR1_CC1S_Pos;
	
	/*	Bits 7:4 IC1F: Input capture 1 filter	
			0011: fSAMPLING=fCK_INT, N=8	*/
	TIM2->CCMR1 |= 0x3 << TIM_CCMR1_IC1F_Pos;
	
	/*	Bit 1 CC1P: Capture/Compare 1 output Polarity.
			CC1 channel configured as input: CC1NP/CC1P bits select TI1FP1 and TI2FP1 
			polarity for trigger or capture operations.
			00: noninverted/rising edge	*/
	TIM2->CCER &=~ TIM_CCER_CC1P_Msk;
	
	/*	Bits 3:2 IC1PSC: Input capture 1 prescaler	
			00: no prescaler, capture is done each time an edge is detected on the 
			capture input	*/
	TIM2->CCMR1 &=~ TIM_CCMR1_IC1PSC_Msk;
	
	/*	Bit 0 CC1E: Capture/Compare 1 output enable.
			CC1 channel configured as input: This bit determines if a capture of the counter value can
			actually be done into the input capture/compare register 1 (TIMx_CCR1) or not.
			1: Capture enabled	*/
	TIM2->CCER |= TIM_CCER_CC1E;
	
	/*	Bit 1 CC1IE: Capture/Compare 1 interrupt enable
			1: CC1 interrupt enabled.	*/
	TIM2->DIER |= TIM_DIER_CC1IE;
	
//	/*	Bit 9 CC1DE: Capture/Compare 1 DMA request enable
//			1: CC1 DMA request enabled.	*/
//	TIM2->DIER |= TIM_DIER_CC1DE;

	/*	Bit 1 CC1G: Capture/compare 1 generation
			1: A capture/compare event is generated on channel 1:
			If channel CC1 is configured as input:
			The current value of the counter is captured in TIMx_CCR1 register. The CC1IF flag is set,
			the corresponding interrupt or DMA request is sent if enabled. The CC1OF flag is set if the
			CC1IF flag was already high.	*/
	TIM2->EGR |= TIM_EGR_CC1G;
	
	
	TIM2->CR1 |= TIM_CR1_CEN;
	
	NVIC_EnableIRQ(TIM2_IRQn);	
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	GPIOA->MODER 	|= (2UL<<2*5);
	GPIOA->OSPEEDR |= (3UL<<2*5);
	GPIOA->PUPDR |= (2UL<<2*5);
	GPIOA->OTYPER &=~ (1UL<<5);
	GPIOA->AFR[0] |= (0x1<<20U);
	
}

void PLL(void)
{
	
	RCC->CR |= 0x10000; // ACTIVAR EL OSCILADOR HSE
	while((RCC->CR & 0x20000) == 0); // ESPERAR QUE EL HSE ESTE ESTABILIZADO
	RCC->APB1ENR = 0x10000000; //POWER INTERFACE CLOCK ENABLE
//	RCC->CFGR = 0x9000; // APB2(/2) Y APB1 (/4)
	RCC->CFGR |= (0x4<<RCC_CFGR_PPRE2_Pos) | (0x5<<RCC_CFGR_PPRE1_Pos);
	RCC->PLLCFGR = 0x7405408; // HSE RELOJ DE ENTRADA AL PLL; PLLN = 336; PLLP = 2.
	RCC->CR |= 0x01000000; // ACTIVAR EL PLL
	while((RCC->CR & 0x02000000) == 0); // ESPERAR QUE EL PLL ESTE LISTO
	FLASH->ACR = 0x205; // ACTIVAR EL ART ACCELERATOR Y ACTUALIZA 5WAIT STATES
	RCC->CFGR |= 2; // SELECCIONAR EL PLL COMO LA FUENTE DE RELOJ DEL MICRO
	
	SystemCoreClockUpdate();
	
}//End PLL
