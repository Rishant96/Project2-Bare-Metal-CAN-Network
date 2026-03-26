#include "MCU_STM32.h"

static void clock_init(void)
{
	FLASH_R->ACR = FLASH_ACR_LATENCY_2WS | FLASH_ACR_PRFTBE;
	
	RCC->CR |= RCC_CR_HSEON;
	while (!(RCC->CR & RCC_CR_HSERDY));
	
	RCC->CR |= RCC_CR_CSSON;
	
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL9;

	/* Turn on PLL and wait */
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY));
	
	/* Switch system clock to PLL */
	RCC->CFGR = (RCC->CFGR & ~(3U << 0)) | RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL);
}

static void gpio_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN
				  |  RCC_APB2ENR_IOPCEN
				  |  RCC_APB2ENR_AFIOEN
				  |  RCC_APB2ENR_USART1EN;
				  
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
	
	/* Setup Pin 13 */
	GPIOC->CRH &= ~(0xFU << 20);
	GPIOC->CRH |=  (0x2U << 20);
	
	/* Setup Pin 0 */
	GPIOA->CRL &= ~(0xFU << 0);
	GPIOA->CRL |=  (0x8U << 0);
	GPIOA->BSRR = (1U << 0);
	
	/* Setup Pin 9 */
	GPIOA->CRH &= ~(0xFU << 4);
	GPIOA->CRH |=  (0xBU << 4);
	
	/* Setup Pin 11 */
	GPIOA->CRH &= ~(0xFU << 12);
	GPIOA->CRH |=  (0x8U << 12);
	GPIOA->BSRR = (1U << 11);
	
	/* Setup Pin 12 */
	GPIOA->CRH &= ~(0xFU << 16);
	GPIOA->CRH |=  (0xBU << 16);
}


static void uart_putc(char c)
{
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (uint32_t)c;
}

static void uart_put_byte(uint32_t b4)
{
	int i;
	for (i = 7; i >= 0; i--)
	{
		char c = (char)(((0x0000000f << (4 * i)) & b4) >> (4 * i));
		if (c < 10) c += '0';
		else c += 'A' - 10;
		uart_putc(c);
	}
}

static void uart_write(const char *s)
{
    while (*s) {
        uart_putc(*s++);
    }
}

static void uart_init(void)
{
	USART1->BRR = 0x271;
	USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static void tim2_init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	TIM2->PSC = 7199;
	TIM2->ARR = 9999;
	TIM2->DIER = TIM_DIER_UIE;
	TIM2->CR1  = TIM_CR1_CEN;
	
	NVIC_ISER0 = (1U << IRQ_TIM2);
}

static void exti0_init(void)
{
	EXTI->FTSR |= (1U << 0);
	EXTI->IMR  |= (1U << 0);
	
	NVIC_ISER0 = (1U << IRQ_EXTI0);
}

void NMI_Handler(void)
{
	if (RCC->CIR & RCC_CIR_CSSF) {
		RCC->CIR |= RCC_CIR_CSSC;
		uart_write("HSE Failure\r\n");
	}
}

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;   /* Clear interrupt flag */
        GPIOC->ODR ^= (1U << 13);  /* Toggle LED */
		uart_write("tick");
#ifdef NODE_A
        uart_write(" A");
#endif
#ifdef NODE_B
        uart_write(" B");
#endif
		uart_write("\r\n");
    }
}

static void can1_tx(uint32_t std_id, uint32_t len, uint32_t data);

void EXTI0_IRQHandler(void)
{
	if (EXTI->PR & (1U << 0)) {
		EXTI->PR = (1U << 0);
		uart_write("button!\r\n");
#ifdef NODE_A
		uart_write("node A!\r\n");
#endif
#ifdef NODE_B
		uart_write("node B!\r\n");
#endif

		uart_write("CAN1->TSR = ");
		uart_put_byte(CAN1->TSR);
		uart_write("\r\n");
		
		uart_write("CAN1->MSR = ");
		uart_put_byte(CAN1->MSR);
		uart_write("\r\n");
		
		uart_write("CAN1->MCR = ");
		uart_put_byte(CAN1->MCR);
		uart_write("\r\n");
		
		uart_write("CAN1->BTR = ");
		uart_put_byte(CAN1->BTR);
		uart_write("\r\n");
		
#ifdef NODE_A
		uart_write("Node A: sending 'CAFE'\r\n");
		can1_tx(0x123, 2, 0xCAFE);		
#endif
		
		uart_write("CAN1->TSR = ");
		uart_put_byte(CAN1->TSR);
		uart_write("\r\n");
		
		uart_write("CAN1->MSR = ");
		uart_put_byte(CAN1->MSR);
		uart_write("\r\n");
		
		uart_write("CAN1->MCR = ");
		uart_put_byte(CAN1->MCR);
		uart_write("\r\n");
		
		uart_write("CAN1->BTR = ");
		uart_put_byte(CAN1->BTR);
		uart_write("\r\n");
	}	
}

static void can1_init(void)
{	
	CAN1->MCR = (CAN1->MCR & ~CAN1_MCR_SLEEP) | CAN1_MCR_INRQ;
	while (CAN1->MSR & CAN1_MSR_SLAK) { }
	while (!(CAN1->MSR & CAN1_MSR_INAK)) { }
	
	CAN1->BTR = (0U << 24) | (1U << 20) | (14U << 16) | (3U << 0); /* SJW = 1, TS2 = 2, TS1 = 15, BRP = 4 (all minus 1) */
	/*CAN1->BTR |= (1U << 30); Loopback mode for testing */
	
	CAN1->FMR  |=  1;
	CAN1->FS1R |=  1;
	CAN1->F0R1  =  0;
	CAN1->F0R2  =  0;
	CAN1->FA1R |=  1;
	CAN1->FMR  &= ~1;
	
	CAN1->MCR |= (1U << 4);
	CAN1->MCR &= ~CAN1_MCR_INRQ;
	while (CAN1->MSR & CAN1_MSR_INAK) { }
}

static void can1_tx(uint32_t std_id, uint32_t len, uint32_t data)
{
	while (!(CAN1->TSR & CAN1_TSR_TME0)) { }

	CAN1->TI0R = (std_id << 21);
	CAN1->TDT0R = len;
	CAN1->TDL0R = data;
	
	CAN1->TI0R |= CAN1_TIXR_TXRQ;
	
	while (!(CAN1->TSR & CAN1_TSR_RQCP0)) { } 
	CAN1->TSR |= CAN1_TSR_RQCP0;
}

int main(void)
{
	clock_init();
	gpio_init();
	uart_init();
	tim2_init();
	exti0_init();
	can1_init();
	
	uart_write("Project 2 - CAN Bus communication\r\n");

	for (;;) {
#ifdef NODE_B
		if (CAN1->RF0R & CAN1_RF0R_FMP0) {
			uart_write("Node B!\r\n");
		
			uart_write("RX ID  = ");
			uart_put_byte(CAN1->RI0R);
			uart_write("\r\n");
			
			uart_write("RX DATA = ");
			uart_put_byte(CAN1->RDL0R);
			uart_write("\r\n");
			
			CAN1->RF0R |= CAN1_RF0R_RFOM0;
		}
#endif
	} /* Using interrupts for flow control */
}