#include "MCU_STM32.h"

static uint8_t uart_tx_buf[256];
static ring_buf_t uart_tx_ring;

static void rb_init(ring_buf_t *buffer, uint8_t *array, uint16_t capacity)
{
	Assert(capacity > 0);
	Assert(buffer != 0);	
	Assert(array != 0);
	Assert((capacity & (capacity - 1)) == 0);
	
	buffer->base = array;
	buffer->capacity = capacity;
	buffer->mask = capacity - 1;
	buffer->head = buffer->tail = 0;
}

static int8_t rb_get(ring_buf_t *rb, uint8_t *byte)
{
	int8_t result;
	uint16_t next;
	
	Assert(rb != 0);
	Assert(rb->base != 0);
	next = (rb->tail + 1) & rb->mask;
	
	if (rb->head != rb->tail)
	{
		*byte = rb->base[rb->tail];
		__DMB();
		rb->tail = next;
		result = 0;
	}
	else
	{
		result = -1;
	}
	
	return result;
}

static int8_t rb_put(ring_buf_t *rb, uint8_t byte)
{
	int8_t result;
	uint16_t next;
	
	Assert(rb != 0);
	Assert(rb->base != 0);
	next = (rb->head + 1) & rb->mask;
	
	if (next != rb->tail)
	{
		rb->base[rb->head] = byte;
		__DMB();
		rb->head = next;
		result = 0;
	}
	else
	{
		result = -1;
	}
	
	return result;
}

static int8_t rb_puts(ring_buf_t *rb, const char *msg)
{
	int8_t result;
    
	Assert(rb != 0);	
	Assert(rb->base != 0);
	Assert(msg != 0);
	
	result = 0;
	
	while(*msg != 0 && result == 0)
	{
		result = rb_put(rb, *msg);
		msg++;
	}
	
	return result;
}

static int8_t rb_put_unum(ring_buf_t *rb, uint32_t val)
{
	int8_t result = -1, i = 0;
	char buf[11];
	
	if (val == 0) {
        result = rb_put(rb, '0');
        return result;
    }
    
    while (val > 0) 
	{
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }
    
    while (i > 0) 
	{
        result = rb_put(rb, buf[--i]);
    }
	
	return result;
}

static int8_t rb_put_num(ring_buf_t *rb, int32_t val)
{
	int8_t result = -1;
	
	if (val < 0) { result = rb_put(rb, '-'); result = rb_put_unum(rb, -val); }
    else result = rb_put_unum(rb, val);
	
	return result;
}

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

#ifdef NODE_B
static void uart_put_hex8(uint8_t val)
{
    char hi = (char)(val >> 4);
    char lo = (char)(val & 0x0F);
    uart_putc(hi < 10 ? '0' + hi : 'A' - 10 + hi);
    uart_putc(lo < 10 ? '0' + lo : 'A' - 10 + lo);
}

static void uart_put_hex32(uint32_t val)
{	
    uart_put_hex8((uint8_t)(val >> 24));
    uart_put_hex8((uint8_t)(val >> 16));
    uart_put_hex8((uint8_t)(val >> 8));
    uart_put_hex8((uint8_t)(val));
}
#endif

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

#ifdef NODE_A
static void can_msg_set_data_u16(can_msg_t *msg, uint16_t val) {
    Assert(msg != 0);
    msg->data[0] = (uint8_t)(val & 0xFF);
    msg->data[1] = (uint8_t)((val >> 8) & 0xFF);
}

static void can1_tx_id_dlc_data(can_id_t std_id, can_dlc_t dlc, uint8_t *data)
{	
	Assert(std_id.raw <= 0x7FF);
	Assert(dlc.raw <= 8);
	
	while (!(CAN1->TSR & CAN1_TSR_TME0)) { }
    
	CAN1->TI0R = (std_id.raw << 21);
	CAN1->TDT0R = dlc.raw;
	{
		uint32_t low = 0, high = 0;
		uint8_t i;
		for (i = 0; i < dlc.raw && i < 4; i++) {
			low |= (uint32_t)data[i] << (8 * i);
		}
		CAN1->TDL0R = low;
        
		if (dlc.raw > 4) {
			for (i = 4; i < dlc.raw && i < 8; i++) {
				high |= (uint32_t)data[i] << (8 * (i - 4));
			}
			CAN1->TDH0R = high;
		}
	}
	CAN1->TI0R |= CAN1_TIXR_TXRQ;
	
	while (!(CAN1->TSR & CAN1_TSR_RQCP0)) { } 
	CAN1->TSR |= CAN1_TSR_RQCP0;
}

static void can1_tx_msg(can_msg_t *msg)
{
	Assert(msg != 0);
	can1_tx_id_dlc_data(msg->id, msg->dlc, msg->data);
}
#endif

static void can1_init(void)
{	
	CAN1->MCR = (CAN1->MCR & ~CAN1_MCR_SLEEP) | CAN1_MCR_INRQ;
	while (CAN1->MSR & CAN1_MSR_SLAK) { }
	while (!(CAN1->MSR & CAN1_MSR_INAK)) { }
	
	CAN1->BTR = CAN_BTR_SJW(1) | CAN_BTR_TS2(2) | CAN_BTR_TS1(15) | CAN_BTR_BRP(4);
	/*CAN1->BTR |= (1U << 30); Loopback mode for testing */
	
	CAN1->FMR  |=  1;
	CAN1->FS1R |=  1;
	CAN1->F0R1  =  0;
	CAN1->F0R2  =  0;
	CAN1->FA1R |=  1;
	CAN1->FMR  &= ~1;
	
	CAN1->MCR |= CAN1_MCR_NART;
	CAN1->MCR &= ~CAN1_MCR_INRQ;
	while (CAN1->MSR & CAN1_MSR_INAK) { }
	
	CAN1->IER |= CAN1_IER_FMPIE0 | CAN1_IER_ERRIE
        | CAN1_IER_BOFIE | CAN1_IER_EPVIE | CAN1_IER_EWGIE;
	NVIC_ISER0 = (1U << IRQ_CAN_RX0) | (1U << IRQ_CAN_SCE);
}

void CAN_SCE_IRQHandler(void)
{
    uint32_t esr = CAN1->ESR;
    uint8_t tec = (uint8_t)(esr >> 16);
    uint8_t rec = (uint8_t)(esr >> 24);
    uint8_t lec = (uint8_t)((esr >> 4) & 0x7);
    
    rb_puts(&uart_tx_ring, "ERR lec=");
    rb_put_unum(&uart_tx_ring, lec);
    rb_puts(&uart_tx_ring, " tec=");
    rb_put_unum(&uart_tx_ring, tec);
    rb_puts(&uart_tx_ring, " rec=");
    rb_put_unum(&uart_tx_ring, rec);
    if (esr & CAN_ESR_BOFF) rb_puts(&uart_tx_ring, " BOFF");
    if (esr & CAN_ESR_EPVF) rb_puts(&uart_tx_ring, " EPVF");
    rb_puts(&uart_tx_ring, "\r\n");
    
    CAN1->ESR &= ~CAN_ESR_LEC;
    CAN1->MSR |= CAN1_MSR_ERRI;
}

void EXTI0_IRQHandler(void)
{
    Assert(EXTI->PR & (1U << 0));
	if (EXTI->PR & (1U << 0)) {
		EXTI->PR = (1U << 0);
		uart_write("button!\r\n");
#ifdef NODE_A
		uart_write("node A!\r\n");
#endif
#ifdef NODE_B
		uart_write("node B!\r\n");
#endif
#ifdef NODE_A
        {
            uint8_t i;
            can_msg_t msg;
            uart_write("Node A: sending 'CAFE'\r\n");
            msg.id.raw = 0x123;
            msg.dlc.raw = 2;
            for (i = 0; i < 8; i++) msg.data[i] = 0;
            can_msg_set_data_u16(&msg, 0xCAFE);
            can1_tx_msg(&msg);	
        }
#endif
	}	
}

void USB_LP_CAN_RX0_IRQHandler(void)
{
	can_msg_t rx;
	Assert(CAN1->RF0R & CAN1_RF0R_FMP0);
	
	rx.id.raw  = (uint32_t)((CAN1->RI0R >> 21) & 0x7FF);
	rx.dlc.raw = (uint8_t)(CAN1->RDT0R & 0x0F);
    
	Assert(rx.id.raw <= 0x7FF);
	Assert(rx.dlc.raw <= 8);
	{
		uint32_t low  = CAN1->RDL0R;
		uint32_t high = CAN1->RDH0R;
		rx.data[0] = (uint8_t)(low & 0xFF);
		rx.data[1] = (uint8_t)((low >> 8) & 0xFF);
		rx.data[2] = (uint8_t)((low >> 16) & 0xFF);
		rx.data[3] = (uint8_t)((low >> 24) & 0xFF);
		rx.data[4] = (uint8_t)(high & 0xFF);
		rx.data[5] = (uint8_t)((high >> 8) & 0xFF);
		rx.data[6] = (uint8_t)((high >> 16) & 0xFF);
		rx.data[7] = (uint8_t)((high >> 24) & 0xFF);
		
		rb_puts(&uart_tx_ring, "RX ID=");
		rb_put_unum(&uart_tx_ring, rx.id.raw);
		rb_puts(&uart_tx_ring, " DLC=");
		rb_put(&uart_tx_ring, '0' + rx.dlc.raw);
		rb_puts(&uart_tx_ring, " DATA=");
		{
			int i;
			for (i = 0; i < rx.dlc.raw; i++) {
				rb_put_num(&uart_tx_ring, rx.data[i]);
			}
		}
		rb_puts(&uart_tx_ring, "\r\n");
	}
    
	CAN1->RF0R |= CAN1_RF0R_RFOM0;   /* release FIFO0 */
}

int main(void)
{
	clock_init();
	gpio_init();
	uart_init();
    rb_init(&uart_tx_ring, uart_tx_buf, sizeof(uart_tx_buf));
	tim2_init();
	exti0_init();
	can1_init();
	
    
    rb_put_num(&uart_tx_ring, -32);
    rb_puts(&uart_tx_ring, "hello\r\n");
    
	uart_write("Project 2 - CAN Bus communication\r\n");
    
	for (;;) {
		{
			uint8_t c;
			while (rb_get(&uart_tx_ring, &c) == 0)
			{
				while (!(USART1->SR & USART_SR_TXE));
				USART1->DR = c;
			}
		}
#ifdef NODE_B
		if (CAN1->RF0R & CAN1_RF0R_FMP0) {
			uart_write("Node B!\r\n");
            
			uart_write("RX ID  = ");
			uart_put_hex32(CAN1->RI0R);
			uart_write("\r\n");
			
			uart_write("RX DATA = ");
			uart_put_hex32(CAN1->RDL0R);
			uart_write("\r\n");
			
			CAN1->RF0R |= CAN1_RF0R_RFOM0;
		}
#endif
	} /* Using interrupts for flow control */
}
