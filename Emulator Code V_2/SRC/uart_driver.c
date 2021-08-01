#include "gpio_driver.h"
#include "rcc_clock.h"
#include "delay.h"
#include "uart_driver.h"







/*!< USART CR1 register clear Mask ((~(uint16_t)0xE9F3)) */
#define CR1_CLEAR_MASK            ((uint16_t)(USART_CR1_M | USART_CR1_PCE | \
                                              USART_CR1_PS | USART_CR1_TE | \
                                              USART_CR1_RE))


/*!< USART CR3 register clear Mask ((~(uint16_t)0xFCFF)) */
#define CR3_CLEAR_MASK            ((uint16_t)(USART_CR3_RTSE | USART_CR3_CTSE))

/*!< USART Interrupts mask */
#define IT_MASK                   ((uint16_t)0x001F)




/******************************************************************************/
/*                                                                            */
/*                      Helper functions                                      */
/*                                                                            */
/******************************************************************************/

/**
	*Enable the given USART peripheral 
	*Paramter: *uartx : base address of the USART or UART peripheral
	*return: None
	*/
void USART_Enable(USART_TypeDef *uartx)
{
	
	uartx->CR1 |= USART_CR1_UE;  //Uart Enable
}



/**
	*Disable the given USART peripheral 
  *Parameter: *uartx : base address of the USART or UART peripheral
  *Return: None
	*/
void USART_Disable(USART_TypeDef *uartx)
{
	uartx->CR1 &= ~USART_CR1_UE;
}



/**
	*Enable/Disable the Transmit block of the  given USART peripheral 
	*Parameter *uartx : base address of the USART or UART peripheral
  *Return: None
	*/
void USART_EnableTx(USART_TypeDef *uartx)
{
	
		uartx->CR1 |= USART_CR1_TE;	//Tx Enable
}



void USART_DisableTx(USART_TypeDef *uartx)
{

	uartx->CR1 &= ~USART_CR1_TE;

}	



/**
	*Enable/Disable the Receive block of the  given USART peripheral 
  *Parameter :*uartx : base address of the USART or UART peripheral
  *Return: None
	*/
void USART_EnableRx(USART_TypeDef *uartx)
{
	
  uartx->CR1 |= USART_CR1_RE;  //Rx Enable
		
}

void USART_DisableRx(USART_TypeDef *uartx)
{
	
 uartx->CR1 &= ~USART_CR1_RE;

}





/**********************************************************/
/*********Drivers******************************************/

void USART1_Init(USART_Type* Init)
{
	//Enable peripheral Clock
	RCC_USART1_CLK_ENABLE();
	
	//GPIO clock enable
	RCC_GPIOA_CLOCK_ENABLE_CLK();
	
	
	GPIO_Config_Mode (GPIOA,GPIO_PIN_9,GPIO_MODE_AF);        //TX pin 
  GPIO_Config_OType (GPIOA, GPIO_PIN_9, GPIO_OTYPE_PP); 
  GPIO_Config_Speed (GPIOA, GPIO_PIN_9, GPIO_HIGH_SPEED);    
  GPIO_Config_PuPd	(GPIOA,GPIO_PIN_9,GPIO_PUPD_NOPULL);
  GPIO_Config_AF(GPIOA, GPIO_PIN_9, 7); 

	GPIO_Config_Mode (GPIOA,GPIO_PIN_10,GPIO_MODE_AF);        //RX pin 
  GPIO_Config_Speed (GPIOA, GPIO_PIN_10, GPIO_HIGH_SPEED);    
  GPIO_Config_PuPd	(GPIOA,GPIO_PIN_10,GPIO_PUPD_NOPULL);
  GPIO_Config_AF(GPIOA, GPIO_PIN_10, 7); 
	
	
/*	GPIO_Config_Mode (GPIOA,GPIO_PIN_11,GPIO_MODE_AF);        //CTS pin  
  GPIO_Config_Speed (GPIOA, GPIO_PIN_11, GPIO_HIGH_SPEED);    
  GPIO_Config_PuPd	(GPIOA,GPIO_PIN_11,GPIO_PUPD_NOPULL);
  GPIO_Config_AF(GPIOA, GPIO_PIN_11, 7); 

	GPIO_Config_Mode (GPIOA,GPIO_PIN_12,GPIO_MODE_AF);        //RTS pin 
  GPIO_Config_Speed (GPIOA, GPIO_PIN_12, GPIO_HIGH_SPEED);    
  GPIO_Config_PuPd	(GPIOA,GPIO_PIN_12,GPIO_PUPD_NOPULL);
  GPIO_Config_AF(GPIOA, GPIO_PIN_12, 7); 
	*/
	
	uint32_t tmpreg = 0x00;
	uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
	
	/*---------------------------- USART CR2 Configuration -----------------------*/
	tmpreg = USART1->CR2;

  /* Clear STOP[13:12] bits */
  tmpreg &= (uint32_t)~((uint32_t)USART_CR2_STOP);

  /* Configure the USART Stop Bits:
      Set STOP[13:12] bits according to USART_StopBits value */
  tmpreg |= (uint32_t)Init->USART_StopBits;
  
  /* Write to USART CR2 */
  USART1->CR2 = (uint16_t)tmpreg;
	
	
	
	/*---------------------------- USART CR1 Configuration -----------------------*/
  tmpreg = USART1->CR1;

  /* Clear M, PCE, PS, TE and RE bits */
  tmpreg &= (uint32_t)~((uint32_t)CR1_CLEAR_MASK);

  /* Configure the USART Word Length, Parity and mode: 
     Set the M bits according to USART_WordLength value 
     Set PCE and PS bits according to USART_Parity value
     Set TE and RE bits according to USART_Mode value */
  tmpreg |= (uint32_t)Init->USART_WordLength | Init->USART_Parity |
            Init->USART_Mode;

  /* Write to USART CR1 */
  USART1->CR1 = (uint16_t)tmpreg;
	
	
	
	
	/*---------------------------- USART CR3 Configuration ----------------------- 
  tmpreg = USART1->CR3;

  Clear CTSE and RTSE bits 
  tmpreg &= (uint32_t)~((uint32_t)CR3_CLEAR_MASK);

 Configure the USART HFC : 
      Set CTSE and RTSE bits according to USART_HardwareFlowControl value 
  tmpreg |= Init->USART_HardwareFlowControl;

  Write to USART CR3
  USART1->CR3 = (uint16_t)tmpreg;*/

	
	
	 /* Determine the integer part */
  if ((USART1->CR1 & USART_CR1_OVER8) != 0)
  {
    /* Integer part computing in case Oversampling mode is 8 Samples */
    integerdivider = 100*((48000000) / (8 * (Init->USART_BaudRate)));     
  }
  else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
  {
    /* Integer part computing in case Oversampling mode is 16 Samples */
    integerdivider = 100*((48000000) / (16 * (Init->USART_BaudRate)));        //multiplying by 100 to get the fractional part
  }
  tmpreg = (integerdivider / 100) << 4;

  /* Determine the fractional part */
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

  /* Implement the fractional part in the register */
  if ((USART1->CR1 & USART_CR1_OVER8) != 0)
  {
    tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
  }
  else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
  {
    tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
  }
  
  /* Write to USART BRR register */
  USART1->BRR = (uint16_t)tmpreg;
	
	
}



void USART2_Init(USART_Type* Init)
{
	
	//Enable peripheral Clock
	RCC_USART2_CLK_ENABLE();
	
	//GPIO clock enable
	RCC_GPIOA_CLOCK_ENABLE_CLK();
	
	
	GPIO_Config_Mode (GPIOA,GPIO_PIN_2,GPIO_MODE_AF);        //TX pin 
  GPIO_Config_OType (GPIOA, GPIO_PIN_2, GPIO_OTYPE_PP); 
  GPIO_Config_Speed (GPIOA, GPIO_PIN_2, GPIO_HIGH_SPEED);    
  GPIO_Config_PuPd	(GPIOA,GPIO_PIN_2,GPIO_PUPD_NOPULL);
  GPIO_Config_AF(GPIOA, GPIO_PIN_2, 7); 

	GPIO_Config_Mode (GPIOA,GPIO_PIN_3,GPIO_MODE_AF);        //RX pin 
  GPIO_Config_Speed (GPIOA, GPIO_PIN_3, GPIO_HIGH_SPEED);    
  GPIO_Config_PuPd	(GPIOA,GPIO_PIN_3,GPIO_PUPD_NOPULL);
  GPIO_Config_AF(GPIOA, GPIO_PIN_3, 7); 

	
	
	
	uint32_t tmpreg = 0x00;
	uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
	
	/*---------------------------- USART CR2 Configuration -----------------------*/
	tmpreg = USART2->CR2;
	
	

  /* Clear STOP[13:12] bits */
  tmpreg &= (uint32_t)~((uint32_t)USART_CR2_STOP);

  /* Configure the USART Stop Bits:
      Set STOP[13:12] bits according to USART_StopBits value */
  tmpreg |= (uint32_t)Init->USART_StopBits;
  
  /* Write to USART CR2 */
  USART2->CR2 = (uint16_t)tmpreg;
	
	
	
	/*---------------------------- USART CR1 Configuration -----------------------*/
  tmpreg = USART2->CR1;

  /* Clear M, PCE, PS, TE and RE bits */
  tmpreg &= (uint32_t)~((uint32_t)CR1_CLEAR_MASK);

  /* Configure the USART Word Length, Parity and mode: 
     Set the M bits according to USART_WordLength value 
     Set PCE and PS bits according to USART_Parity value
     Set TE and RE bits according to USART_Mode value */
  tmpreg |= (uint32_t)Init->USART_WordLength | Init->USART_Parity |
            Init->USART_Mode;

  /* Write to USART CR1 */
  USART2->CR1 = (uint16_t)tmpreg;
	

	
	
	 /* Determine the integer part */
  if ((USART2->CR1 & USART_CR1_OVER8) != 0)
  {
    /* Integer part computing in case Oversampling mode is 8 Samples */
    integerdivider = 100*((24000000) / (8 * (Init->USART_BaudRate)));    
  }
  else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
  {
    /* Integer part computing in case Oversampling mode is 16 Samples */
    integerdivider =100*((24000000) / (16 * (Init->USART_BaudRate)));    
  }
  tmpreg = (integerdivider/100 ) << 4;

  /* Determine the fractional part */
  fractionaldivider = integerdivider - (100*(tmpreg >> 4));

  /* Implement the fractional part in the register */
  if ((USART2->CR1 & USART_CR1_OVER8) != 0)
  {
    tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
  }
  else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
  {
    tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
  }
  
  /* Write to USART BRR register */
  USART2->BRR = (uint16_t)tmpreg;
	
}




void USART3_Init(USART_Type* Init)
{
	
	//Enable peripheral Clock
	RCC_USART3_CLK_ENABLE();
	
	//GPIO clock enable
  RCC_GPIOC_CLOCK_ENABLE_CLK();
	
	GPIO_Config_Mode (GPIOC,GPIO_PIN_10,GPIO_MODE_AF);        //TX pin 
  GPIO_Config_OType (GPIOC, GPIO_PIN_10, GPIO_OTYPE_PP); 
  GPIO_Config_Speed (GPIOC, GPIO_PIN_10, GPIO_HIGH_SPEED);    
  GPIO_Config_PuPd	(GPIOC,GPIO_PIN_10,GPIO_PUPD_NOPULL);
  GPIO_Config_AF(GPIOC, GPIO_PIN_10, 7); 

	GPIO_Config_Mode (GPIOC,GPIO_PIN_5,GPIO_MODE_AF);        //RX pin 
  GPIO_Config_Speed (GPIOC, GPIO_PIN_5, GPIO_HIGH_SPEED);    
  GPIO_Config_PuPd	(GPIOC,GPIO_PIN_5,GPIO_PUPD_NOPULL);
  GPIO_Config_AF(GPIOC, GPIO_PIN_5, 7); 
	
	
	uint32_t tmpreg = 0x00;
	uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
	
	/*---------------------------- USART CR2 Configuration -----------------------*/
	tmpreg = USART3->CR2;

  /* Clear STOP[13:12] bits */
  tmpreg &= (uint32_t)~((uint32_t)USART_CR2_STOP);

  /* Configure the USART Stop Bits:
      Set STOP[13:12] bits according to USART_StopBits value */
  tmpreg |= (uint32_t)Init->USART_StopBits;
  
  /* Write to USART CR2 */
  USART3->CR2 = (uint16_t)tmpreg;
	
	
	
	/*---------------------------- USART CR1 Configuration -----------------------*/
  tmpreg = USART3->CR1;

  /* Clear M, PCE, PS, TE and RE bits */
  tmpreg &= (uint32_t)~((uint32_t)CR1_CLEAR_MASK);

  /* Configure the USART Word Length, Parity and mode: 
     Set the M bits according to USART_WordLength value 
     Set PCE and PS bits according to USART_Parity value
     Set TE and RE bits according to USART_Mode value */
  tmpreg |= (uint32_t)Init->USART_WordLength | Init->USART_Parity |
            Init->USART_Mode;

  /* Write to USART CR1 */
  USART3->CR1 = (uint16_t)tmpreg;
	
	
	
	
	
	
	
	 /* Determine the integer part */
  if ((USART3->CR1 & USART_CR1_OVER8) != 0)
  {
    /* Integer part computing in case Oversampling mode is 8 Samples */
    integerdivider = 100*((24000000) / (8 * (Init->USART_BaudRate)));     
  }
  else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
  {
    /* Integer part computing in case Oversampling mode is 16 Samples */
    integerdivider = 100*((24000000) / (16 * (Init->USART_BaudRate)));        //multiplying by 100 to get the fractional part
  }
  tmpreg = (integerdivider / 100) << 4;

  /* Determine the fractional part */
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

  /* Implement the fractional part in the register */
  if ((USART3->CR1 & USART_CR1_OVER8) != 0)
  {
    tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
  }
  else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
  {
    tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
  }
  
  /* Write to USART BRR register */
  USART3->BRR = (uint16_t)tmpreg;
	
}


void USART6_Init(USART_Type* Init)
{
	
	//Enable peripheral Clock
	RCC_USART6_CLK_ENABLE();
	
	//GPIO clock enable
	RCC_GPIOC_CLOCK_ENABLE_CLK();
	
	GPIO_Config_Mode (GPIOC,GPIO_PIN_6,GPIO_MODE_AF);        //TX pin 
  GPIO_Config_OType (GPIOC, GPIO_PIN_6, GPIO_OTYPE_PP); 
  GPIO_Config_Speed (GPIOC, GPIO_PIN_6, GPIO_HIGH_SPEED);    
  GPIO_Config_PuPd	(GPIOC,GPIO_PIN_6,GPIO_PUPD_NOPULL);
  GPIO_Config_AF(GPIOC, GPIO_PIN_6, 8); 

	GPIO_Config_Mode (GPIOC,GPIO_PIN_7,GPIO_MODE_AF);        //RX pin 
  GPIO_Config_Speed (GPIOC, GPIO_PIN_7, GPIO_HIGH_SPEED);    
  GPIO_Config_PuPd	(GPIOC,GPIO_PIN_7,GPIO_PUPD_NOPULL);
  GPIO_Config_AF(GPIOC, GPIO_PIN_7, 8); 
	
	
	uint32_t tmpreg = 0x00;
	uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
	
	/*---------------------------- USART CR2 Configuration -----------------------*/
	tmpreg = USART6->CR2;

  /* Clear STOP[13:12] bits */
  tmpreg &= (uint32_t)~((uint32_t)USART_CR2_STOP);

  /* Configure the USART Stop Bits:
      Set STOP[13:12] bits according to USART_StopBits value */
  tmpreg |= (uint32_t)Init->USART_StopBits;
  
  /* Write to USART CR2 */
  USART6->CR2 = (uint16_t)tmpreg;
	
	
	
	/*---------------------------- USART CR1 Configuration -----------------------*/
  tmpreg = USART6->CR1;

  /* Clear M, PCE, PS, TE and RE bits */
  tmpreg &= (uint32_t)~((uint32_t)CR1_CLEAR_MASK);

  /* Configure the USART Word Length, Parity and mode: 
     Set the M bits according to USART_WordLength value 
     Set PCE and PS bits according to USART_Parity value
     Set TE and RE bits according to USART_Mode value */
  tmpreg |= (uint32_t)Init->USART_WordLength | Init->USART_Parity |
            Init->USART_Mode;

  /* Write to USART CR1 */
  USART6->CR1 = (uint16_t)tmpreg;
	
	
	
	
	
	
	  /* Determine the integer part */
  if ((USART6->CR1 & USART_CR1_OVER8) != 0)
  {
    /* Integer part computing in case Oversampling mode is 8 Samples */
    integerdivider = 100*((48000000) / (8 * (Init->USART_BaudRate)));     
  }
  else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
  {
    /* Integer part computing in case Oversampling mode is 16 Samples */
    integerdivider = 100*((48000000) / (16 * (Init->USART_BaudRate)));        //multiplying by 100 to get the fractional part
  }
  tmpreg = (integerdivider / 100) << 4;

  /* Determine the fractional part */
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

  /* Implement the fractional part in the register */
  if ((USART6->CR1 & USART_CR1_OVER8) != 0)
  {
    tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
  }
  else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
  {
    tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
  }
  
  /* Write to USART BRR register */
  USART6->BRR = (uint16_t)tmpreg;
	

}









uint8_t USART_TxBusy(USART_TypeDef *USARTx)
{

if(USARTx->SR & USART_SR_TC)
	return 1;

else
	return 0;

}




////////////////////////////////////////////
///////Similar to Tiva C Course(on edX)


//------------UART_InChar------------
// Wait for new serial port input
// Input: none
// Output: ASCII code for key typed
char USART_InChar(USART_TypeDef *USARTx){
  
  return (uint8_t)(USARTx->DR & (uint16_t)0x00FF);
}

uint8_t USART_InHex(USART_TypeDef *USARTx) {
	
	return (uint8_t)(USARTx->DR);
}	


//------------UART_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void USART_OutChar(USART_TypeDef *USARTx,char data){
                  
  /* Transmit Data */
  USARTx->DR = ((uint16_t)data & (uint16_t)0x01FF);
	while((USARTx->SR & USART_SR_TC) == 0);  //Changed from SR_TXE to SR_TC
}

void USART_OutHex(USART_TypeDef *USARTx, uint8_t data) {
	
	USARTx->DR = data;
	while((USARTx->SR & USART_SR_TC) == 0);  //Changed from SR_TXE to SR_TC
}


//------------UART_InUDec------------
// InUDec accepts ASCII input in unsigned decimal format
//     and converts to a 32-bit unsigned number
//     valid range is 0 to 4294967295 (2^32-1)
// Input: none
// Output: 32-bit unsigned number
// If you enter a number above 4294967295, it will return an incorrect value
// Backspace will remove last digit typed
uint32_t USART_InUDec(USART_TypeDef *USARTx){
uint32_t number=0, length=0;
char character;
  character = USART_InChar(USARTx);
  while(character != CR){ // accepts until <enter> is typed
// The next line checks that the input is a digit, 0-9.

    if((character>='0') && (character<='9')) {
      number = 10*number+(character-'0');   // this line overflows if above 4294967295
      length++;
      USART_OutChar(USARTx,character);      //echo to terminal
    }
// If the input is a backspace, then the return number is
// changed and a backspace is outputted to the screen
    else if((character==BS) && length){
      number /= 10;
      length--;
      USART_OutChar(USARTx,character);
    }
    character = USART_InChar(USARTx);
  }
  return number;

}



//-----------------------UART_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void USART_OutUDec(USART_TypeDef *USARTx,uint32_t n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    USART_OutUDec(USARTx,n/10);
    n = n%10;
  }
  USART_OutChar(USARTx,n+'0'); /* n is between 0 and 9 */
}


	




void USART_SetBaudRate(USART_TypeDef *USARTx,uint32_t Baud)
{
  uint32_t tmpreg = 0x00;
	uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
	
	
	 /* Determine the integer part */
  if ((USART2->CR1 & USART_CR1_OVER8) != 0)
  {
    /* Integer part computing in case Oversampling mode is 8 Samples */
    integerdivider = 100*((24000000) / (8 * (Baud)));    
  }
  else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
  {
    /* Integer part computing in case Oversampling mode is 16 Samples */
    integerdivider =100*((24000000) / (16 * (Baud)));    
  }
  tmpreg = (integerdivider/100 ) << 4;

  /* Determine the fractional part */
  fractionaldivider = integerdivider - (100*(tmpreg >> 4));

  /* Implement the fractional part in the register */
  if ((USART2->CR1 & USART_CR1_OVER8) != 0)
  {
    tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
  }
  else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
  {
    tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
  }
  
  /* Write to USART BRR register */
  USART2->BRR = (uint16_t)tmpreg;

}






/* USART interrupt related functions *********************************************/


void USART_IntMaskEnable(USART_TypeDef *USARTx, uint32_t IntMask)
{

	
	/*For Control Register 3*/
	if(IntMask==USART_IT_CTS)
		USARTx->CR3 |= IntMask;
	
	/*For Control Register 1*/
	else
		USARTx->CR1 |= IntMask;

}	



void USART_IntMaskDisable(USART_TypeDef *USARTx, uint32_t IntMask)
{

	/*For Control Register 3*/
	if(IntMask==USART_IT_CTS)
		USARTx->CR3 &= ~IntMask;
	
	/*For Control Register 1*/
	else
		USARTx->CR1 &= ~IntMask;

	
	
}




void USART_IntFlagClear(USART_TypeDef *USARTx, uint32_t IntMask)
{

	
	switch(IntMask)
	{
		case USART_IT_CTS:
			USARTx->SR &= ~USART_SR_CTS;
		  break;
		
		case USART_IT_TC:
			USARTx->SR &= ~USART_SR_TC;
		  break;
		
		case USART_IT_RXNE:
			USARTx->SR &= ~USART_SR_RXNE;
		  break;
	
		
		default :
			break;
		
		//////////////////////
		///////////
	  //IDLE, PE and TXE interrupts are cleared using two step prosess:
		//	 (a read from the status register followed by a read or write access to the
    //   USART_DR data register)
		
		
		// see  manual
	}

}	



/*Use it carefully it may clear the flags******************/
/*Use in conjuction with flag macros  to determine required  flag*/
uint32_t USART_GetStatus(USART_TypeDef *USARTx)
{

  uint32_t status;
	status=USARTx->SR;
	return status;
	
	
}