#ifndef _UART_DRIVER_H
#define _UART_DRIVER_H

/*board specific header file (stm32f446re nucleo) */
#include "stm32f446xx.h"
#include <stdint.h>

// standard ASCII symbols
#define CR   0x0D  //Enter
#define BS   0x08  //BackSpace
#define LF   0x0A  //Line feed

/*Macros to enable the clocks for various UART ************************************/
#define RCC_USART1_CLK_ENABLE()           ( RCC->APB2ENR |=  ( 1 << 4))
#define RCC_USART2_CLK_ENABLE()           ( RCC->APB1ENR |=  ( 1 << 17))
#define RCC_USART3_CLK_ENABLE()           ( RCC->APB1ENR |=  ( 1 << 18))
#define RCC_USART6_CLK_ENABLE()           ( RCC->APB2ENR |=  ( 1 << 5))



/*UART possible error codes *******************************************************/
#define USART_ERROR_NONE         ((uint32_t)0x00000000)   /*!< No error            */
#define USART_ERROR_PE           ((uint32_t)0x00000001)   /*!< Parity error        */
#define USART_ERROR_FE           ((uint32_t)0x00000002)   /*!< Frame error         */
#define USART_ERROR_NE           ((uint32_t)0x00000004)   /*!< Noise error         */
#define USART_ERROR_ORE          ((uint32_t)0x00000008)   /*!< Overrun error       */


/*USART Init Structure definition **************************************************/ 
typedef struct
{
  uint32_t USART_BaudRate;            /*!< This member configures the USART communication baud rate.
                                           The baud rate is computed using the following formula:
                                            - IntegerDivider = ((PCLKx) / (8 * (OVR8+1) * (USART_InitStruct->USART_BaudRate)))
                                            - FractionalDivider = ((IntegerDivider - ((u32) IntegerDivider)) * 8 * (OVR8+1)) + 0.5 
                                           Where OVR8 is the "oversampling by 8 mode" configuration bit in the CR1 register. */

  uint16_t USART_WordLength;          /*!< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref USART_Word_Length */

  uint16_t USART_StopBits;            /*!< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref USART_Stop_Bits */

  uint16_t USART_Parity;              /*!< Specifies the parity mode.
                                           This parameter can be a value of @ref USART_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits). */
 
  uint16_t USART_Mode;                /*!< Specifies whether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref USART_Mode */

  uint16_t USART_HardwareFlowControl; /*!< Specifies wether the hardware flow control mode is enabled
                                           or disabled.
                                           This parameter can be a value of @ref USART_Hardware_Flow_Control */
} USART_Type;







/*USART_Word_Length *********************************************************/ 
#define USART_WORDLENGTH_8B                   ((uint16_t)0x0000)
#define USART_WORDLENGTH_9B                   ((uint16_t)0x1000)

/*USART_Stop_Bits ************************************************************/ 
#define USART_STOPBITS_1                      ((uint16_t)0x0000)
#define USART_STOPBITS_0_5                    ((uint16_t)0x1000)
#define USART_STOPBITS_2                      ((uint16_t)0x2000)
#define USART_STOPBITS_1_5                    ((uint16_t)0x3000)


/*USART_Parity ***************************************************************/ 
#define USART_PARITY_NO                       ((uint16_t)0x0000)
#define USART_PARITY_EVEN                     ((uint16_t)0x0400)
#define USART_PARITY_ODD                      ((uint16_t)0x0600)


/*USART_Mode ******************************************************************/ 
#define USART_MODE_Rx                         ((uint16_t)0x0004)
#define USART_MODE_Tx                         ((uint16_t)0x0008)
#define USART_MODE_Rx_Tx                      (uint16_t)(USART_MODE_Rx|USART_MODE_Tx)

//Only for USART1<---------------
/*USART_Hardware_Flow_Control **************************************************/ 
#define USART_HARDWAREFLOWCONTROL_NONE        ((uint16_t)0x0000)           
#define USART_HARDWAREFLOWCONTROL_RTS         ((uint16_t)0x0100)
#define USART_HARDWAREFLOWCONTROL_CTS         ((uint16_t)0x0200)
#define USART_HARDWAREFLOWCONTROL_RTS_CTS     ((uint16_t)0x0300)






/*******USART_Interrupt_definition ***************************************/
#define USART_IT_PE                          USART_CR1_PEIE
#define USART_IT_TXE                         USART_CR1_TXEIE
#define USART_IT_TC                          USART_CR1_TCIE
#define USART_IT_RXNE                        USART_CR1_RXNEIE
#define USART_IT_IDLE                        USART_CR1_IDLEIE
#define USART_IT_CTS                         USART_CR3_CTSIE







void USART1_Init(USART_Type* Init);
void USART2_Init(USART_Type* Init);
void USART3_Init(USART_Type* Init);
void USART6_Init(USART_Type* Init);
void USART_SetBaudRate(USART_TypeDef *USARTx,uint32_t baud);

void USART_Enable(USART_TypeDef *uartx);
void USART_Disable(USART_TypeDef *uartx);

//RX Enable & Disable 
void USART_EnableRx(USART_TypeDef *uartx);
void USART_DisableRx(USART_TypeDef *uartx);

//For ASCII
char USART_InChar(USART_TypeDef *USARTx);
void USART_OutChar(USART_TypeDef *USARTx,char data);

//For HEX
uint8_t USART_InHex(USART_TypeDef *USARTx);
void USART_OutHex(USART_TypeDef *USARTx, uint8_t data);


//For Decimal numbers
uint32_t USART_InUDec(USART_TypeDef *USARTx);
void USART_OutUDec(USART_TypeDef *USARTx,uint32_t n);

uint8_t USART_TxBusy(USART_TypeDef *USARTx);

/* USART interrupt related functions *********************************************/
void USART_IntMaskEnable(USART_TypeDef *USARTx, uint32_t IntMask);
void USART_IntMaskDisable(USART_TypeDef *USARTx, uint32_t IntMask);
void USART_IntFlagClear(USART_TypeDef *USARTx, uint32_t IntMask);

uint32_t USART_GetStatus(USART_TypeDef *USARTx);



#endif
