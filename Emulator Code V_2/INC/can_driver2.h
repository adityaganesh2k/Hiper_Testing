#ifndef _CAN_DRIVER_H
#define _CAN_DRIVER_H

/*board specific header file (stm32f446re nucleo) */
#include "stm32f446xx.h"
#include <stdint.h>


/* Macros to ENABLE clock for differnt CAN using RCC Register */
#define RCC_CAN1_CLOCK_ENABLE_CLK()   (RCC->APB1ENR |= (1<<25))
#define RCC_CAN2_CLOCK_ENABLE_CLK()   (RCC->APB1ENR |= (1<<26))


/* Macros to ENABLE clock for differnt CAN using RCC Register in LOW POWER Mode */
#define RCC_CAN1_CLOCK_ENABLE_LP_CLK()   (RCC->APB1LPENR |= (1<<25))
#define RCC_CAN2_CLOCK_ENABLE_LP_CLK()   (RCC->APB1LPENR |= (1<<26))




/****************************************************/

/*      Register definition for CAN Driver      */
/*                                                  */
/*                                                  */
/****************************************************/

#define CAN_MSR_REG         1
#define CAN_TSR_REG         2
#define CAN_RF0R_REG        3
#define CAN_RF1R_REG        4
#define CAN_ESR_REG         5


/******definition for CAN_MCR 	register*********/
//#define CAN_INRQ_ENTER                            1              
//#define CAN_INRQ_EXIT                             0

//#define CAN_SLEEP_ENTER                           1              /* This bit is cleared by hardware when the AWUM bit */
//#define CAN_SLEEP_EXIT                            0              /*   is set and a SOF bit is detected on the         */

#define CAN_TXFIFO_PRIO_BY_ID                     ((uint32_t) 0x01 )
#define CAN_TXFIFO_PRIO_BY_ORDER                  ((uint32_t) 0x00 )

#define CAN_RXFIFO_OVERRUN_UNLOCKED               ((uint32_t) 0x00 )    //see this 
#define CAN_RXFIFO_OVERRUN_LOCKED                 ((uint32_t) 0x01 )

#define CAN_NART_ENABLE                           ((uint32_t) 0x01 )             
#define CAN_NART_DISABLE                          ((uint32_t) 0x00 )

#define CAN_AWUM_ENABLE                           ((uint32_t) 0x01 )              
#define CAN_AWUM_DISABLE                          ((uint32_t) 0x00 )

#define CAN_ABOM_ENABLE                           ((uint32_t) 0x01 )              
#define CAN_ABOM_DISABLE                          ((uint32_t) 0x00 )
                      
#define CAN_TTCM_ENABLE                           ((uint32_t) 0x01 )              
#define CAN_TTCM_DISABLE                          ((uint32_t) 0x00 )

#define IS_CAN_CONFIG_PARAM(X)  ((X)<=0x01)

//#define CAN_MASTER_RESET                          1              /* Force a master reset of the bxCAN -> Sleep mode activated  */
//#define CAN_MASTER_NORMAL                         0              /* after reset. */




/****************************************************/

/* "Bit-Timing" macros for CAN 
    Related to "BTR" Register*/

/* TQ->Time Quanta */
/****************************************************/

/* CAN Synchronization Jump Width */
#define CAN_SJW_1TQ                 ((uint32_t)0x00)         /*!< 1 time quantum */
#define CAN_SJW_2TQ                 ((uint32_t)0x01)         /*!< 2 time quantum */
#define CAN_SJW_3TQ                 ((uint32_t)0x02)         /*!< 3 time quantum */
#define CAN_SJW_4TQ                 ((uint32_t)0x03)         /*!< 4 time quantum */

#define IS_CAN_SJW(SJW) (((SJW) == CAN_SJW_1TQ) || ((SJW) == CAN_SJW_2TQ)|| \
                         ((SJW) == CAN_SJW_3TQ) || ((SJW) == CAN_SJW_4TQ))


/* CAN Time Quantum in Bit Segment 1 */
#define CAN_BS1_1TQ                 ((uint32_t)0x00)            /*!< 1 time quantum */                   
#define CAN_BS1_2TQ                 ((uint32_t)0x01)            /*!< 2 time quantum */
#define CAN_BS1_3TQ                 ((uint32_t)0x02)            /*!< 3 time quantum */
#define CAN_BS1_4TQ                 ((uint32_t)0x03)            /*!< 4 time quantum */
#define CAN_BS1_5TQ                 ((uint32_t)0x04)            /*!< 5 time quantum */
#define CAN_BS1_6TQ                 ((uint32_t)0x05)            /*!< 6 time quantum */
#define CAN_BS1_7TQ                 ((uint32_t)0x06)            /*!< 7 time quantum */
#define CAN_BS1_8TQ                 ((uint32_t)0x07)            /*!< 8 time quantum */
#define CAN_BS1_9TQ                 ((uint32_t)0x08)            /*!< 9 time quantum */
#define CAN_BS1_10TQ                ((uint32_t)0x09)            /*!< 10 time quantum */
#define CAN_BS1_11TQ                ((uint32_t)0x0A)            /*!< 11 time quantum */
#define CAN_BS1_12TQ                ((uint32_t)0x0B)            /*!< 12 time quantum */
#define CAN_BS1_13TQ                ((uint32_t)0x0C)            /*!< 13 time quantum */
#define CAN_BS1_14TQ                ((uint32_t)0x0D)            /*!< 14 time quantum */
#define CAN_BS1_15TQ                ((uint32_t)0x0E)            /*!< 15 time quantum */
#define CAN_BS1_16TQ                ((uint32_t)0x0F)            /*!< 16 time quantum */

#define IS_CAN_BS1(BS1) ((BS1) <= CAN_BS1_16TQ)       


/* CAN Time Quantum in Bit Segment 2 */
#define CAN_BS2_1TQ                 ((uint32_t)0x00)             /*!< 1 time quantum */
#define CAN_BS2_2TQ                 ((uint32_t)0x01)             /*!< 2 time quantum */
#define CAN_BS2_3TQ                 ((uint32_t)0x02)             /*!< 3 time quantum */
#define CAN_BS2_4TQ                 ((uint32_t)0x03)             /*!< 4 time quantum */
#define CAN_BS2_5TQ                 ((uint32_t)0x04)             /*!< 5 time quantum */
#define CAN_BS2_6TQ                 ((uint32_t)0x05)             /*!< 6 time quantum */
#define CAN_BS2_7TQ                 ((uint32_t)0x06)             /*!< 7 time quantum */
#define CAN_BS2_8TQ                 ((uint32_t)0x07)             /*!< 8 time quantum */

#define IS_CAN_BS2(BS2) ((BS2) <= CAN_BS2_8TQ)




/* CAN Operating Mode */
/*see BTR register*/
#define CAN_MODE_NORMAL             ((uint32_t)0x00)              /*!< normal mode */ 
#define CAN_MODE_LOOPBACK           ((uint32_t)0x01)              /*!< loopback mode */
#define CAN_MODE_SILENT             ((uint32_t)0x02)              /*!< silent mode */
#define CAN_MODE_SILENT_LOOPBACK    ((uint32_t)0x03)              /*!< loopback combined with silent mode */

#define IS_CAN_MODE(MODE) (((MODE) == CAN_MODE_NORMAL) || \
                           ((MODE) == CAN_MODE_LOOPBACK)|| \
                           ((MODE) == CAN_MODE_SILENT) || \
                           ((MODE) == CAN_MODE_SILENT_LOOPBACK))





/****************************************************/
/* "Filter and Identifier" macros for CAN */
/* FMR, FM1R, FS1R, FFA1R Reistes  */
/****************************************************/
/* CAN Filter Mode */
#define CAN_FILTER_ACTIVE           (0x00000000U)                      /*!<  Active filters mode */
#define CAN_FILTER_INIT             (0x00000001U)                      /*!< Initialization mode for the filters */


/* CAN Filter Mode */
#define CAN_FILTERMODE_IDMASK       (0x00000000U)                       /*!< Identifier mask mode */
#define CAN_FILTERMODE_IDLIST       (0x00000001U)                       /*!< Identifier list mode */
#define IS_CAN_FILTER_MODE(MODE) (((MODE) == CAN_FILTERMODE_IDMASK) || \
                                  ((MODE) == CAN_FILTERMODE_IDLIST))


/* CAN Filter Scale */
#define CAN_FILTERSCALE_16BIT       (0x00000000U)                         /*!< Two 16-bit filters */
#define CAN_FILTERSCALE_32BIT       (0x00000001U)                         /*!< One 32-bit filter  */
#define IS_CAN_FILTER_SCALE(SCALE) (((SCALE) == CAN_FILTERSCALE_16BIT) || \
                                    ((SCALE) == CAN_FILTERSCALE_32BIT))
																		

/* CAN Filter FIFO Assignment */
#define CAN_FILTER_FIFO0            (0x00000000U)                         /*!< Filter FIFO 0 assignment for filter x */
#define CAN_FILTER_FIFO1            (0x00000001U)                         /*!< Filter FIFO 1 assignment for filter x */
#define IS_CAN_FILTER_FIFO(FIFO) (((FIFO) == CAN_FILTER_FIFO0) || \
                                  ((FIFO) == CAN_FILTER_FIFO1))
																	

/* CAN Identifier Type */
#define CAN_ID_STD                  (0x00000000U)                         /*!< Standard Id */
#define CAN_ID_EXT                  (0x00000001U)                         /*!< Extended Id */
#define IS_CAN_IDTYPE(IDTYPE)  (((IDTYPE) == CAN_ID_STD) || \
                                ((IDTYPE) == CAN_ID_EXT))


/* CAN Remote Transmission Request */
#define CAN_RTR_DATA                (0x00000000U)                         /*!< Data frame   */
#define CAN_RTR_REMOTE              (0x00000001U)                         /*!< Remote frame */
#define IS_CAN_RTR(RTR) (((RTR) == CAN_RTR_DATA) || ((RTR) == CAN_RTR_REMOTE))




/****************************************************/

/* "Message Storage" macros for CAN */

/****************************************************/
/* CAN Receive FIFO Number */
#define CAN_RX_FIFO0                (0x00000000U)                           /*!< CAN receive FIFO 0 */
#define CAN_RX_FIFO1                (0x00000001U)                           /*!< CAN receive FIFO 1 */
#define IS_CAN_RX_FIFO(FIFO) (((FIFO) == CAN_RX_FIFO0) || ((FIFO) == CAN_RX_FIFO1))


/* CAN Tx Mailboxes */
#define CAN_TX_MAILBOX0             (0x00000000U)                                  /*!< Tx Mailbox 0  */
#define CAN_TX_MAILBOX1             (0x00000001U)                                  /*!< Tx Mailbox 1  */
#define CAN_TX_MAILBOX2             (0x00000002U)                                  /*!< Tx Mailbox 2  */
#define IS_CAN_TX_MAILBOX(TRANSMITMAILBOX) (((TRANSMITMAILBOX) == CAN_TX_MAILBOX0 ) || \
                                            ((TRANSMITMAILBOX) == CAN_TX_MAILBOX1 ) || \
                                            ((TRANSMITMAILBOX) == CAN_TX_MAILBOX2 ))



/****************************************************/
/* Macros related to Interrupts */
/* "IER" Register */
/* interrupt mask ----> IntMask*/         //see IntEnable Function
/****************************************************/

/* Transmit interrupts */ 
#define CAN_IT_TME                  ((uint32_t)0x00000001)           /*!< Transmit mailbox empty Interrupt*/

/* Receive Interrupts */
#define CAN_IT_FMP0                 ((uint32_t)0x00000002)           /*!< FIFO 0 message pending Interrupt*/
#define CAN_IT_FF0                  ((uint32_t)0x00000004)           /*!< FIFO 0 full Interrupt*/
#define CAN_IT_FOV0                 ((uint32_t)0x00000008)           /*!< FIFO 0 overrun Interrupt*/

#define CAN_IT_FMP1                 ((uint32_t)0x00000010)           /*!< FIFO 1 message pending Interrupt*/
#define CAN_IT_FF1                  ((uint32_t)0x00000020)           /*!< FIFO 1 full Interrupt*/
#define CAN_IT_FOV1                 ((uint32_t)0x00000040)           /*!< FIFO 1 overrun Interrupt*/

/* Operating Mode Interrupts */
#define CAN_IT_WKU                  ((uint32_t)0x00010000)           /*!< Wake-up Interrupt*/
#define CAN_IT_SLK                  ((uint32_t)0x00020000)           /*!< Sleep acknowledge Interrupt*/

/* Error Interrupts */
#define CAN_IT_EWG                  ((uint32_t)0x00000100)           /*!< Error warning Interrupt*/
#define CAN_IT_EPV                  ((uint32_t)0x00000200)           /*!< Error passive Interrupt*/
#define CAN_IT_BOF                  ((uint32_t)0x00000400)           /*!< Bus-off Interrupt*/
#define CAN_IT_LEC                  ((uint32_t)0x00000800)           /*!< Last error code Interrupt*/
#define CAN_IT_ERR                  ((uint32_t)0x00008000)           /*!< Error Interrupt*/





/****************************************************/

/* Macros related to Errors */
/* "ESR" Register */
/****************************************************/                                                     
#define CAN_NO_ERR           ((uint8_t)0x00)                       /*!< No Error */ 
#define	CAN_STUFF_ERR        ((uint8_t)0x10)                       /*!< Stuff Error */ 
#define	CAN_FORM_ERR         ((uint8_t)0x20)                       /*!< Form Error */ 
#define	CAN_ACK_ERR          ((uint8_t)0x30)                       /*!< Acknowledgment Error */ 
#define	CAN_BITRECESSIVE_ERR ((uint8_t)0x40)                       /*!< Bit Recessive Error */ 
#define	CAN_BITDOMINANT_ERR  ((uint8_t)0x50)                       /*!< Bit Dominant Error */ 
#define	CAN_CRC_ERR          ((uint8_t)0x60)                       /*!< CRC Error  */ 
#define	CAN_SOFTWARESET_ERR  ((uint8_t)0x70)                       /*!< Software Set Error */ 




/****************************************************/

/* Structure definition for CAN Driver */

/****************************************************/

typedef enum 
{
	DISABLE = 0, 
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))



/*CAN "Clock Parameter" init structure definition */
/* "BTR" register */
typedef struct
{
  uint32_t Prescaler;                  /*!< Specifies the length of a time quantum.
                                            This parameter must be a number between Min_Data = 1 and Max_Data = 1024. */

  uint32_t Mode;                       /*!< Specifies the CAN operating mode.
                                            This parameter can be a value of CAN_operating_mode 
	                                          "Normal Mode", "Silent mode" etc    see Macros*/

  uint32_t SJW;                        /*!< Specifies the maximum number of time quanta the CAN hardware
                                            is allowed to lengthen or shorten a bit to perform resynchronization.
                                            This parameter can be a value of CAN_synchronisation_jump_width */
	
  uint32_t BS1;                        /*!< Specifies the number of time quanta in Bit Segment 1.
                                            This parameter can be a value of CAN_time_quantum_in_bit_segment_1 */

  uint32_t BS2;                        /*!< Specifies the number of time quanta in Bit Segment 2.
                                            This parameter can be a value of CAN_time_quantum_in_bit_segment_2 */

} CAN_ClockParamTypeDef;



/*CAN "Config Parameter" init structure definition */
/* in "MCR" register */

typedef struct
{	
  uint32_t TimeTriggeredMode;   /*!< Enable or disable the time triggered communication mode.
                                            This parameter can be set to CAN_TTCM_ENABLE or CAN_TTCM_DISABLE. */

  uint32_t AutoBusOff;          /*!< Enable or disable the automatic bus-off management.
                                            This parameter can be set to CAN_ABOM_ENABLE or CAN_ABOM_DISABLE */

  uint32_t AutoWakeUp;          /*!< Enable or disable the automatic wake-up mode.
                                            This parameter can be set to CAN_AWUM_ENABLE or CAN_AWUM_DISABLE. */

  uint32_t NoAutoRetransmission;  /*!< Enable or disable the non-automatic retransmission mode.
                                            This parameter can be set to CAN_NART_ENABLE or CAN_NART_DISABLE */

  uint32_t ReceiveFifoLocked;   /*!< Enable or disable the Receive FIFO Locked mode.
                                            This parameter can be set to CAN_RXFIFO_OVERRUN_UNLOCKED or CAN_RXFIFO_OVERRUN_LOCKED.*/

  uint32_t TransmitFifoPriority;/*!< Enable or disable the transmit FIFO priority.
                                            This parameter can be set to CAN_TXFIFO_PRIO_BY_ID  or CAN_TXFIFO_PRIO_BY_ORDER */

} CAN_ConfigParamTypeDef;







/* CAN Tx message structure definition  */


typedef struct
{
  uint32_t StdId;   /*!< Specifies the standard identifier.
                        This parameter can be a value between 0 to 0x7FF. */

  uint32_t ExtId;   /*!< Specifies the extended identifier.
                        This parameter can be a value between 0 to 0x1FFFFFFF. */

  uint32_t IDE;     /*!< Specifies the type of identifier for the message that 
                        will be transmitted. This parameter can be a value 
                        of  CAN_identifier_type */

  uint32_t RTR;     /*!< Specifies the type of frame for the message that will 
                        be transmitted. This parameter can be a value of 
                        CAN_remote_transmission_request */

  uint32_t DLC;     /*!< Specifies the length of the frame that will be 
                        transmitted. This parameter can be a value between 
                        0 to 8 */

  uint8_t Data[8]; /*!< Contains the data to be transmitted. It ranges from 0 
                        to 0xFF. */
} CAN_TxMsgType;



/** 
  * @brief  CAN Rx message structure definition  
  */

typedef struct
{
  uint32_t StdId;  /*!< Specifies the standard identifier.
                        This parameter can be a value between 0 to 0x7FF. */

  uint32_t ExtId;  /*!< Specifies the extended identifier.
                        This parameter can be a value between 0 to 0x1FFFFFFF. */

  uint8_t IDE;     /*!< Specifies the type of identifier for the message that 
                        will be received. This parameter can be a value of 
                         CAN_identifier_type */

  uint8_t RTR;     /*!< Specifies the type of frame for the received message.
                        This parameter can be a value of 
                        CAN_remote_transmission_request */
	                  /* see RIxR rgister for all above */

  uint8_t DLC;     /*!< Specifies the length of the frame that will be received.
                        This parameter can be a value between 0 to 8 */

  uint8_t Data[8]; /*!< Contains the data to be received. It ranges from 0 to 
                        0xFF. */

  uint8_t FMI;     /*!< Specifies the index of the filter the message stored in 
                        the mailbox passes through. This parameter can be a 
                        value between 0 to 0xFF 
												see RDTxR register*/
} CAN_RxMsgType;








/****************************************************/
/* Macros related to Flags */
/****************************************************/
/*If required*/
/////////////////////not defined yet//////////////////////////////////////////////////////////



/* for assert test*/
#define IS_CAN_DLC(DLC)       ((DLC) <= ((uint32_t)0x08))
#define IS_CAN_PRESCALER(PRESCALER) (((PRESCALER) >= 1) && ((PRESCALER) <= 1024))
#define IS_CAN_STDID(STDID)   ((STDID) <= 0x7FFU)
#define IS_CAN_EXTID(EXTID)   ((EXTID) <= 0x1FFFFFFFU)
#define IS_CAN_STARTBANK(BANK) (((BANK)>= 1 ) && ((BANK)<=27))
#define IS_CAN_REGISTERNO(RNO)  (((RNO) >=1) && ((RNO) <= 2 ))



/* Hardware Initialization */
/*  and Configuration      */
void CAN_Init_Req(CAN_TypeDef* CANx);                            //(after timer API is done, make it uint8_t type and add TIMEOUT)
void CAN_Init_Clock(CAN_TypeDef* CANx, CAN_ClockParamTypeDef* ClockParam);
void CAN_Init_Config(CAN_TypeDef* CANx, CAN_ConfigParamTypeDef* ConfigParam);
void CAN_Init_Quit(CAN_TypeDef* CANx);                          //(after timer API is done , make it uint8_t type and add TIMEOUT)
void CAN_DBGFreeze(CAN_TypeDef* CANx, FunctionalState NewState);



/* Filter Initialization             */          
/*  and Configuration only for CAN1  */
void CAN_FilterInit_Req(uint8_t FilterNumber);                                 //see FMR register
void CAN_FilterInit_SlaveBankStart(uint32_t StartBank);                        //see FMR
void CAN_FilterInit_FilterMode(uint8_t FilterNumber,uint32_t Mode);            //see FM1R register
void CAN_FilterInit_FilterScale(uint8_t FilterNumber,uint32_t Scale);          //see FS1R register
void CAN_FilterInit_FilterAssign(uint8_t FilterNumber,uint32_t FIFONo);        //see FFA1R register
void CAN_FilterActivate(uint8_t FilterNumber);                                 //see FA1R register
void CAN_FilterDeActivate(uint8_t FilterNumber);
void CAN_FilterInit_Quit(void);
void CAN_Filter_IDMaskModify(uint8_t FilterNumber, uint8_t RegisterNo, uint32_t value);   //see FiRx register filter no.<=27, x<=1




//void CAN_Sleep_Req(CAN_TypeDef* CANx);                           //(after timer API is done, make it uint8_t type and add TIMEOUT)
//void CAN_WakeUp_Req(CAN_TypeDef* CANx);                          //(after timer API is done, make it uint8_t type and add TIMEOUT)



/* CAN Frames Transmission functions ******************************************/
uint32_t CAN_Which_MailboxIsEmpty(CAN_TypeDef* CANx);
void CAN_Transmit(CAN_TypeDef* CANx, CAN_TxMsgType* TxMsg,uint32_t MailBoxNo);
void CAN_CancelTransmit(CAN_TypeDef* CANx, uint32_t Mailbox);


/* CAN Frames Reception functions *********************************************/
void CAN_Receive(CAN_TypeDef* CANx, uint32_t FIFONo, CAN_RxMsgType* RxMsg);
uint8_t CAN_MessagePending(CAN_TypeDef* CANx, uint32_t FIFONo);
void CAN_FIFORelease(CAN_TypeDef* CANx, uint32_t FIFONo);


/* CAN interrupt related functions *********************************************/
void CAN_IntMaskEnable(CAN_TypeDef* CANx, uint32_t IntMask);
void CAN_IntMaskDisable(CAN_TypeDef* CANx, uint32_t IntMask);
void CAN_IntFlagClear(CAN_TypeDef* CANx, uint32_t IntMask);


/*Status Related functions */
uint32_t CAN_StatusRegisterGet(CAN_TypeDef* CANx,uint8_t REG_NAME);

//Implement this FlagClear() if required ...IntFlagClear() may take care 
//uint32_t CAN_FlagClear(CAN_TypeDef* CANx,register name, flag);



#endif

