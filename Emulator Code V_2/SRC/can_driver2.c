#include <stdint.h>
#include <assert.h>
#include "can_driver2.h"


/****************************************************/ 
/* Driver API definition  */ 
/****************************************************/ 


/*Request hardware to Enter in Initialization mode  */
/*Parameters: CAN module no.(1 or 2)                */
/*Return: none                                      */
void CAN_Init_Req(CAN_TypeDef* CANx)
{
   /* Exit from sleep mode */
   CANx->MCR &= (~(uint32_t)CAN_MCR_SLEEP);
	
  /* Request initialisation */
  CANx->MCR |= CAN_MCR_INRQ ;
	
  /* Wait the acknowledge */
  while (((CANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK))
  {
   
  }
		
}

/*CAN BTR register can only be modified if it is in INIT. mode     */


/*Configure bit-timing parameters                                   */
/*Parameters: CAN module no.(1 or 2),  CAN_ClockParamTypeDef object */
/*Return: none                                                      */
void CAN_Init_Clock(CAN_TypeDef* CANx, CAN_ClockParamTypeDef* ClockParam)
{

	assert(IS_CAN_PRESCALER(ClockParam->Prescaler));
  assert(IS_CAN_MODE(ClockParam->Mode));
	assert(IS_CAN_BS2(ClockParam->BS2));
	assert(IS_CAN_BS1(ClockParam->BS1));
	assert(IS_CAN_SJW(ClockParam->SJW));
	
	
	/*clear the required position */
	CANx->BTR &= ~((0x03 << 24) | (0x07 << 20) | (0x0F << 16)| (0x3FF));
                           
	
	/* Set the bit timing register */
  CANx->BTR |= (uint32_t)(ClockParam->Mode << 30) | 
                          (ClockParam->SJW << 24) | 
                          (ClockParam->BS1 << 16) | 
                          (ClockParam->BS2 << 20) |            //if required bits are not clered then use
                          (ClockParam->Prescaler - 1);        //  BTR=() because RESET state of register is NON-ZERO

}


/*Configure additional parameters                                    */
/*Parameters: CAN module no.(1 or 2),  CAN_ConfigParamTypeDef object */
/*Return: none                                                       */

void CAN_Init_Config(CAN_TypeDef* CANx, CAN_ConfigParamTypeDef* ConfigParam)
{

   assert(IS_CAN_CONFIG_PARAM(ConfigParam->AutoBusOff));
   assert(IS_CAN_CONFIG_PARAM(ConfigParam->NoAutoRetransmission));
	 assert(IS_CAN_CONFIG_PARAM(ConfigParam->AutoWakeUp));
	 assert(IS_CAN_CONFIG_PARAM(ConfigParam->ReceiveFifoLocked));
	 assert(IS_CAN_CONFIG_PARAM(ConfigParam->TimeTriggeredMode));
	 assert(IS_CAN_CONFIG_PARAM(ConfigParam->TransmitFifoPriority));

   /*clear the required position */
   CANx->MCR &=~(uint32_t)((1 << 2) | (1 << 3) | ( 1<< 4) | (1 <<5 ) | (1 << 6) | (1 << 7)) ;	 //clear required bits
	
	    
   /* Set the time triggered communication mode */
   CANx->MCR |= (uint32_t)(ConfigParam->TransmitFifoPriority << 2 ) |
	                        (ConfigParam->ReceiveFifoLocked << 3)     |
	                        (ConfigParam->NoAutoRetransmission << 4 ) |
	                        (ConfigParam->AutoWakeUp << 5)            |
	                        (ConfigParam->AutoBusOff) << 6            |
	                        (ConfigParam->TimeTriggeredMode << 7);
    
	
}


/*Leave the Initialization mode                  */
/*Parameters: CAN module no.(1 or 2)             */
/*Return: none                                   */

void CAN_Init_Quit(CAN_TypeDef* CANx)
{

   /* Request leave initialisation */
   CANx->MCR &= ~(uint32_t)CAN_MCR_INRQ;

   /* Wait the acknowledge */
   while (((CANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK))
   {
     
   }

}

/*Set Debug Environment CAN property  (freeze or Normal)    */   // see manual
/*Parameters: CAN module no.(1 or 2), (ENABLE or DISABLE)   */
/*Return: none                                              */

void CAN_DBGFreeze(CAN_TypeDef* CANx, FunctionalState NewState)
{
  /* Check the parameters */
  assert(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable Debug Freeze  */
    CANx->MCR |= (uint32_t)(1 << 16);
  }
  else
  {
    /* Disable Debug Freeze */
    CANx->MCR &= ~(uint32_t)(1 << 16);
  }
}




/********************************************************************8*/


/*Request Filter Initialization mode                  */
/*Parameters: filter no. (0 to 27)                    */  //see manual for more..
/*Return: none                                        */
void CAN_FilterInit_Req(uint8_t FilterNumber)      
{

  /* Filter Deactivation */
  CAN1->FA1R &= ~(uint32_t)(1 << FilterNumber);
	
	/* Initialisation mode for the filter */
  CAN1->FMR |= CAN_FILTER_INIT;
		
}
	


/*CAN Slave Start Bank Configuration                                 */
/*Parameters: start of SLAVE filter ( 1 to 27)                       */  
/*Return: none                                                       */
/*000001 = 1 filter assigned to CAN1 and 27 assigned to CAN2
  011011 = 27 filters assigned to CAN1 and 1 filter assigned to CAN2 */  //see manual for more

void CAN_FilterInit_SlaveBankStart(uint32_t StartBank)
{
  assert(IS_CAN_STARTBANK(StartBank));
	
	/* Initialisation mode for the filter */
  CAN1->FMR |= CAN_FILTER_INIT;                   //not required if already done...
	
  /*slave start bank configuration */
	CAN1->FMR &= (0xFFFFC0FF);                      //clear the required position 
	CAN1->FMR |= (uint32_t)((StartBank) << 8);
	
	
}

/*Filter Mode Configuration                                         */
/*Parameters: filter no.(0 to 27),mode (id or mask) see MACROS      */  
/*Return: none                                                      */
void CAN_FilterInit_FilterMode(uint8_t FilterNumber,uint32_t Mode)
{
  assert(IS_CAN_FILTER_MODE(Mode));
	 
	/* Filter Mode */
  if (Mode == CAN_FILTERMODE_IDMASK)
  {
    /*Id/Mask mode for the filter*/
    CAN1->FM1R &= ~(uint32_t)(1 << FilterNumber);
  }
  else 
  {
    /*Identifier list mode for the filter*/
    CAN1->FM1R |= (uint32_t)(1 << FilterNumber);
  }
	 
}	


/*Filter Scale Configuration                                     */
/*Parameters: filter no. (0 to 27), (16 bit or 32 bit) see MACROS*/  //see manual for more..
/*Return: none                                                   */
void CAN_FilterInit_FilterScale(uint8_t FilterNumber,uint32_t Scale)
{
  assert(IS_CAN_FILTER_FIFO(Scale));
	 
	/* Filter Scale */
  if (Scale == CAN_FILTERSCALE_16BIT)
  {
    /* 16-bit scale for the filter */
    CAN1->FS1R &= ~(uint32_t)(1 << FilterNumber);
	}
	
	else
	{
	
	  /* 32-bit scale for the filter */
    CAN1->FS1R |= (uint32_t)(1 << FilterNumber);
		
	}
	
}




/*FIFO Assignment Configuration                                */
/*Parameters: filter no. (0 to 27), FIFO NO (0 to 1) see MACROS*/  //see manual for more..
/*Return: none                                                 */
void CAN_FilterInit_FilterAssign(uint8_t FilterNumber,uint32_t FIFONo)
{
  assert(IS_CAN_FILTER_FIFO(FIFONo));
  
	/* Filter FIFO assignment */
  if (FIFONo == CAN_FILTER_FIFO0)
  {
    /* FIFO 0 assignation for the filter */
    CAN1->FFA1R &= ~(uint32_t)(1 << FilterNumber);
  }

  else
  {
    /* FIFO 1 assignation for the filter */
    CAN1->FFA1R |= (uint32_t)(1 << FilterNumber);
  }
  
}


/* Filter Activation          */
/*                            */
void CAN_FilterActivate(uint8_t FilterNumber)
{

  /* Filter activation */
  CAN1->FA1R |= (uint32_t)(1 << FilterNumber);
	
	/* Leave Initialisation mode for the filter */
  //CAN1->FMR &= ~CAN_FILTER_INIT;             ///commented this 
	
}


/*quit initaliztion mode */  

void CAN_FilterInit_Quit(void)
{
 
	/* Leave Initialisation mode for the filter */
  CAN1->FMR &= ~CAN_FILTER_INIT;
  
}



/* Filter DeActivation        */
/*                            */
void CAN_FilterDeActivate(uint8_t FilterNumber)
{

	/* Filter Deactivation */
  CAN1->FA1R &= ~(uint32_t)(1 << FilterNumber);
	
}


/*Modify the Filter registers                                                                          */
/*according to the requirement                                                                         */
/*Parameters: filter no. (0 to 27), register no. (1->(id) or 2->(mask)), value according to identifier */  //see manual for more..
/*Return: none                                                                                         */
void CAN_Filter_IDMaskModify(uint8_t FilterNumber, uint8_t RegisterNo, uint32_t Value)
{
	assert(IS_CAN_REGISTERNO(RegisterNo));
	
  /* Filter Deactivation */
  CAN1->FA1R &= ~(uint32_t)(1 << FilterNumber);
	
	if(RegisterNo == 1)
	 {
	   CAN1->sFilterRegister[FilterNumber].FR1 = Value;
	 }
	
	else
	 {
     CAN1->sFilterRegister[FilterNumber].FR2 = Value;
	 }
	
	/* Filter activation */
  CAN1->FA1R |= (uint32_t)(1 << FilterNumber);
	

}

/*Trasmit a message object.                                                      */
/*Parameters: CAN module(1 or 2), msg Object, mailbox no.(0 or 1 or 2) see MACROS*/  
/*Return: none                                                                   */

void CAN_Transmit(CAN_TypeDef* CANx, CAN_TxMsgType* TxMsg, uint32_t MailBoxNo)
{
  
	  assert(IS_CAN_TX_MAILBOX(MailBoxNo));

	  /* Set up the Id */
    CANx->sTxMailBox[MailBoxNo].TIR &= (uint32_t)0x01;   //clear all bit and leave TXRQ as earlier
    if (TxMsg->IDE == CAN_ID_STD)
    {
      assert(IS_CAN_STDID(TxMsg->StdId));  
      CANx->sTxMailBox[MailBoxNo].TIR |= ((TxMsg->StdId << 21)   |       //see for this part, the register definition and placement. 
                                          (TxMsg->IDE << 2)      |     //<---not necessary
                                          (TxMsg->RTR << 1));
    }
    else
    {
      assert(IS_CAN_EXTID(TxMsg->ExtId));
      CANx->sTxMailBox[MailBoxNo].TIR |= ((TxMsg->ExtId << 3)   | 
                                          (TxMsg->IDE << 2)     | 
                                          (TxMsg->RTR << 1));
 		}
    
		
		
		
		/* Set up the DLC */
    CANx->sTxMailBox[MailBoxNo].TDTR &= (uint32_t)0xFFFFFFF0;
    CANx->sTxMailBox[MailBoxNo].TDTR |= TxMsg->DLC;

    /* Set up the data field */
    CANx->sTxMailBox[MailBoxNo].TDLR = (((uint32_t)TxMsg->Data[3] << 24)      | 
                                             ((uint32_t)TxMsg->Data[2] << 16) |
                                             ((uint32_t)TxMsg->Data[1] << 8)  | 
                                             ((uint32_t)TxMsg->Data[0]));
    CANx->sTxMailBox[MailBoxNo].TDHR = (((uint32_t)TxMsg->Data[7] << 24)      | 
                                             ((uint32_t)TxMsg->Data[6] << 16) |
                                             ((uint32_t)TxMsg->Data[5] << 8)  |
                                             ((uint32_t)TxMsg->Data[4]));
    /* Request transmission */
    CANx->sTxMailBox[MailBoxNo].TIR |= (uint32_t)0x01;

}




/*Abort Ransmit of a msg in mailbox                                  */
/*Parameters: CAN module(1 or 2), mailbox no.(0 or 1 or 2) see MACROS*/  
/*Return: none                                                       */
void CAN_CancelTransmit(CAN_TypeDef* CANx, uint32_t Mailbox)
{
  
	assert(IS_CAN_TX_MAILBOX(Mailbox));
  
	/* abort transmission */
  switch (Mailbox)
  {
    case (CAN_TX_MAILBOX0): CANx->TSR |= CAN_TSR_ABRQ0;
      break;
    case (CAN_TX_MAILBOX1): CANx->TSR |= CAN_TSR_ABRQ1;
      break;
    case (CAN_TX_MAILBOX2): CANx->TSR |= CAN_TSR_ABRQ2;
      break;
    default:
      break;
  }
}




/*Serach Empty mailbox                                                   */
/*Parameters: CAN module(1 or 2)                                         */  
/*Return: mailbox no ...(see Macros), returns 0x0F if none mailbox empty */
uint32_t CAN_Which_MailboxIsEmpty(CAN_TypeDef* CANx)
{

	uint32_t mailbox=0x0F;
	
 if ((CANx->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
  {
    mailbox = CAN_TX_MAILBOX0;
  }
  else if ((CANx->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
  {
    mailbox = CAN_TX_MAILBOX1;
  }
  else if ((CANx->TSR&CAN_TSR_TME2) == CAN_TSR_TME2)
  {
    mailbox = CAN_TX_MAILBOX2;
  }
  else
  {
    mailbox = (uint32_t)0x0F;                         //none empty
  }

return mailbox;
}



/*Receive a message object.                                                      */
/*Parameters: CAN module(1 or 2), FIFO no., msg object pointer                   */  
/*Return: none                                                                   */
void CAN_Receive(CAN_TypeDef* CANx, uint32_t FIFONo, CAN_RxMsgType* RxMsg)
{

  assert(IS_CAN_RX_FIFO(FIFONo));
  
	/* Get the Id */
  RxMsg->IDE = (uint8_t)0x01 & (CANx->sFIFOMailBox[FIFONo].RIR >> 2);     //see this shifting in debug
  if (RxMsg->IDE == CAN_ID_STD)
  {
    RxMsg->StdId = (uint32_t)0x000007FF & (CANx->sFIFOMailBox[FIFONo].RIR >> 21);
  }
  else
  {
    RxMsg->ExtId = (uint32_t)0x1FFFFFFF & (CANx->sFIFOMailBox[FIFONo].RIR >> 3);
  }
  
  RxMsg->RTR = (uint8_t)0x01 & (CANx->sFIFOMailBox[FIFONo].RIR >> 1);
  /* Get the DLC */
  RxMsg->DLC = (uint8_t)0x0F & (CANx->sFIFOMailBox[FIFONo].RDTR);
  /* Get the FMI */
  RxMsg->FMI = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONo].RDTR >> 8);
  
	/* Get the data field */
  RxMsg->Data[0] = (uint8_t)0xFF & CANx->sFIFOMailBox[FIFONo].RDLR;
  RxMsg->Data[1] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONo].RDLR >> 8);
  RxMsg->Data[2] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONo].RDLR >> 16);
  RxMsg->Data[3] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONo].RDLR >> 24);
  RxMsg->Data[4] = (uint8_t)0xFF & CANx->sFIFOMailBox[FIFONo].RDHR;
  RxMsg->Data[5] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONo].RDHR >> 8);
  RxMsg->Data[6] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONo].RDHR >> 16);
  RxMsg->Data[7] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONo].RDHR >> 24);
  
	/* Release the FIFO */
  
	/* Release FIFO0 */
  if (FIFONo == CAN_RX_FIFO0)
  {
    CANx->RF0R |= CAN_RF0R_RFOM0;
  }
  /* Release FIFO1 */
  else 
  {
    CANx->RF1R |= CAN_RF1R_RFOM1;
  }


}



/*Release a FIFO after receiving msg  (on each call it decreases FMP[] value by 1 until FIFO is empty)*/
/*Parameters: CAN module(1 or 2), FIFO No. (see MACROS)                                               */  
/*Return: none                                                                                        */
void CAN_FIFORelease(CAN_TypeDef* CANx, uint32_t FIFONo)
{
 
  assert(IS_CAN_RX_FIFO(FIFONo));
  /* Release FIFO0 */
  if (FIFONo == CAN_RX_FIFO0)
  {
    CANx->RF0R |= CAN_RF0R_RFOM0;
  }
  /* Release FIFO1 */
  else 
  {
    CANx->RF1R |= CAN_RF1R_RFOM1;
  }
}



/*No. of non empty mailboxes in a FIFO                                           */
/*Parameters: CAN module(1 or 2), FIFO No. (see MACROS)                          */  
/*Return: FMP[] bits in RFxR register                                            */
uint8_t CAN_MessagePending(CAN_TypeDef* CANx, uint32_t FIFONo)
{

  uint8_t pending=0;
  
  assert(IS_CAN_RX_FIFO(FIFONo));
	
	/* if FIFO0*/
  if (FIFONo == CAN_RX_FIFO0)
  {
    pending = (uint8_t)(CANx->RF0R&(uint32_t)0x03);
  }
	
	/* if FIFO1*/
  else if (FIFONo == CAN_RX_FIFO1)
  {
    pending = (uint8_t)(CANx->RF1R&(uint32_t)0x03);
  }
  else
  {
    pending = 0;   //no pending
  }
  return pending;

}



/*Enable an interrupt for CAN (this is not the NVIC enabler... see can_test.c)       */
/*Parameters: CAN module(1 or 2), Interrupt Mask (see macros)                        */  
/*Return: none                                                                       */
void CAN_IntMaskEnable(CAN_TypeDef* CANx, uint32_t IntMask)
{

/* Set up interrupt*/
CANx->IER |= IntMask;

}



/*Disable an interrupt for CAN (this is not the NVIC disabling... )*/
/*Parameters: CAN module(1 or 2), Interrupt Mask (see macros)      */  
/*Return: none                                                     */
void CAN_IntMaskDisable(CAN_TypeDef* CANx, uint32_t IntMask)
{

/* Disable interrupt*/
CANx->IER &= ~(IntMask);

}



/*Clear the Interrupt pending in CAN module (not the NVIC clearing ..see can_test.c)*/
/*Parameters: CAN module(1 or 2), Interrupt Mask (see macros)                       */  
/*Return: none                                                                      */
void CAN_IntFlagClear(CAN_TypeDef* CANx, uint32_t IntMask)
{

switch (IntMask)
  {
    case CAN_IT_TME:
      /* Clear CAN_TSR_RQCPx by writing 1 ...see manual*/
      CANx->TSR = CAN_TSR_RQCP0|CAN_TSR_RQCP1|CAN_TSR_RQCP2;  
      break;
   
		//FIFO full interrupt is cleared using FIFO_Release()...
		
		case CAN_IT_FF0:
      /* Clear CAN_RF0R_FULL0 by writing 1...see manual*/  //rc_w1 type
      CANx->RF0R = CAN_RF0R_FULL0; 
      break;
   
		case CAN_IT_FOV0:
      /* Clear CAN_RF0R_FOVR0 by writing 1...see manual*/
      CANx->RF0R = CAN_RF0R_FOVR0; 
      break;
   
		case CAN_IT_FF1:
      /* Clear CAN_RF1R_FULL1 by writing 1...see manual*/
      CANx->RF1R = CAN_RF1R_FULL1;  
      break;
   
		case CAN_IT_FOV1:
      /* Clear CAN_RF1R_FOVR1 by writing 1...see manual*/
      CANx->RF1R = CAN_RF1R_FOVR1; 
      break;
    
		case CAN_IT_WKU:
      /* Clear CAN_MSR_WKUI by writing 1...see manual*/
      CANx->MSR = CAN_MSR_WKUI;  
      break;
   
		case CAN_IT_SLK:
      /* Clear CAN_MSR_SLAKI by writing 1...see manual*/ 
      CANx->MSR = CAN_MSR_SLAKI;   
      break;
   
		case CAN_IT_EWG:
      /* Clear CAN_MSR_ERRI by writing 1...see manual */
      CANx->MSR = CAN_MSR_ERRI;
      break;
    
		case CAN_IT_EPV:
      /* Clear CAN_MSR_ERRI by writing 1...see manual */
      CANx->MSR = CAN_MSR_ERRI; 
      break;
    
		case CAN_IT_BOF:
      /* Clear CAN_MSR_ERRI by writing 1...see manual */ 
      CANx->MSR = CAN_MSR_ERRI; 
       break;
   
		case CAN_IT_LEC:
      /*  Clear LEC bits */
      CANx->ESR = (uint32_t)0x00; 
      /* Clear CAN_MSR_ERRI by writing 1...see manual */
      CANx->MSR = CAN_MSR_ERRI; 
      break;
   
		case CAN_IT_ERR:
      /*Clear LEC bits */
      CANx->ESR = (uint32_t)0x00;         //only LEC field is writable so ,writing 0x00 to entire register has no effect on others
      /* Clear CAN_MSR_ERRI by writing 1...see manual */
      CANx->MSR = CAN_MSR_ERRI; 
       break;
		
		/* BOFF, EPVF and EWGF Flags are cleared by hardware depending on the CAN Bus status*/
       
    default:
       break;
   }

}



/*Get the current value of Status Registers e.g. MSR,ESR etc.                       */
/*can be used in conjunction with Desired Flag..bit-position defined in stm32f44xx.h*/
/*Parameters: CAN module(1 or 2), Register Name(see macros)                         */  
/*Return: Currebt Register Content                                                  */
uint32_t CAN_StatusRegisterGet(CAN_TypeDef* CANx,uint8_t REG_NAME)
{

uint32_t status;	
switch (REG_NAME)
  {
    case CAN_MSR_REG:
      status=CANx->MSR;
      break;
    
		
		case CAN_TSR_REG:
      status=CANx->TSR;
      break;
   
		case CAN_RF0R_REG:
      status=CANx->RF0R;
      break;
   
		case CAN_RF1R_REG:
      status=CANx->RF1R; 
      break;
   
		case CAN_ESR_REG:
      status=CANx->ESR;
      break;
    
    default:
       break;
   }

return status;
	 
}

//