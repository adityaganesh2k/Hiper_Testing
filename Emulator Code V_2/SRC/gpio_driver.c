#include <stdint.h>
#include <assert.h>
#include "gpio_driver.h"




/****************************************************/

/* Driver API definition  */

/****************************************************/

/* Initialization and de-initialization functions *****************************/

void GPIO_Config_Mode (GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, uint32_t ModeVal)
{

assert(IS_GPIO_ALL_PERIPH (GPIOx));    //assert_param() has some dependencies, so temporarily using assert()
assert(IS_GPIO_PIN (GPIO_Pin));
assert(IS_GPIO_MODE (ModeVal));

GPIOx->MODER &= ~((0x00000003) << (2 * GPIO_Pin));      /*clear the required position */  //(cause RESET values are not always "0")
GPIOx->MODER |= ((ModeVal) << (2 * GPIO_Pin));	       /* each pin need 2 bit in MODER register */



}




void GPIO_Config_PuPd (GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, uint32_t PuPdVal)
{

assert (IS_GPIO_ALL_PERIPH (GPIOx));
assert (IS_GPIO_PIN (GPIO_Pin));
assert (IS_GPIO_PUPD (PuPdVal));

GPIOx->PUPDR &=	~((0x00000003) << (2 * GPIO_Pin));      /*clear the required position */  //(cause RESET values are not always "0")
GPIOx->PUPDR |= ((PuPdVal) << (2 * GPIO_Pin));	        /* each pin need 2 bit in PUPDR register */

}




void GPIO_Config_OType (GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, uint32_t OTypeVal)
{

assert (IS_GPIO_ALL_PERIPH (GPIOx));
assert (IS_GPIO_PIN (GPIO_Pin));
assert (IS_GPIO_OTYPE (OTypeVal));


GPIOx->OTYPER |= ((OTypeVal) << (GPIO_Pin));	/* each pin need 1 bit OTYPER register */

}




void GPIO_Config_Speed (GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, uint32_t SpeedVal)
{

assert (IS_GPIO_ALL_PERIPH (GPIOx));
assert (IS_GPIO_PIN (GPIO_Pin));
assert (IS_GPIO_SPEED (SpeedVal));
	
GPIOx->OSPEEDR &=	~((0x00000003) << (2 * GPIO_Pin));      /*clear the required position */  //(cause RESET values are not always "0")
GPIOx->OSPEEDR |= ((SpeedVal) << (2 * GPIO_Pin));	/* each pin need 2 bit in OSPEEDR register */

}




  /* GPIO Alternate functions configuration function *************************** */

void GPIO_Config_AF (GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, uint32_t GPIO_AF)
{


if (GPIO_Pin <= 7)

    {

     GPIOx->AFR[0] |= (GPIO_AF << (GPIO_Pin * 4));	/* each pin need 4 bit in AFR register */

     }


 else

    {

    GPIOx->AFR[1] |= (GPIO_AF << ((GPIO_Pin % 8) * 4));	/* each pin need 4 bit in AFR register */

    }


}





/* GPIO Read and Write functions **********************************************/
  uint8_t GPIO_Read_Bit (GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin)
{



  /* Check the parameters */
  assert(IS_GPIO_ALL_PERIPH(GPIOx));
  assert(IS_GPIO_PIN(GPIO_Pin));

	uint8_t bitstatus ;
	/*Read from the IDR register then right shift by the value of the pin no. to get
	the input status value of the pin to LSB */

  bitstatus=((GPIOx->IDR >> GPIO_Pin) & 0x00000001);

  return bitstatus;


}



uint16_t GPIO_Read_Port (GPIO_TypeDef * GPIOx)
{

 /* Check the parameters */
  assert(IS_GPIO_ALL_PERIPH(GPIOx));

  return ((uint16_t)GPIOx->IDR);

}



void GPIO_Write_Bit (GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin, uint8_t BitVal)
{

 /* Check the parameters */
  assert(IS_GPIO_ALL_PERIPH(GPIOx));
  assert(IS_GPIO_PIN(GPIO_Pin));


	if(BitVal)
	{
	GPIOx->ODR |= (1<<GPIO_Pin);
	}
  else
	{
	GPIOx->ODR &= ~(1<<GPIO_Pin);
	}
	


}



void GPIO_Write_Port (GPIO_TypeDef * GPIOx, uint16_t PortVal)
{

/* Check the parameters */
  assert(IS_GPIO_ALL_PERIPH(GPIOx));

  GPIOx->ODR = PortVal;

}


