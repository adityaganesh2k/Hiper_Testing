#ifndef _GPIO_DRIVER_H
#define _GPIO_DRIVER_H

/*board specific header file (stm32f446re nucleo) */
#include "stm32f446xx.h"


/****************************************************/

/* GPIO macros for initialization purpose */

/****************************************************/


/* GPIO MODE SETTING VALUES */
#define  GPIO_MODE_IN      ((uint32_t)0x00)   /*!< GPIO Input Mode */
#define  GPIO_MODE_OUT     ((uint32_t)0x01)   /*!< GPIO Output Mode */
#define  GPIO_MODE_AF      ((uint32_t)0x02)   /*!< GPIO Alternate function Mode */
#define  GPIO_MODE_AN      ((uint32_t)0x03)   /*!< GPIO Analog Mode */


/* GPIO PORT OUTPUT TYPE SETTING VALUES */
#define  GPIO_OTYPE_PP     ((uint32_t)0x00)   /*!< push-pull    */
#define  GPIO_OTYPE_OD     ((uint32_t)0x01)   /*!< open-drain   */


/* GPIO PORT SPEED SELECTION SETTING VALUES */
#define  GPIO_LOW_SPEED     ((uint32_t)0x00)  /*!< Low speed    */
#define  GPIO_MEDIUM_SPEED  ((uint32_t)0x01)  /*!< Medium speed */
#define  GPIO_FAST_SPEED    ((uint32_t)0x02)  /*!< Fast speed   */
#define  GPIO_HIGH_SPEED    ((uint32_t)0x03)  /*!< High speed   */


/* GPIO PULL-UP/PULL-DOWN SELECTION VALUES */
#define  GPIO_PUPD_NOPULL   ((uint32_t)0x00)  /*!< No Pull Push  */
#define  GPIO_PUPD_UP       ((uint32_t)0x01)  /*!< Pull-up       */
#define  GPIO_PUPD_DOWN     ((uint32_t)0x02)  /*!< Pull-down     */


/* Macros to ENABLE clock for differnt ports using RCC Register */

#define RCC_GPIOA_CLOCK_ENABLE_CLK()   (RCC->AHB1ENR |= (1<<0))
#define RCC_GPIOB_CLOCK_ENABLE_CLK()   (RCC->AHB1ENR |= (1<<1))
#define RCC_GPIOC_CLOCK_ENABLE_CLK()   (RCC->AHB1ENR |= (1<<2))
#define RCC_GPIOD_CLOCK_ENABLE_CLK()   (RCC->AHB1ENR |= (1<<3))
#define RCC_GPIOE_CLOCK_ENABLE_CLK()   (RCC->AHB1ENR |= (1<<4))
#define RCC_GPIOF_CLOCK_ENABLE_CLK()   (RCC->AHB1ENR |= (1<<5))
#define RCC_GPIOG_CLOCK_ENABLE_CLK()   (RCC->AHB1ENR |= (1<<6))
#define RCC_GPIOH_CLOCK_ENABLE_CLK()   (RCC->AHB1ENR |= (1<<7))

/* Macros to ENABLE clock for differnt ports using RCC Register in LOW POWER Mode */

#define RCC_GPIOA_CLOCK_ENABLE_LP_CLK()   (RCC->AHB1LPENR |= (1<<0))
#define RCC_GPIOB_CLOCK_ENABLE_LP_CLK()   (RCC->AHB1LPENR |= (1<<1))
#define RCC_GPIOC_CLOCK_ENABLE_LP_CLK()   (RCC->AHB1LPENR |= (1<<2))
#define RCC_GPIOD_CLOCK_ENABLE_LP_CLK()   (RCC->AHB1LPENR |= (1<<3))
#define RCC_GPIOE_CLOCK_ENABLE_LP_CLK()   (RCC->AHB1LPENR |= (1<<4))
#define RCC_GPIOF_CLOCK_ENABLE_LP_CLK()   (RCC->AHB1LPENR |= (1<<5))
#define RCC_GPIOG_CLOCK_ENABLE_LP_CLK()   (RCC->AHB1LPENR |= (1<<6))
#define RCC_GPIOH_CLOCK_ENABLE_LP_CLK()   (RCC->AHB1LPENR |= (1<<7))



/*
  GPIO_pins_define GPIO pins define
*/
#define GPIO_PIN_0                 ((uint16_t)0U)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)1U)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)2U)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)3U)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)4U)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)5U)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)6U)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)7U)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)8U)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)9U)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)10U)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)11U)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)12U)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)13U)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)14U)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)15U)  /* Pin 15 selected   */





/* 
   GPIO Bit SET and Bit RESET enumeration 
*/
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET   = 1
}BitAction;



/****************************************************/

/* functions used for assert test  */

/****************************************************/

/* to check valid port */
#define IS_GPIO_ALL_PERIPH(PERIPH) (((PERIPH) == GPIOA) || \
                                    ((PERIPH) == GPIOB) || \
                                    ((PERIPH) == GPIOC) || \
                                    ((PERIPH) == GPIOD) || \
                                    ((PERIPH) == GPIOE) || \
                                    ((PERIPH) == GPIOF) || \
                                    ((PERIPH) == GPIOG) || \
                                    ((PERIPH) == GPIOH))

/* to check valid mode */
#define IS_GPIO_MODE(MODE)   (((MODE) == GPIO_MODE_IN)  || ((MODE) == GPIO_MODE_OUT) || \
                             ((MODE) == GPIO_MODE_AF)|| ((MODE) == GPIO_MODE_AN))

/* to check valid output type */
#define IS_GPIO_OTYPE(OTYPE) (((OTYPE) == GPIO_OTYPE_PP) || ((OTYPE) == GPIO_OTYPE_OD))


/* to check valid speed */
#define IS_GPIO_SPEED(SPEED) (((SPEED) == GPIO_LOW_SPEED) || ((SPEED) == GPIO_MEDIUM_SPEED) || \
                             ((SPEED) == GPIO_FAST_SPEED)||  ((SPEED) == GPIO_HIGH_SPEED)) 

/* to check valid pull push */
#define IS_GPIO_PUPD(PUPD)   (((PUPD) == GPIO_PUPD_NOPULL) || ((PUPD) == GPIO_PUPD_UP) || \
                             ((PUPD) == GPIO_PUPD_DOWN))														

/* to check valid bit action */														
#define IS_GPIO_BIT_ACTION(ACTION) (((ACTION) == GPIO_PIN_RESET) || ((ACTION) == GPIO_PIN_SET))		


/* to check valid pin */														
#define IS_GPIO_PIN(PIN) (((PIN) == GPIO_PIN_0) || \
                              ((PIN) == GPIO_PIN_1) || \
                              ((PIN) == GPIO_PIN_2) || \
                              ((PIN) == GPIO_PIN_3) || \
                              ((PIN) == GPIO_PIN_4) || \
                              ((PIN) == GPIO_PIN_5) || \
                              ((PIN) == GPIO_PIN_6) || \
                              ((PIN) == GPIO_PIN_7) || \
                              ((PIN) == GPIO_PIN_8) || \
                              ((PIN) == GPIO_PIN_9) || \
                              ((PIN) == GPIO_PIN_10) || \
                              ((PIN) == GPIO_PIN_11) || \
                              ((PIN) == GPIO_PIN_12) || \
                              ((PIN) == GPIO_PIN_13) || \
                              ((PIN) == GPIO_PIN_14) || \
                              ((PIN) == GPIO_PIN_15))


/****************************************************/

/* Driver API delaration  */

/****************************************************/

/* Configuration functions *****************************/
void GPIO_Config_Mode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t ModeVal);   /* mode configuration */
void GPIO_Config_PuPd(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t PuPdVal);   /* Pull configuration */
void GPIO_Config_OType(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t OTypeVal); /* output type configuration */
void GPIO_Config_Speed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t SpeedVal); /* speed configuration */
void GPIO_Config_AF(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t GPIO_AF);     /*Alt function configuration function */




/* GPIO Read and Write functions **********************************************/
uint8_t GPIO_Read_Bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t GPIO_Read_Port(GPIO_TypeDef* GPIOx);


void GPIO_Write_Bit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t BitVal);
void GPIO_Write_Port(GPIO_TypeDef* GPIOx, uint16_t PortVal);



/* GPIO interrupt related functions **********************************************/

///* Interrupt Configuration funtion **********************************************/
//void GPIO_Config_Int();

///* Interrupt Enable funtion **********************************************/
//void GPIO_Enable_Int();

///* Interrupt Configuration funtion **********************************************/
//void GPIO_Clear_Int();



#endif
/* end of _GPIO_DRIVER_H */
