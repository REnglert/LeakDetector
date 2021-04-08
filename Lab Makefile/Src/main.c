/**
  *
  * Bruce Englert
  * u1010546
  *
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
void _Error_Handler(char * file, int line);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int initPins(void){
    //PC6 RED
    GPIOC->MODER |= 1 << 12; //set bit 12
    //PC7 BLUE
    GPIOC->MODER |= 1 << 14; //set bit 14
    //PC8 ORANGE
    GPIOC->MODER |= 1 << 16; //set bit 16
    //PC9 GREEN
    GPIOC->MODER |= 1 << 18; //set bit 18

    return 0;
}


void setLED(char led, int set){
    int l = 6;
    switch (led){
        case 'r':
            l = 6;
            break;
        case 'b':
            l = 7;
            break;
        case 'o':
            l = 8;
            break;
        case 'g':
            l = 9;
            break;
        default:
            break;
    }
    
    switch (set){
        case 0:
            GPIOC->ODR &= ~(1<<l);
            break;
        case 1:
            GPIOC->ODR |= 1<<l;
            break ;
        case 2:
            GPIOC->ODR ^= 1<<l;
            break;
        default:
            break;
    }
}

void tx_char(char c){
    while(((USART3->ISR) & (1<<7)) == 0);
    USART3->TDR = c;
}

void tx_word(char c[]){
    int i = 0;
    while(c[i] != '\0'){
        tx_char(c[i]);
        i++;
    }
}


/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    RCC->APB2ENR    |= RCC_APB2ENR_SYSCFGCOMPEN;

    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    //TX setup
    //PB11 TX: LQPR64 30 : AF4 0100 [15, 14, 13, 12]
    GPIOB-> MODER |= GPIO_MODER_MODER11_1; //Bit 1<<23
    GPIOB->AFR[1] |= 1<<14;
    //Set the Baud rate for communication to be 115200 bits/second.
    // USART_BRR = f_CLK / baud rate = 8000000 / 115200 = 69
    USART3->BRR = 69 ; //
    //The USART starts with portions of the peripheral disabled for low-power use. You will need to enable the transmitter and receiver hardware.
    USART3 -> CR1 |= USART_CR1_RXNEIE | USART_CR1_TE;
    USART3 ->CR1 |= USART_CR1_UE;
    
    char *c;
    asprintf(&c, "Example output\r\n");
    tx_word(c);
    
    //1. Initialize the LED pins to output.
    initPins();
    //2. Select a GPIO pin to use as the ADC input.
    //ADC_IN0 PA0
    //LQFP64: 14
    GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER0_0;
    //3. Enable the ADC1 in the RCC peripheral.
    //Above
    //4. Configure the ADC to 8-bit resolution, continuous conversion mode, hardware triggers disabled (software trigger only).
    ADC1->CFGR1 |= ADC_CFGR1_RES_1 | ADC_CFGR1_CONT;
    //5. Select/enable the input pin’s channel for ADC conversion.
    ADC1->CHSELR = ADC_CHSELR_CHSEL0;
    //6. Perform a self-calibration, enable, and start the ADC.
    ADC1->CR |= ADC_CR_ADCAL; //elf calihbration
    while(ADC1->CR & ADC_CR_ADCAL) {}
    ADC1->CR |= ADC_CR_ADEN; //enable
    while(ADC1->ISR & ADC_ISR_ADRDY) {}
    ADC1->CR |= ADC_CR_ADSTART; //start
    //7. In the main application loop, read the ADC data register and turn on/off LEDs depending on the value.
    

    uint8_t index = 0;
    while(1){
        input = ADC1->DR;
        
        if(input > 180){
            setLED('b', 1);
        }
        else if(input > 120){
            setLED('g', 1);
        }
        else if(input > 60){
            setLED('o', 1);
        }
        else if(input <= 60 && input > 40){
            setLED('r', 1);
        }
        
        HAL_Delay(1);
    }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
