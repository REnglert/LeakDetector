/**
  *
  * Bruce Englert 
  * u1010546
  * 
  * Mitch Talmadge
  * u1031378
  *
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
void _Error_Handler(char * file, int line);

void SystemClock_Config(void);

void setLED(char led, int set){
  /*
  Options for led are: 'r' 'g', 'b', 'o' 
    For the 4 leds available. 
  Settings: 
    0 - LED off 
    1 - LED on 
    2 - LED toggle 
  */
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

void rgboSet (int r, int g, int b, int o){
  /*
  For each led set to option 0, 1, or 2. 

  Options: 
    0 - LED off 
    1 - LED on 
    2 - LED toggle 
  */
  setLED('o', o); 
  setLED('r', r); 
  setLED('b', b); 
  setLED('g', g); 
}

int main(void)
{
  /* 
    Pinout:
    --------------------
    PA0 - ADC In (Vibration Sensor)
    
    PB10 - UART TX
	  PB11 - UART RX

    PC6 - Red LED Onboard
    PC7 - Blue LED Onboard
    PC8 - Orange LED Onboard
    PC9 - Green LED Onboard

    Manuals:
    --------------------
    - Peripherals: https://www.st.com/content/ccc/resource/technical/document/reference_manual/c2/f8/8a/f2/18/e6/43/96/DM00031936.pdf/files/DM00031936.pdf/jcr:content/translations/en.DM00031936.pdf
    - STM32F072xB Datasheet: https://www.st.com/resource/en/datasheet/DM00090510.pdf
    - Discovery Datasheet: https://www.st.com/content/ccc/resource/technical/document/user_manual/3b/8d/46/57/b7/a9/49/b4/DM00099401.pdf/files/DM00099401.pdf/jcr:content/translations/en.DM00099401.pdf
    - STM Programming Manual: https://www.st.com/content/ccc/resource/technical/document/programming_manual/fc/90/c7/17/a1/44/43/89/DM00051352.pdf/files/DM00051352.pdf/jcr:content/translations/en.DM00051352.pdf

    Setup Instructions: 
    --------------------
    ALARM - int duration in ms for how long the pipe detects water before 
      deciding the current behavior is a leak. 

   */

  HAL_Init();
  SystemClock_Config();

  SetupADC();
  SetupUART();
  SetupLEDs();

  UARTSendString("Started\r\n");

  uint16_t ALARM = 20000; //2 minutes, Max is 60000 1 hour 

  uint16_t  sink = 0, toilet = 0, tub = 0, high = 0;
  uint16_t  count = 0; 
  uint8_t   state = 0; //0 = quiet, 1 = sink, 2 = toilet, 3 = tub, 4 = high/multiple 
  uint16_t  adjusted = 0;
  uint16_t  timeout = 0; 

  char *c;
  while(1){
    uint16_t input = ADC1->DR;
    if(input > 2050){
      adjusted = input - 2050; 
    }
    else {
      continue; 
    }
    asprintf(&c, "%d\r\n", adjusted);
    UARTSendString(c);
    free(c);
    count ++; 
    timeout ++; 

    if(count >= 900){
      //Check in descending order 
      //High usage multiple appliances  
      if(high >= 15){ 
        state = 4; 

        rgboSet(0, 0, 1, 0); 
      }
      //Tub being used 
      else if(tub >= 15){
        state = 3; 

        rgboSet(1, 0, 0, 0);
      }
      //toilet being used 
      else if(toilet >= 15){
        state = 2; 

        rgboSet(0, 0, 0, 1);
      }
      //sink being used 
      else if(sink >= 15){
        state = 1; 

        rgboSet(0, 1, 0, 0); 
      }
      //quiet
      else { 
        state = 0; 

        rgboSet(0, 0, 0, 0);
        timeout = 0; 
      }
      sink = 0; 
      toilet = 0; 
      tub = 0; 
      high = 0; 
      count = 0; 
    }

    if(timeout >= ALARM){
      rgboSet(1, 1, 1, 1);  
      timeout = ALARM+1; 
    }

    //Set threshold values as desired 
    //Potential Multiple appliances running 
    if(adjusted >= 200){
      high ++; 
    }
    //Sink 
    else if(adjusted >= 20 && adjusted < 45){
      sink ++; 
    }
    //Toilet 
    else if(adjusted >= 45 && adjusted < 90){
      toilet++; 
    }
    //Tub
    else if(adjusted >= 90 && adjusted < 200){
      tub++; 
    }

    HAL_Delay(1); 
  }
}

void SetupADC(void)
{
  /* Setup Input Pin PA0 
  ------------------------------------- */
	// Enable the GPIOA clock in the RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// Analog Mode (0b11)
	GPIOA->MODER |= GPIO_MODER_MODER0_Msk;
	
	// Output Type Open-Drain (0b1)
	GPIOA->OTYPER |= GPIO_OTYPER_OT_0;
	
	// Low Speed (0b00)
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR0_Msk;
	
	// Disable Pull Up / Down (0b00)
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0_Msk;
	
  /* Setup ADC1
  ------------------------------------- */
	// Enable ADC1 clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	// 12-bit resolution (0b00)
    ADC1->CFGR1 &= ~ADC_CFGR1_RES_Msk;
    // 8-bit resolution (0b10
//    ADC1->CFGR1 |= ADC_CFGR1_RES_1;
    
	// Continuous conversion mode (0b1)
	ADC1->CFGR1 |= ADC_CFGR1_CONT;
	
	// Disabled hardware trigger detection (0b00)
	ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;
	
	// Select channel 0 for PA0
	ADC1->CHSELR |= ADC_CHSELR_CHSEL0;
	
	// Self-calibrate 
	ADC1->CR |= ADC_CR_ADCAL;
	// Wait for calibration to complete
	while(ADC1->CR & ADC_CR_ADCAL);
	
	// Enable
	ADC1->ISR |= ADC_ISR_ADRDY;
	ADC1->CR |= ADC_CR_ADEN;
	// Wait for ready
	while(!(ADC1->ISR & ADC_ISR_ADRDY));
	
	// Start
	ADC1->CR |= ADC_CR_ADSTART;
}

void SetupLEDs(void)
{
	// Enable the GPIOC clock in the RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// General Purpose Output (0b01)
	GPIOC->MODER &= ~GPIO_MODER_MODER6_1; // PC6 - Red LED
	GPIOC->MODER |= GPIO_MODER_MODER6_0;
	
	GPIOC->MODER &= ~GPIO_MODER_MODER7_1; // PC7 - Blue LED
	GPIOC->MODER |= GPIO_MODER_MODER7_0;
	
	GPIOC->MODER &= ~GPIO_MODER_MODER8_1; // PC8 - Orange LED
	GPIOC->MODER |= GPIO_MODER_MODER8_0;
	
	GPIOC->MODER &= ~GPIO_MODER_MODER9_1; // PC9 - Green LED
	GPIOC->MODER |= GPIO_MODER_MODER9_0;
	
	// Output Type Push-Pull (0b0)
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_6; // PC6
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_7; // PC7
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_8; // PC8
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_9; // PC9
	
	// Low Speed (0b00)
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR6_Msk; // PC6
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR7_Msk; // PC7
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR8_Msk; // PC8
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR9_Msk; // PC9
	
	// Disable Pull Up/Down (0b00)
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR6_Msk; // PC6
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR7_Msk; // PC7
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR8_Msk; // PC8
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR9_Msk; // PC9	
}

void SetupUART(void) {
	// Enable GPIOB Clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	// TX: PB10
	// RX: PB11
	
	// Alternate Function Output (0b10)
	GPIOB->MODER &= ~GPIO_MODER_MODER10_0;
	GPIOB->MODER |= GPIO_MODER_MODER10_1;
	GPIOB->MODER &= ~GPIO_MODER_MODER11_0;
	GPIOB->MODER |= GPIO_MODER_MODER11_1;
	
	// Use USART3 (AF4)
	GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL10_Pos);
	GPIOB->AFR[1] |= (4 << GPIO_AFRH_AFSEL11_Pos);
	
	// Enable USART3 Clock
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// Set USART3 baud rate to 115200 bps
	// USART_BRR = fCLK/Baud
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;

	// Enable transmitter and receiver
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;
	
	// Final enable of USART3
	USART3->CR1 |= USART_CR1_UE;
}

char UARTRead(void) {
	while(!(USART3->ISR & USART_ISR_RXNE));
	return USART3->RDR;
}

void UARTSendString(char* str) {
	uint8_t i = 0;
	char c;
	while((c = str[i]) != 0) {
		UARTSend(c);
		i++;
	}
}

void UARTSend(char c) {
	while(!(USART3->ISR & USART_ISR_TXE));
	USART3->TDR = c;
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
