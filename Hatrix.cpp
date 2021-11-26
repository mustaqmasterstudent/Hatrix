/**
 * @file HATRIX.cpp
 * @author Mustaq Ahmed Althaf Hussain
 * @version 2.1
 * @date 26.11.2021 (Friday)
 * @pre ("Mention any precondition here if needed.")
 * @bug ("Mention any bug here if needed or found.")
 * @warning ("Mention any warning here if needed.")
 * @copyright Copyright (c) 2021. ("Copyrights of the code can be given here.")  
 * @brief The aim is to create a HATRIX library for Modular Electronics Hardware.
 * @details This will contain the Communication protocol between the Boards and storing and working arround with the useful values.
 */

#include "Hatrix.h"

Hatrix::Hatrix()
{
    
}

Hatrix::~Hatrix()
{

}

void Hatrix::fnHatrixBegin()
{
    // We are seting the priorities using HAL library.
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0); // I2C1 is given highest and first priority.
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 1); // I2C1 is given highest and first priority.
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 2); // I2C2 is given second priority.
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 3); // I2C2 is given second priority.
    HAL_NVIC_SetPriority(SPI2_IRQn, 0, 4); // SPI2 is given third priority.
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 5); // USART1 is given fourth priority.
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 6); // USART2 is given fifth priority.
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 7); // TIM2 is given sixth priority.
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 8); // TIM3 is given seventh priority.
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 9); // TIM4 is given eighth priority.
    HAL_NVIC_SetPriority(TIM1_BRK_IRQn, 0, 10); // TIM1 is given ninth priority.
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 11); // TIM1 is given ninth priority.
    HAL_NVIC_SetPriority(TIM1_TRG_COM_IRQn, 0, 12); // TIM1 is given ninth priority.
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 13); // TIM1 is given ninth priority.
    
    #if defined(V3_Hatrix_HW_H) || defined(V2_Hatrix_HW_H) || defined(VCO2_Hatrix_HW_H)
        I2C_1.begin(); // Initialized the I2C1 using begin function.
        I2C_2.begin(); // Initialized the I2C2 using begin function.
        #if defined(V3_Hatrix_HW_H)
            SPI_2.begin(); // Initialized the SPI2 using begin function.
        #endif
        UART_1.begin(115200); // Initialized the USART1 using begin function with the baud rate of 115200.
        #if defined(V3_Hatrix_HW_H) || defined(V2_Hatrix_HW_H)
            UART_2.begin(115200); // Initialized the USART2 using begin function with the baud rate of 115200.
        #endif
    #endif

    pinMode(LED_BUILTIN, OUTPUT); // Sets the default LED pin MODE to "OUTPUT" which is in PC13(LED_BUILTIN).
}
