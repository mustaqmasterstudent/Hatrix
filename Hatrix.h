/**
 * @file HATRIX.h
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

#ifndef Hatrix_H
#define Hatrix_H

#include "Hatrix_HW.h" // Adding this to utilize the functionalities of all the functional blocks of the Hardware.
                       // **Note: Do not forget to navigate to this file and include the correct header file for the respective Heardware**.
                       // /*******IMPORTANT**************IMPORTANT**************IMPORTANT***************IMPORTANT**********IMPORTANT*********/

static uint16_t u16PagesConcept[50][15]; // The memory is allocated to store the values from different functional blocks.

/**
 * @brief Hatrix Class.
 * Manly for communication portocol via UART and RS485.
 * This should be implemented after documenting everything.
 */
class Hatrix
{
    public : 
        /**
         * @brief Construct a new Hatrix object.
         *  
         */
        Hatrix(); 
        /**
         * @brief Destroy the Hatrix object.
         * 
         */
        ~Hatrix();

        /**
         * @brief This function is used Set the preorities. And begin all the necessary and basic functions. 
         * The preority is given in this order: I2C1, I2C2, SPI2, USART1, USART2, TIM2, TIM3, TIM4, and TIM1. 
         * I2C1, I2C2, SPI2, USART1, and USART2 has begin with begin function. 
         * PC13 (LED_BUILTIN) is given as OUTPUT using pinMode. 
         */
        void fnHatrixBegin();
    
    private : 



};

#endif