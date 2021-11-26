/**
 * @file VCO2_Hatrix_HW.h
 * @author Mustaq Ahmed Althaf Hussain
 * @version 2.1
 * @date 26.11.2021 (Friday)
 * @pre ("Mention any precondition here if needed.")
 * @bug ("Mention any bug here if needed or found.")
 * @warning ("Mention any warning here if needed.")
 * @copyright Copyright (c) 2021. ("Copyrights of the code can be given here.")  
 * @brief The aim is to create a HATRIX library for Modular Electronics Hardware.
 * @details This will contain the all the details of version CO2sensor board hardware.
 */

#ifndef VCO2_Hatrix_HW_H
#define VCO2_Hatrix_HW_H

#define EMPTY_DATA        0 // If the data type is empty.
#define LOCATION_EXTERNAL 1 // The location of pin will be given 1, if it is external pin.
#define LOCATION_INTERNAL 2 // The location of pin will be given 2, if it is internal pin.

typedef struct
{
    uint8_t u8Pin      ;   // Name of the Pin as per the MCU for internal pins, and for external pins as per documentation in the library we used.
    uint8_t u8Location ;   // The location of pin will be given LOCATION_INTERNAL for internal pin and LOCATION_EXTERNAL for external pin.
    uint8_t u8Address  ;   // Address of the pin will be given here.
}structGPIO;

typedef struct
{
    uint8_t  u8Pin                          ; // Name of the Pin as per the MCU for internal pins, and for external pins as per documentation in the library we used.
    uint16_t u16ScaleMaximumInternalVoltage ; // The refernce voltage of ADC pin will be given here.
    uint16_t u16Resolution                  ; // Resolution of the ADC pin will be given here.
    uint8_t  u8Location                     ; // The location of pin will be given LOCATION_INTERNAL for internal pin and LOCATION_EXTERNAL for external pin.
    uint8_t  u8Address                      ; // Address of the pin will be given here.
    float    fInputChannelVoltageDivider    ; // Voltage divider Multiplication factor will be given here.
}structADC;  

typedef struct
{
    uint8_t  u8Pin                          ; // Name of the PWM Pin as per the MCU for internal pins, and for external pins as per documentation in the library we used.
    uint8_t  u8Inhibit                      ; // Name of the Inhibit Pin as per the MCU for internal pins, and for external pins as per documentation in the library we used.
    uint8_t  u8Isense                       ; // Name of the Isense Pin as per the MCU for internal pins, and for external pins as per documentation in the library we used.
    uint8_t  u8Vsense                       ; // Name of the Vsense Pin as per the MCU for internal pins, and for external pins as per documentation in the library we used.
    uint16_t u16ScaleMaximumInternalVoltage ; // The refernce voltage of ADC pin will be given here.
    uint16_t u16Resolution                  ; // Resolution of the ADC pin will be given here.
    uint8_t  u8PWMPinLocation               ; // The location of pin will be given LOCATION_INTERNAL for internal pin and LOCATION_EXTERNAL for external pin.
    uint8_t  u8InhibitLocation              ; // The location of pin will be given LOCATION_INTERNAL for internal pin and LOCATION_EXTERNAL for external pin.
    uint8_t  u8IsenseLocation               ; // The location of pin will be given LOCATION_INTERNAL for internal pin and LOCATION_EXTERNAL for external pin.
    uint8_t  u8VsenseLocation               ; // The location of pin will be given LOCATION_INTERNAL for internal pin and LOCATION_EXTERNAL for external pin.
    uint8_t  u8PWMPinAddress                ; // Address of the pin will be given here.
    uint8_t  u8InhibitAddress               ; // Address of the pin will be given here.
    uint8_t  u8IsenseAddress                ; // Address of the pin will be given here.
    uint8_t  u8VsenseAddress                ; // Address of the pin will be given here.
    float    fIsenseResistanceValue         ; // Resistance value to calculate Isense will be given here.
    float    fVsenseVoltageDividerValue     ; // Voltage divider Multiplication factor will be given here.
    float    fIsenseIdkILIS                 ; // IdkILIS value to calculate Load Current.
}structPWM;

typedef struct
{
    uint8_t u8Address; // There are possible of 8 I2C address {0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67} for MCP9600 IC.
}structTCI;

typedef struct
{
    uint8_t u8NeoPixelLEDPin; // Name of the Neo Pixel LED Pin as per the MCU.
}structNEOPixelLED;

typedef struct
{
    uint8_t u8AddressPins[7]; // Name of the Board Address Pin as per the MCU for internal pins, and for external pins as per documentation in the library we used.
    uint8_t u8AddressPinsLocation[7]; // The location of pin will be given LOCATION_INTERNAL for internal pin and LOCATION_EXTERNAL for external pin.
    uint8_t u8AddressPinsAddress[7]; // Address of the pin will be given here.
}structBoardAddressPins;


/// There are toatally 2 GPIO pins in the version CO2 board. Each of them are given unique names as per the KiCAD schematics using above struct [Since the name in Kicad starts from number. I have added "GPO_" as prefix to the name].
                                /*Pin*/    /*Location*/       /*Address*/
const structGPIO GPO_5V_S_EN_1 = { PB8  , LOCATION_INTERNAL , EMPTY_DATA } ; // GPO_5V_S_EN_1
const structGPIO GPO_5V_S_EN_2 = { PB9  , LOCATION_INTERNAL , EMPTY_DATA } ; // GPO_5V_S_EN_2

/// There are toatally 2 ADC pins in the version 2 board. Each of them are given unique names as per the KiCAD schematics using above struct.
                       /*Pin*//*Vref*//*Res*/   /*Location*/     /*Addr*/   /*InputMultiplication*/
const structADC Aii_1 = { PA0 , 3300 , 1024 , LOCATION_INTERNAL , EMPTY_DATA , ((10000+10000+10000)/10000) } ; // Aii_1
const structADC Aii_2 = { PA1 , 3300 , 1024 , LOCATION_INTERNAL , EMPTY_DATA , ((10000+10000+10000)/10000) } ; // Aii_2

/// Only one Neo Pixel LED via internal pin in the version 2 board. 
                                      /*Pin*/
const structNEOPixelLED NEO_PIXEL = {  PA5  } ; // NEO_PIXEL

/// Only 3 address pins (rotory dip switch) are available for Version-2 Board.
                                              /*Pin*//*Pin*//*Pin*/   /*Pin*/     /*Pin*/     /*Pin*/     /*Pin*/      /*PinLoction*/     /*PinLoction*/      /*PinLoction*/     /*PinLoction*/     /*PinLoction*/     /*PinLoction*/     /*PinLoction*/    /*PinAddress*//*PinAddress*//*PinAddress*//*PinAddress*//*PinAddress*//*PinAddress*//*PinAddress*/
const structBoardAddressPins BoardAddress = {{ PA2 , PA3 , PA4 , EMPTY_DATA, EMPTY_DATA, EMPTY_DATA, EMPTY_DATA}, {LOCATION_INTERNAL, LOCATION_INTERNAL, LOCATION_INTERNAL, LOCATION_INTERNAL, LOCATION_INTERNAL, LOCATION_INTERNAL, LOCATION_INTERNAL}, { EMPTY_DATA ,  EMPTY_DATA ,  EMPTY_DATA ,  EMPTY_DATA ,  EMPTY_DATA ,  EMPTY_DATA ,  EMPTY_DATA  }};

#endif