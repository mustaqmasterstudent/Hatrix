/**
 * @file HATRIX.ino
 * @author Mustaq Ahmed Althaf Hussain
 * @version 2.1
 * @date 26.11.2021 (Friday)
 * @pre ("Mention any precondition here if needed.")
 * @bug ("Mention any bug here if needed or found.")
 * @warning ("Mention any warning here if needed.")
 * @copyright Copyright (c) 2021. ("Copyrights of the code can be given here.")  
 * @brief Example by using Hatrix library.
 * @details Simple Example by using simple functions from all the possible class for version 3 board.
 */

#include"Hatrix.h" // Before starting anything. Navigate to this header file and then navigate to the Hatrix_HW.h header file. Then add the correct header file of the header. To make this work correctly.
                   // /*******IMPORTANT**************IMPORTANT**************IMPORTANT***************IMPORTANT**********IMPORTANT********IMPORTANT*********IMPORTANT**********IMPORTANT********IMPORTANT*****/

#define JENS_DEBUG

#define HATRIX_CLASS

#define GPIO_CLASS_BOARD_ADDRESS 

#define NEOPIXEL_CLASS

#define GPIO_CLASS
#define ADC_CLASS
#define PWM_CLASS
#define TCI_CLASS
#define CO2_CLASS

#ifdef HATRIX_CLASS

    Hatrix HATRIX ;

#endif

#ifdef GPIO_CLASS_BOARD_ADDRESS

    GPIO BoardAddressGPIO ; // Board Address

#endif

#ifdef NEOPIXEL_CLASS

    NEOPIXEL Neo = NEOPIXEL(NEO_PIXEL) ;

#endif

#ifdef GPIO_CLASS

    GPIO GPIO0 = GPIO(IND_GPO0) ; // GPO_CH1    
    GPIO GPIO1 = GPIO(IND_GPO1) ; // GPO_CH2   
    GPIO GPIO2 = GPIO(IND_GPO2) ; // GPO_CH3 
    GPIO GPIO3 = GPIO(IND_GPO3) ; // GPO_CH4  
    GPIO GPIO4 = GPIO(IND_GPO4) ; // GPO_CH5   
    GPIO GPIO5 = GPIO(IND_GPO5) ; // GPO_CH6   

#endif

#ifdef ADC_CLASS

    ADC ADC_0 = ADC(AIN0) ; // AIN0 
    ADC ADC_1 = ADC(AIN1) ; // AIN1 
    ADC ADC_2 = ADC(AIN2) ; // AIN2 
    ADC ADC_3 = ADC(AIN3) ; // AIN3 

#endif    

#ifdef PWM_CLASS

    PWM * PWM_0 = new PWM(PWM0) ; // PWM0 // We need to initialitize this constructor with pointer because we use timers.
    PWM * PWM_1 = new PWM(PWM1) ; // PWM1 // We need to initialitize this constructor with pointer because we use timers.
    PWM * PWM_2 = new PWM(PWM2) ; // PWM2 // We need to initialitize this constructor with pointer because we use timers.
    PWM * PWM_3 = new PWM(PWM3) ; // PWM3 // We need to initialitize this constructor with pointer because we use timers.
    PWM * PWM_4 = new PWM(PWM4) ; // PWM4 // We need to initialitize this constructor with pointer because we use timers.
    PWM * PWM_5 = new PWM(PWM5) ; // PWM5 // We need to initialitize this constructor with pointer because we use timers.

#endif  

#ifdef TCI_CLASS

    TCI TCI1 = TCI(J29) ; // J29       
    TCI TCI2 = TCI(J25) ; // J25       
    TCI TCI3 = TCI(J28) ; // J28       
    TCI TCI4 = TCI(J27) ; // J27       
    TCI TCI5 = TCI(J26) ; // J26      

#endif  

#ifdef CO2_CLASS

    CO2 CO21 = CO2(0x69, K30_CO2_TYPE) ; // Co2    
    CO2 CO22 = CO2(0x6B, K33_CO2_TYPE) ; // Co2    

#endif  

#ifdef GPIO_CLASS_BOARD_ADDRESS

    uint8_t BoardAddressValue ; // 

#endif

#ifdef GPIO_CLASS

    uint8_t GPIO_IND_GPO0 ; //  
    uint8_t GPIO_IND_GPO1 ; //  
    uint8_t GPIO_IND_GPO2 ; //  
    uint8_t GPIO_IND_GPO3 ; //  
    uint8_t GPIO_IND_GPO4 ; //  
    uint8_t GPIO_IND_GPO5 ; //  

#endif

#ifdef ADC_CLASS

    float ADC_AIN0 ; //  
    float ADC_AIN1 ; //  
    float ADC_AIN2 ; //  
    float ADC_AIN3 ; //  

    #ifdef JENS_DEBUG

        uint16_t u16ADCADSConversionRate ; // 

    #endif

#endif 

#ifdef PWM_CLASS

    float PWM0_CurrentLoad, PWM0_VoltageLoad, PWM0_PowerLoad ; //  
    float PWM1_CurrentLoad, PWM1_VoltageLoad, PWM1_PowerLoad ; //  
    float PWM2_CurrentLoad, PWM2_VoltageLoad, PWM2_PowerLoad ; //  
    float PWM3_CurrentLoad, PWM3_VoltageLoad, PWM3_PowerLoad ; //  
    float PWM4_CurrentLoad, PWM4_VoltageLoad, PWM4_PowerLoad ; //  
    float PWM5_CurrentLoad, PWM5_VoltageLoad, PWM5_PowerLoad ; //  

#endif  

#ifdef TCI_CLASS

    float TCI_J29_Temperature, TCI_J29_AmbientTemperature, TCI_J29_DeltaTemperature ; //   
    float TCI_J25_Temperature, TCI_J25_AmbientTemperature, TCI_J25_DeltaTemperature ; //   
    float TCI_J28_Temperature, TCI_J28_AmbientTemperature, TCI_J28_DeltaTemperature ; //   
    float TCI_J27_Temperature, TCI_J27_AmbientTemperature, TCI_J27_DeltaTemperature ; //   
    float TCI_J26_Temperature, TCI_J26_AmbientTemperature, TCI_J26_DeltaTemperature ; //  

#endif  

#ifdef CO2_CLASS

    uint16_t Co2_Co21; // 
    uint16_t Co2_Co22; // 

#endif  

#ifdef JENS_DEBUG

    String         InputStringSerial     =   ""   ;
    uint8_t        InputCharSerialRead            ;
    uint8_t        Count_of_Comas                 ;
    unsigned long  Current_Time                   ;
    uint32_t       Loop_Number                    ;

    #ifdef PWM_CLASS

        uint8_t  PWM0_Dutycycle              ;
        uint8_t  PWM1_Dutycycle              ;
        uint8_t  PWM2_Dutycycle              ;
        uint8_t  PWM3_Dutycycle              ;
        uint8_t  PWM4_Dutycycle              ;
        uint8_t  PWM5_Dutycycle              ;
        uint32_t PWM0_PWM1_Frequency         ;
        uint32_t PWM2_PWM3_Frequency         ;
        uint32_t PWM4_PWM5_Frequency         ;
        uint8_t  PWM0_InhibitPinMode         ;
        uint8_t  PWM1_InhibitPinMode         ;
        uint8_t  PWM2_InhibitPinMode         ;
        uint8_t  PWM3_InhibitPinMode         ;
        uint8_t  PWM4_InhibitPinMode         ;
        uint8_t  PWM5_InhibitPinMode         ;
        uint16_t PWM0_PWM1_ADSConversionRate ;
        uint16_t PWM2_PWM3_ADSConversionRate ;
        uint16_t PWM4_PWM5_ADSConversionRate ;

    #endif  

#endif

void setup()
{
    #ifdef JENS_DEBUG

        Serial.begin(115200);
        while(!Serial);

    #endif

    #ifdef HATRIX_CLASS

        HATRIX.fnHatrixBegin();  

    #endif

    #ifdef GPIO_CLASS_BOARD_ADDRESS

        BoardAddressGPIO.fnGPIOHatrixBoardAddressInit();

    #endif

    #ifdef NEOPIXEL_CLASS

        Neo.fnNeoLEDBegin();
        Neo.fnNeoLEDColorSetAllLED(Neo.White);

    #endif

    #ifdef GPIO_CLASS

        GPIO0.fnGPIOPinModePinValue(OUTPUT, LOW);
        GPIO1.fnGPIOPinModePinValue(OUTPUT, LOW);
        GPIO2.fnGPIOPinModePinValue(OUTPUT, LOW);
        GPIO3.fnGPIOPinModePinValue(OUTPUT, LOW);
        GPIO4.fnGPIOPinModePinValue(OUTPUT, LOW);
        GPIO5.fnGPIOPinModePinValue(OUTPUT, LOW);

    #endif

    #ifdef ADC_CLASS

        ADC_0.fnADCBegin();
        ADC_1.fnADCBegin();
        ADC_2.fnADCBegin();
        ADC_3.fnADCBegin();

        #ifdef JENS_DEBUG

            u16ADCADSConversionRate = 128;

            ADC_0.fnADCConversionRateUpdate(u16ADCADSConversionRate);
            ADC_1.fnADCConversionRateUpdate(u16ADCADSConversionRate);
            ADC_2.fnADCConversionRateUpdate(u16ADCADSConversionRate);
            ADC_3.fnADCConversionRateUpdate(u16ADCADSConversionRate);

        #endif

    #endif 

    #ifdef PWM_CLASS

        #ifdef JENS_DEBUG

            PWM0_InhibitPinMode = 0;
            PWM1_InhibitPinMode = 0;
            PWM2_InhibitPinMode = 0;
            PWM3_InhibitPinMode = 0;
            PWM4_InhibitPinMode = 0;
            PWM5_InhibitPinMode = 0;
            PWM0_Dutycycle = 0;
            PWM1_Dutycycle = 0;
            PWM2_Dutycycle = 0;
            PWM3_Dutycycle = 0;
            PWM4_Dutycycle = 0;
            PWM5_Dutycycle = 0;
            PWM0_PWM1_Frequency = 200;
            PWM2_PWM3_Frequency = 200;
            PWM4_PWM5_Frequency = 200;
            PWM0_PWM1_ADSConversionRate = 128;
            PWM2_PWM3_ADSConversionRate = 128;
            PWM4_PWM5_ADSConversionRate = 128;

        #endif  

        PWM_0->fnPWMBeginStart(PWM0_PWM1_Frequency, PWM0_Dutycycle);
        PWM_1->fnPWMBeginStart(PWM0_PWM1_Frequency, PWM1_Dutycycle);
        PWM_2->fnPWMBeginStart(PWM2_PWM3_Frequency, PWM2_Dutycycle);
        PWM_3->fnPWMBeginStart(PWM2_PWM3_Frequency, PWM3_Dutycycle);
        PWM_4->fnPWMBeginStart(PWM4_PWM5_Frequency, PWM4_Dutycycle);
        PWM_5->fnPWMBeginStart(PWM4_PWM5_Frequency, PWM5_Dutycycle);

        PWM_0->fnPWMSleepMode();
        PWM_1->fnPWMSleepMode();
        PWM_2->fnPWMSleepMode();
        PWM_3->fnPWMSleepMode();
        PWM_4->fnPWMSleepMode();
        PWM_5->fnPWMSleepMode();

        #ifdef JENS_DEBUG

            PWM_0->fnPWMConversionRateUpdate(PWM0_PWM1_ADSConversionRate);
            PWM_1->fnPWMConversionRateUpdate(PWM0_PWM1_ADSConversionRate);
            PWM_2->fnPWMConversionRateUpdate(PWM2_PWM3_ADSConversionRate);
            PWM_3->fnPWMConversionRateUpdate(PWM2_PWM3_ADSConversionRate);
            PWM_4->fnPWMConversionRateUpdate(PWM4_PWM5_ADSConversionRate);
            PWM_5->fnPWMConversionRateUpdate(PWM4_PWM5_ADSConversionRate);


        #endif  

    #endif 

    #ifdef TCI_CLASS

        TCI1.fnTCIBegin();
        TCI2.fnTCIBegin();
        TCI3.fnTCIBegin();
        TCI4.fnTCIBegin();
        TCI5.fnTCIBegin();

    #endif 

    #ifdef CO2_CLASS

        //CO21.fnCO2Begin(); // /*This function is needed only if we use SCD30 Sensor.*/

    #endif 
}

void loop()
{ 
    #ifdef JENS_DEBUG

        Current_Time = micros();  
        Serial.print(Current_Time); Serial.print(",");   
        Loop_Number = Loop_Number + 1;
        Serial.print(Loop_Number); Serial.print(",");   

    #endif

    #ifdef GPIO_CLASS_BOARD_ADDRESS

        BoardAddressValue = BoardAddressGPIO.fnGPIOHatrixBoardAddress(); 
        #ifdef JENS_DEBUG

            Serial.print(BoardAddressValue); Serial.print(",");

        #endif

    #endif

    #ifdef GPIO_CLASS

        GPIO_IND_GPO0 = GPIO0.fnGPIOPinStatus();
        #ifdef JENS_DEBUG

            Serial.print(GPIO_IND_GPO0); Serial.print(",");

        #endif
        GPIO_IND_GPO1 = GPIO1.fnGPIOPinStatus();
        #ifdef JENS_DEBUG

            Serial.print(GPIO_IND_GPO1); Serial.print(",");

        #endif
        GPIO_IND_GPO2 = GPIO2.fnGPIOPinStatus();
        #ifdef JENS_DEBUG

            Serial.print(GPIO_IND_GPO2); Serial.print(",");

        #endif
        GPIO_IND_GPO3 = GPIO3.fnGPIOPinStatus();
        #ifdef JENS_DEBUG

            Serial.print(GPIO_IND_GPO3); Serial.print(",");

        #endif
        GPIO_IND_GPO4 = GPIO4.fnGPIOPinStatus();
        #ifdef JENS_DEBUG

            Serial.print(GPIO_IND_GPO4); Serial.print(",");

        #endif
        GPIO_IND_GPO5 = GPIO5.fnGPIOPinStatus();
        #ifdef JENS_DEBUG

            Serial.print(GPIO_IND_GPO5); Serial.print(",");

        #endif

    #endif

    #ifdef ADC_CLASS

        ADC_AIN0 = ADC_0.fnADCInputChannelVoltageValue();
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(ADC_AIN0); Serial.print(",");

        #endif
        ADC_AIN1 = ADC_1.fnADCInputChannelVoltageValue();
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(ADC_AIN1); Serial.print(",");

        #endif
        ADC_AIN2 = ADC_2.fnADCInputChannelVoltageValue();
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(ADC_AIN2); Serial.print(",");

        #endif
        ADC_AIN3 = ADC_3.fnADCInputChannelVoltageValue();
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(ADC_AIN3); Serial.print(",");

        #endif

        #ifdef JENS_DEBUG

            Serial.print(u16ADCADSConversionRate); Serial.print(",");

        #endif

    #endif 

    #ifdef PWM_CLASS

        PWM0_CurrentLoad = PWM_0->fnPWMLoadCurrent(); 
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(PWM0_CurrentLoad); Serial.print(",");

        #endif
        PWM0_VoltageLoad = PWM_0->fnPWMLoadVoltage(); 
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(PWM0_VoltageLoad); Serial.print(",");

        #endif
        PWM0_PowerLoad = PWM_0->fnPWMLoadPower(); 
        #ifdef JENS_DEBUG

            Serial.print(PWM0_PowerLoad); Serial.print(",");
            Serial.print(PWM0_InhibitPinMode); Serial.print(","); Serial.print(PWM0_Dutycycle); Serial.print(","); Serial.print(PWM0_PWM1_Frequency); Serial.print(","); Serial.print(PWM0_PWM1_ADSConversionRate); Serial.print(",");

        #endif

        PWM1_CurrentLoad = PWM_1->fnPWMLoadCurrent(); 
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(PWM1_CurrentLoad); Serial.print(",");

        #endif
        PWM1_VoltageLoad = PWM_1->fnPWMLoadVoltage(); 
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(PWM1_VoltageLoad); Serial.print(",");

        #endif
        PWM1_PowerLoad = PWM_1->fnPWMLoadPower();  
        #ifdef JENS_DEBUG

            Serial.print(PWM1_PowerLoad); Serial.print(",");
            Serial.print(PWM1_InhibitPinMode); Serial.print(","); Serial.print(PWM1_Dutycycle); Serial.print(","); Serial.print(PWM0_PWM1_Frequency); Serial.print(","); Serial.print(PWM0_PWM1_ADSConversionRate); Serial.print(",");

        #endif

        PWM2_CurrentLoad = PWM_2->fnPWMLoadCurrent(); 
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(PWM2_CurrentLoad); Serial.print(",");

        #endif
        PWM2_VoltageLoad = PWM_2->fnPWMLoadVoltage(); 
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(PWM2_VoltageLoad); Serial.print(",");

        #endif
        PWM2_PowerLoad = PWM_2->fnPWMLoadPower();
        #ifdef JENS_DEBUG

            Serial.print(PWM2_PowerLoad); Serial.print(",");
            Serial.print(PWM2_InhibitPinMode); Serial.print(","); Serial.print(PWM2_Dutycycle); Serial.print(","); Serial.print(PWM2_PWM3_Frequency); Serial.print(","); Serial.print(PWM2_PWM3_ADSConversionRate); Serial.print(",");

        #endif 

        PWM3_CurrentLoad = PWM_3->fnPWMLoadCurrent(); 
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(PWM3_CurrentLoad); Serial.print(",");

        #endif
        PWM3_VoltageLoad = PWM_3->fnPWMLoadVoltage(); 
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(PWM3_VoltageLoad); Serial.print(",");

        #endif
        PWM3_PowerLoad = PWM_3->fnPWMLoadPower();
        #ifdef JENS_DEBUG

            Serial.print(PWM3_PowerLoad); Serial.print(",");
            Serial.print(PWM3_InhibitPinMode); Serial.print(","); Serial.print(PWM3_Dutycycle); Serial.print(","); Serial.print(PWM2_PWM3_Frequency); Serial.print(","); Serial.print(PWM2_PWM3_ADSConversionRate); Serial.print(",");

        #endif

        PWM4_CurrentLoad = PWM_4->fnPWMLoadCurrent(); 
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(PWM4_CurrentLoad); Serial.print(",");

        #endif
        PWM4_VoltageLoad = PWM_4->fnPWMLoadVoltage(); 
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(PWM4_VoltageLoad); Serial.print(",");

        #endif
        PWM4_PowerLoad = PWM_4->fnPWMLoadPower();
        #ifdef JENS_DEBUG

            Serial.print(PWM4_PowerLoad); Serial.print(",");
            Serial.print(PWM4_InhibitPinMode); Serial.print(","); Serial.print(PWM4_Dutycycle); Serial.print(","); Serial.print(PWM4_PWM5_Frequency); Serial.print(","); Serial.print(PWM4_PWM5_ADSConversionRate); Serial.print(",");

        #endif

        PWM5_CurrentLoad = PWM_5->fnPWMLoadCurrent(); 
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(PWM5_CurrentLoad); Serial.print(",");

        #endif
        PWM5_VoltageLoad = PWM_5->fnPWMLoadVoltage(); 
        #ifdef JENS_DEBUG

            Serial.print(","); Serial.print(PWM5_VoltageLoad); Serial.print(",");

        #endif
        PWM5_PowerLoad = PWM_5->fnPWMLoadPower();
        #ifdef JENS_DEBUG

            Serial.print(PWM5_PowerLoad); Serial.print(",");
            Serial.print(PWM5_InhibitPinMode); Serial.print(","); Serial.print(PWM5_Dutycycle); Serial.print(","); Serial.print(PWM4_PWM5_Frequency); Serial.print(","); Serial.print(PWM4_PWM5_ADSConversionRate); Serial.print(",");

        #endif

    #endif  

    #ifdef TCI_CLASS

        TCI_J29_Temperature = TCI1.fnTCIThermocoupleTemperature(); 
        #ifdef JENS_DEBUG

            Serial.print(TCI_J29_Temperature); Serial.print(",");

        #endif
        TCI_J29_AmbientTemperature = TCI1.fnTCIAmbientTemperature(); 
        #ifdef JENS_DEBUG

            Serial.print(TCI_J29_AmbientTemperature); Serial.print(",");

        #endif
        TCI_J29_DeltaTemperature = TCI1.fnTCITemperatureDelta();
        #ifdef JENS_DEBUG

            Serial.print(TCI_J29_DeltaTemperature); Serial.print(",");

        #endif

        TCI_J25_Temperature = TCI2.fnTCIThermocoupleTemperature(); 
        #ifdef JENS_DEBUG

            Serial.print(TCI_J25_Temperature); Serial.print(",");

        #endif
        TCI_J25_AmbientTemperature = TCI2.fnTCIAmbientTemperature(); 
        #ifdef JENS_DEBUG

            Serial.print(TCI_J25_AmbientTemperature); Serial.print(",");

        #endif
        TCI_J25_DeltaTemperature = TCI2.fnTCITemperatureDelta();
        #ifdef JENS_DEBUG

            Serial.print(TCI_J25_DeltaTemperature); Serial.print(",");

        #endif

        TCI_J28_Temperature = TCI3.fnTCIThermocoupleTemperature(); 
        #ifdef JENS_DEBUG

            Serial.print(TCI_J28_Temperature); Serial.print(",");

        #endif
        TCI_J28_AmbientTemperature = TCI3.fnTCIAmbientTemperature(); 
        #ifdef JENS_DEBUG

            Serial.print(TCI_J28_AmbientTemperature); Serial.print(",");

        #endif
        TCI_J28_DeltaTemperature = TCI3.fnTCITemperatureDelta();
        #ifdef JENS_DEBUG

            Serial.print(TCI_J28_DeltaTemperature); Serial.print(",");

        #endif

        TCI_J27_Temperature = TCI4.fnTCIThermocoupleTemperature(); 
        #ifdef JENS_DEBUG

            Serial.print(TCI_J27_Temperature); Serial.print(",");

        #endif
        TCI_J27_AmbientTemperature = TCI4.fnTCIAmbientTemperature();
        #ifdef JENS_DEBUG

            Serial.print(TCI_J27_AmbientTemperature); Serial.print(",");

        #endif
        TCI_J27_DeltaTemperature = TCI4.fnTCITemperatureDelta();
        #ifdef JENS_DEBUG

            Serial.print(TCI_J27_DeltaTemperature); Serial.print(",");

        #endif

        TCI_J26_Temperature = TCI5.fnTCIThermocoupleTemperature(); 
        #ifdef JENS_DEBUG

            Serial.print(TCI_J26_Temperature); Serial.print(",");

        #endif
        TCI_J26_AmbientTemperature = TCI5.fnTCIAmbientTemperature(); 
        #ifdef JENS_DEBUG

            Serial.print(TCI_J26_AmbientTemperature); Serial.print(",");

        #endif
        TCI_J26_DeltaTemperature = TCI5.fnTCITemperatureDelta();
        #ifdef JENS_DEBUG

            Serial.print(TCI_J26_DeltaTemperature); Serial.print(",");

        #endif

    #endif 

    #ifdef CO2_CLASS

        Co2_Co21 = CO21.fnCO2Carbondioxide(); 
        #ifdef JENS_DEBUG

            Serial.print(Co2_Co21); Serial.print(",");

        #endif
        Co2_Co22 = CO22.fnCO2Carbondioxide(); 
        #ifdef JENS_DEBUG

            Serial.print(Co2_Co22); Serial.print(",");

        #endif

    #endif 

    #ifdef JENS_DEBUG

        #if defined(PWM_CLASS) || (defined(PWM_CLASS) && defined(GPIO_CLASS)) || (defined(PWM_CLASS) && defined(GPIO_CLASS) && defined(ADC_CLASS))

            Count_of_Comas = 0;
            while(Serial.available())
            {
                InputCharSerialRead = Serial.read(); 
                if(InputCharSerialRead != ',')
                {
                    InputStringSerial += (char)InputCharSerialRead;
                }
                else
                {
                    if(Count_of_Comas == 0 && (uint8_t)InputStringSerial.toInt() != PWM0_InhibitPinMode)
                    {
                        PWM0_InhibitPinMode = (uint8_t)InputStringSerial.toInt();
                        if(PWM0_InhibitPinMode == 1)
                        {
                            PWM_0->fnPWMActiveMode();
                        }
                        else if(PWM0_InhibitPinMode == 0)
                        {
                            PWM_0->fnPWMSleepMode();
                        }
                    }
                    else if(Count_of_Comas == 1 && (uint8_t)InputStringSerial.toInt() != PWM1_InhibitPinMode)
                    {
                        PWM1_InhibitPinMode = (uint8_t)InputStringSerial.toInt();
                        if(PWM1_InhibitPinMode == 1)
                        {
                            PWM_1->fnPWMActiveMode();
                        }
                        else if(PWM1_InhibitPinMode == 0)
                        {
                            PWM_1->fnPWMSleepMode();
                        }
                    }
                    else if(Count_of_Comas == 2 && (uint8_t)InputStringSerial.toInt() != PWM2_InhibitPinMode)
                    {
                        PWM2_InhibitPinMode = (uint8_t)InputStringSerial.toInt();
                        if(PWM2_InhibitPinMode == 1)
                        {
                            PWM_2->fnPWMActiveMode();
                        }
                        else if(PWM2_InhibitPinMode == 0)
                        {
                            PWM_2->fnPWMSleepMode();
                        }
                    }
                    else if(Count_of_Comas == 3 && (uint8_t)InputStringSerial.toInt() != PWM3_InhibitPinMode)
                    {
                        PWM3_InhibitPinMode = (uint8_t)InputStringSerial.toInt();
                        if(PWM3_InhibitPinMode == 1)
                        {
                            PWM_3->fnPWMActiveMode();
                        }
                        else if(PWM3_InhibitPinMode == 0)
                        {
                            PWM_3->fnPWMSleepMode();
                        }
                    }
                    else if(Count_of_Comas == 4 && (uint8_t)InputStringSerial.toInt() != PWM4_InhibitPinMode)
                    {
                        PWM4_InhibitPinMode = (uint8_t)InputStringSerial.toInt();
                        if(PWM4_InhibitPinMode == 1)
                        {
                            PWM_4->fnPWMActiveMode();
                        }
                        else if(PWM4_InhibitPinMode == 0)
                        {
                            PWM_4->fnPWMSleepMode();
                        }
                    }
                    else if(Count_of_Comas == 5 && (uint8_t)InputStringSerial.toInt() != PWM5_InhibitPinMode)
                    {
                        PWM5_InhibitPinMode = (uint8_t)InputStringSerial.toInt();
                        if(PWM5_InhibitPinMode == 1)
                        {
                            PWM_5->fnPWMActiveMode();
                        }
                        else if(PWM5_InhibitPinMode == 0)
                        {
                            PWM_5->fnPWMSleepMode();
                        }
                    }
                    else if(Count_of_Comas == 6 && (uint8_t)InputStringSerial.toInt() < 101 && (uint8_t)InputStringSerial.toInt() != PWM0_Dutycycle)
                    {
                        PWM0_Dutycycle = (uint8_t)InputStringSerial.toInt();
                        PWM_0->fnPWMDutycycleUpdate(PWM0_Dutycycle);
                    }
                    else if(Count_of_Comas == 7 && (uint8_t)InputStringSerial.toInt() < 101 && (uint8_t)InputStringSerial.toInt() != PWM1_Dutycycle)
                    {
                        PWM1_Dutycycle = (uint8_t)InputStringSerial.toInt();
                        PWM_1->fnPWMDutycycleUpdate(PWM1_Dutycycle);
                    }
                    else if(Count_of_Comas == 8 && (uint8_t)InputStringSerial.toInt() < 101 && (uint8_t)InputStringSerial.toInt() != PWM2_Dutycycle)
                    {
                        PWM2_Dutycycle = (uint8_t)InputStringSerial.toInt();
                        PWM_2->fnPWMDutycycleUpdate(PWM2_Dutycycle);
                    }
                    else if(Count_of_Comas == 9 && (uint8_t)InputStringSerial.toInt() < 101 && (uint8_t)InputStringSerial.toInt() != PWM3_Dutycycle)
                    {
                        PWM3_Dutycycle = (uint8_t)InputStringSerial.toInt();
                        PWM_3->fnPWMDutycycleUpdate(PWM3_Dutycycle);
                    }
                    else if(Count_of_Comas == 10 && (uint8_t)InputStringSerial.toInt() < 101 && (uint8_t)InputStringSerial.toInt() != PWM4_Dutycycle)
                    {
                        PWM4_Dutycycle = (uint8_t)InputStringSerial.toInt();
                        PWM_4->fnPWMDutycycleUpdate(PWM4_Dutycycle);
                    }
                    else if(Count_of_Comas == 11 && (uint8_t)InputStringSerial.toInt() < 101 && (uint8_t)InputStringSerial.toInt() != PWM5_Dutycycle)
                    {
                        PWM5_Dutycycle = (uint8_t)InputStringSerial.toInt();
                        PWM_5->fnPWMDutycycleUpdate(PWM5_Dutycycle);
                    }
                    else if(Count_of_Comas == 12 && (uint16_t)InputStringSerial.toInt() != PWM0_PWM1_ADSConversionRate && (uint16_t)InputStringSerial.toInt() == 8 || 16 || 32 || 64 || 128 || 250 || 475 || 860)
                    {
                        PWM0_PWM1_ADSConversionRate = (uint16_t)InputStringSerial.toInt();
                        PWM_0->fnPWMConversionRateUpdate(PWM0_PWM1_ADSConversionRate);
                        PWM_1->fnPWMConversionRateUpdate(PWM0_PWM1_ADSConversionRate);
                    }
                    else if(Count_of_Comas == 13 && (uint16_t)InputStringSerial.toInt() != PWM2_PWM3_ADSConversionRate && (uint16_t)InputStringSerial.toInt() == 8 || 16 || 32 || 64 || 128 || 250 || 475 || 860)
                    {
                        PWM2_PWM3_ADSConversionRate = (uint16_t)InputStringSerial.toInt();
                        PWM_2->fnPWMConversionRateUpdate(PWM2_PWM3_ADSConversionRate);
                        PWM_3->fnPWMConversionRateUpdate(PWM2_PWM3_ADSConversionRate);
                    }
                    else if(Count_of_Comas == 14 && (uint16_t)InputStringSerial.toInt() != PWM4_PWM5_ADSConversionRate && (uint16_t)InputStringSerial.toInt() == 8 || 16 || 32 || 64 || 128 || 250 || 475 || 860)
                    {
                        PWM4_PWM5_ADSConversionRate = (uint16_t)InputStringSerial.toInt();
                        PWM_4->fnPWMConversionRateUpdate(PWM4_PWM5_ADSConversionRate);
                        PWM_5->fnPWMConversionRateUpdate(PWM4_PWM5_ADSConversionRate);
                    }
                    else if(Count_of_Comas == 15 && (uint32_t)InputStringSerial.toInt() < 20001 && (uint32_t)InputStringSerial.toInt() != PWM0_PWM1_Frequency)
                    {
                        PWM0_PWM1_Frequency = (uint32_t)InputStringSerial.toInt();
                        PWM_0->fnPWMFrequencyFromStart();
                        PWM_1->fnPWMFrequencyFromStart();
                        PWM_0->fnPWMFrequencyUpdate(PWM0_PWM1_Frequency);
                        PWM_1->fnPWMFrequencyUpdate(PWM0_PWM1_Frequency);
                    }
                    else if(Count_of_Comas == 16 && (uint32_t)InputStringSerial.toInt() < 20001 && (uint32_t)InputStringSerial.toInt() != PWM2_PWM3_Frequency)
                    {
                        PWM2_PWM3_Frequency = (uint32_t)InputStringSerial.toInt();
                        PWM_2->fnPWMFrequencyFromStart();
                        PWM_3->fnPWMFrequencyFromStart();
                        PWM_2->fnPWMFrequencyUpdate(PWM2_PWM3_Frequency);
                        PWM_3->fnPWMFrequencyUpdate(PWM2_PWM3_Frequency);
                    }
                    else if(Count_of_Comas == 17 && (uint32_t)InputStringSerial.toInt() < 20001 && (uint32_t)InputStringSerial.toInt() != PWM4_PWM5_Frequency)
                    {
                        PWM4_PWM5_Frequency = (uint32_t)InputStringSerial.toInt();
                        PWM_4->fnPWMFrequencyFromStart();
                        PWM_5->fnPWMFrequencyFromStart();
                        PWM_4->fnPWMFrequencyUpdate(PWM4_PWM5_Frequency);
                        PWM_5->fnPWMFrequencyUpdate(PWM4_PWM5_Frequency);
                    }
                    else if(Count_of_Comas == 18 && (uint8_t)InputStringSerial.toInt() != GPIO_IND_GPO0)
                    {
                        if((uint8_t)InputStringSerial.toInt() == 0)
                        {
                            GPIO0.fnGPIOPinValue(LOW);
                        }
                        else if((uint8_t)InputStringSerial.toInt() == 1)
                        {
                            GPIO0.fnGPIOPinValue(HIGH);
                        }
                    }
                    else if(Count_of_Comas == 19 && (uint8_t)InputStringSerial.toInt() != GPIO_IND_GPO1)
                    {
                        if((uint8_t)InputStringSerial.toInt() == 0)
                        {
                            GPIO1.fnGPIOPinValue(LOW);
                        }
                        else if((uint8_t)InputStringSerial.toInt() == 1)
                        {
                            GPIO1.fnGPIOPinValue(HIGH);
                        }
                    }
                    else if(Count_of_Comas == 20 && (uint8_t)InputStringSerial.toInt() != GPIO_IND_GPO2)
                    {
                        if((uint8_t)InputStringSerial.toInt() == 0)
                        {
                            GPIO2.fnGPIOPinValue(LOW);
                        }
                        else if((uint8_t)InputStringSerial.toInt() == 1)
                        {
                            GPIO2.fnGPIOPinValue(HIGH);
                        }
                    }
                    else if(Count_of_Comas == 21 && (uint8_t)InputStringSerial.toInt() != GPIO_IND_GPO3)
                    {
                        if((uint8_t)InputStringSerial.toInt() == 0)
                        {
                            GPIO3.fnGPIOPinValue(LOW);
                        }
                        else if((uint8_t)InputStringSerial.toInt() == 1)
                        {
                            GPIO3.fnGPIOPinValue(HIGH);
                        }
                    }
                    else if(Count_of_Comas == 22 && (uint8_t)InputStringSerial.toInt() != GPIO_IND_GPO4)
                    {
                        if((uint8_t)InputStringSerial.toInt() == 0)
                        {
                            GPIO4.fnGPIOPinValue(LOW);
                        }
                        else if((uint8_t)InputStringSerial.toInt() == 1)
                        {
                            GPIO4.fnGPIOPinValue(HIGH);
                        }
                    }
                    else if(Count_of_Comas == 23 && (uint8_t)InputStringSerial.toInt() != GPIO_IND_GPO5)
                    {
                        if((uint8_t)InputStringSerial.toInt() == 0)
                        {
                            GPIO5.fnGPIOPinValue(LOW);
                        }
                        else if((uint8_t)InputStringSerial.toInt() == 1)
                        {
                            GPIO5.fnGPIOPinValue(HIGH);
                        }
                    }
                    else if(Count_of_Comas == 24 && (uint16_t)InputStringSerial.toInt() != u16ADCADSConversionRate && (uint16_t)InputStringSerial.toInt() == 8 || 16 || 32 || 64 || 128 || 250 || 475 || 860)
                    {
                        u16ADCADSConversionRate = (uint16_t)InputStringSerial.toInt();
                        ADC_0.fnADCConversionRateUpdate(u16ADCADSConversionRate);
                        ADC_1.fnADCConversionRateUpdate(u16ADCADSConversionRate);
                        ADC_2.fnADCConversionRateUpdate(u16ADCADSConversionRate);
                        ADC_3.fnADCConversionRateUpdate(u16ADCADSConversionRate);
                    }
                    InputStringSerial = "";
                    Count_of_Comas++;
                }
            }
            InputStringSerial = "";

        #endif

        Current_Time = (micros() - Current_Time);
        Serial.println(Current_Time);
        if(Current_Time > 500000) // Since when I run evrythingthing together it was taking less than 350 milli. ADS1115 is always goes to long delay by itself. So, this condition is given here to manage that.
        {
            ADC_0.fnADCConversionRateUpdate(u16ADCADSConversionRate);
            ADC_1.fnADCConversionRateUpdate(u16ADCADSConversionRate);
            ADC_2.fnADCConversionRateUpdate(u16ADCADSConversionRate);
            ADC_3.fnADCConversionRateUpdate(u16ADCADSConversionRate);
            PWM_0->fnPWMConversionRateUpdate(PWM0_PWM1_ADSConversionRate);
            PWM_1->fnPWMConversionRateUpdate(PWM0_PWM1_ADSConversionRate);
            PWM_2->fnPWMConversionRateUpdate(PWM2_PWM3_ADSConversionRate);
            PWM_3->fnPWMConversionRateUpdate(PWM2_PWM3_ADSConversionRate);
            PWM_4->fnPWMConversionRateUpdate(PWM4_PWM5_ADSConversionRate);
            PWM_5->fnPWMConversionRateUpdate(PWM4_PWM5_ADSConversionRate);
        }
        else
        {
            delay(150);
        }

    #endif
}