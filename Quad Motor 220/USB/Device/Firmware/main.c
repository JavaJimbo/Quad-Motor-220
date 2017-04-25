/***************************************************************************************
 * Project:     Olimex 220 Robot
 * FileName:    main.c 
 * 
 *
 * 1-15-17: 
 * 2-25-17: Add decoding and 
 * 2-28-17: Added PIDcontrol() but haven't tested it yet.xxx
 * 4-22-17: For Quad Motor Board V1.0
 * 4-23-17: Got most functions working on Rev 1 Quad board.
 * Removed PCA9685 code
 * 4-24-17: USB works. Encoders work. 
 ****************************************************************************************/
// #define USE_USB
#define USE_PWM
// #define USE_AD
// #define TEST_EEPROM

// #define I2C_CLOCK_FREQ              100000
// #define EEPROM_I2C_BUS              I2C1
// #define EEPROM_ADDRESS              0x50        // 0b1010000 Serial EEPROM address    
#define DISABLE_OUT PORTBbits.RB4
#define FAULT_IN PORTCbits.RC1     

#define EncoderOne TMR1
#define EncoderTwo TMR4
#define EncoderThree TMR3
#define EncoderFour TMR5


#define ENC1_LATCH PORTAbits.RA4
#define ENC2_LATCH PORTBbits.RB2
#define ENC3_LATCH PORTBbits.RB1
#define ENC4_LATCH PORTBbits.RB0

#define ENC1_DIR PORTBbits.RB15
#define ENC2_DIR PORTBbits.RB14
#define ENC3_DIR PORTBbits.RB13
#define ENC4_DIR PORTCbits.RC9

#define PWM1 OC4RS
#define PWM2 OC3RS
#define PWM3 OC2RS
#define PWM4 OC1RS

#define DIR1_OUT LATAbits.LATA7
#define DIR2_OUT LATCbits.LATC5
#define DIR3_OUT LATAbits.LATA10
#define DIR4_OUT LATCbits.LATC7


#define STX 36
#define ETX 13
#define DLE 16
#define MAXPACKET 80
#define MAXVELOCITY 500
#define DRIVEDIRECT 145
#define ROOMBA 0
#define RASPI 240
#define ROBOTNIK 19
#define SETPID 69
#define START 128
#define STOP 173
#define POWERDOWN 133
#define RESET 7
#define SAFE 131
#define FULL 132
#define QUIT 128
#define SHUTDOWN 160

#define STANDBY 0
#define RUN 111


#define SYS_FREQ 60000000
#define GetPeripheralClock() SYS_FREQ 
// #define USE_LINX


#include "USB/usb.h"
#include "USB/usb_function_cdc.h"
#include "HardwareProfile.h"
#include "I2C_EEPROM_PIC32.h"
#include "PCA9685.h"
#include "Delay.h"
#include <plib.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_15        // PLL Multiplier for 220 - yields 60 Mhz
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx3      // ICE/ICD Comm Channel Select
#pragma config JTAGEN   = OFF           // Use JTAG pins for normal IO
#pragma config DEBUG    = OFF            // Enable/disable debugging


/*** DEFINES *****************************************************************/
#define HOSTuart UART2
#define SYS_FREQ 60000000  // With 8 Mhz crystal and FPLLMUL = MUL_15
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR     
#define MAXBUFFER 64

#define false FALSE
#define true TRUE

#define START_ONE 80
#define START_TWO 80
#define START_THREE 20
#define START_FOUR 20
#define TIMEOUT 200
#define UART_TIMEOUT 400
#define MAXBITLENGTH 20

/** I N C L U D E S **********************************************************/
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "USB/usb_device.h"
#include "USB/usb.h"
#include "HardwareProfile.h"


/** V A R I A B L E S ********************************************************/
#define MAXDATABYTES 64
unsigned char arrData[MAXDATABYTES];
unsigned char HOSTRxBuffer[MAXBUFFER+1];
unsigned char HOSTTxBuffer[MAXBUFFER+1];
unsigned char HOSTRxBufferFull = FALSE;
unsigned int RxDataLength = 0;

unsigned char USBRxBuffer[MAXBUFFER];
unsigned char USBTxBuffer[MAXBUFFER];

unsigned short RxLength = 0;
unsigned short TxLength = 0;

unsigned short previousExpected = 0, numExpectedBytes = 0;
unsigned char error = 0;
unsigned char RXstate = 0;
unsigned char timeoutFlag = FALSE;
unsigned short numBytesReceived = 0;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
// extern unsigned short CRCcalculate(unsigned char *message, unsigned char nBytes);
extern unsigned char getCRC8(unsigned char *ptrMessage, short numBytes);
static void InitializeSystem(void);
void ProcessIO(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
// void USBCBSendResume(void);
void BlinkUSBStatus(void);
void UserInit(void);
void ConfigAd(void);

unsigned char PCAReadByte(unsigned char device, unsigned char PCAcontrolRegister, unsigned char *ptrData);
unsigned char PCAWriteByte(unsigned char device, unsigned char PCAcontrolRegister, unsigned char data);

unsigned char setPCA9685outputs(unsigned char device, unsigned short channel, unsigned short turnON, unsigned short turnOFF);
unsigned char initializePCA9685(unsigned char device);
unsigned int decodePacket(unsigned char *ptrInPacket, unsigned int numInBytes, unsigned char *ptrData);

void Halt(void);
unsigned char testRUN(short PWMvalue, unsigned char direction);
long getPositionError(short i, short targetSpeed);
long PIDcontrol(long error);
// void initializeErrorArrays(void);
unsigned char setMotorPWM(short side, short PWMvalue, unsigned char direction);

void putch(unsigned char ch);

#define PWM_OFFSET 800

long kP = 50, kI = 0, kD = 0;


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void) {
#ifdef USE_USB    
#if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

#if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN; // See HardwareProfile.h
#endif
#endif
    
    UserInit();

#ifdef USE_USB
    USBDeviceInit();
#endif

}//end InitializeSystem


void BlinkUSBStatus(void) {
    static WORD led_count = 0;

    if (led_count == 0)led_count = 10000U;
    led_count--;

#define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
#define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
#define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
#define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if (USBSuspendControl == 1) {
        if (led_count == 0) {
            mLED_1_Toggle();
            if (mGetLED_1()) mLED_2_On()
            else mLED_2_Off()
            }//end if
    } else {
        if (USBDeviceState == DETACHED_STATE)
            mLED_Both_Off()
        else if (USBDeviceState == ATTACHED_STATE)
            mLED_Both_On()
        else if (USBDeviceState == POWERED_STATE)
            mLED_Only_1_On()
        else if (USBDeviceState == DEFAULT_STATE)
            mLED_Only_2_On()
        else if (USBDeviceState == ADDRESS_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle()
                mLED_2_Off()
            }//end if
        } else if (USBDeviceState == CONFIGURED_STATE) {
            if (led_count == 0) {
                mLED_1_Toggle()
                if (mGetLED_1())
                    mLED_2_Off()
                else mLED_2_On()
                }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus

void USBCBSuspend(void) {
    ;
}

void USBCBWakeFromSuspend(void) {
    ;
}

void USBCB_SOF_Handler(void) {
    ;
}

void USBCBErrorHandler(void) {
    ;
}

void USBCBCheckOtherReq(void) {
    USBCheckCDCRequest();
}

void USBCBStdSetDscHandler(void) {
    ;
}

void USBCBInitEP(void) {
    CDCInitEP();
}

/*
void USBCBSendResume(void) {
    static WORD delay_count;

    if (USBGetRemoteWakeupStatus() == TRUE) {
        if (USBIsBusSuspended() == TRUE) {
            USBMaskInterrupts();
            USBCBWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = FALSE; //So we don't execute this code again, 
            //until a new suspend condition is detected.
            delay_count = 3600U;
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 1; // Start RESUME signaling
            delay_count = 1800U; // Set RESUME line for 1-13 ms
            do {
                delay_count--;
            } while (delay_count);
            USBResumeControl = 0; //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}
 */

#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)

void USBCBEP0DataReceived(void) {
}
#endif

BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size) {
    switch (event) {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            break;
        default:
            break;
    }
    return TRUE;
}

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
#ifdef USE_USB

void ProcessIO(void) {
    static unsigned char ch, USBTxIndex = 0;
    unsigned short i, length;
    BYTE numBytesRead;

    // Blink the LEDs according to the USB device status
    BlinkUSBStatus();

    // User Application USB tasks
    if ((USBDeviceState < CONFIGURED_STATE) || (USBSuspendControl == 1)) return;

    numBytesRead = getsUSBUSART(USBRxBuffer, 64);

    for (i = 0; i < numBytesRead; i++) {
        ch = USBRxBuffer[i];
        if (USBTxIndex < MAXBUFFER - 2) HOSTTxBuffer[USBTxIndex++] = ch;
        if (ch == '\r') {
            HOSTTxBuffer[USBTxIndex] = '\0';
            printf("\rUSB RECEIVED: %s", HOSTTxBuffer);
            USBTxIndex = 0;
        }
    }

    if (USBUSARTIsTxTrfReady()) {
        if (HOSTRxBufferFull) {
            HOSTRxBufferFull = FALSE;
            length = strlen(HOSTRxBuffer);
            if (length > MAXBUFFER) length = MAXBUFFER;
            putUSBUSART(HOSTRxBuffer, length);
        }
    }
    CDCTxService();
} //end ProcessIO
#endif

#ifdef USE_AD    

void ConfigAd(void) {

    mPORTCSetPinsAnalogIn(BIT_1);

    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31

    //  set AM7 (A1 on Olimex 220 board) input to analog
    // #define PARAM4    ENABLE_AN0_ANA | ENABLE_AN1_ANA| ENABLE_AN2_ANA | ENABLE_AN3_ANA
#define PARAM4    ENABLE_AN7_ANA


    // USE AN7    
#define PARAM5 SKIP_SCAN_AN0 |SKIP_SCAN_AN1 |SKIP_SCAN_AN2 |SKIP_SCAN_AN3 |\
SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 |\
SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 |\
SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15


    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();
}

void __ISR(_ADC_VECTOR, ipl6) ADHandler(void) {
    unsigned short offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();


    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < MAXPOTS; i++)
        arrPots[i] = (unsigned short) ReadADC10(offSet + i); // read the result of channel 0 conversion from the idle buffer
    ADflag = TRUE;
}
#endif

union convertType {
    unsigned char byte[2];
    short integer;
} convert;

#define NUM_ENCODERS 2
#define LEFT 0
#define RIGHT 1
long actualPos[NUM_ENCODERS], targetPos[NUM_ENCODERS];

long errorLeft, errorRight, Lcorr, Rcorr, PWMleft, PWMright, posLeft, posRight;
unsigned char ch;

#define MAXSPEED 1000

int main(void) {
unsigned char MOT1Direction = 0, MOT2Direction = 0, MOT3Direction = 0, MOT4Direction = 0;
unsigned char PrevMOT1Direction = 0, PrevMOT2Direction = 0, PrevMOT3Direction = 0, PrevMOT4Direction = 0;
unsigned char NewDirection = 0;
long wheel1 = 0, wheel2 = 0, wheel3 = 0, wheel4 = 0;
long velocity1 = 0, velocity2 = 0, velocity3 = 0, velocity4 = 0;
short displayCounter = 0;
unsigned char displayMode = FALSE;

    InitializeSystem();
    DelayMs(200);
    printf("\rTESTING EVERYTHING ALL TOGETHER INCLUDING PROGRAMMING");
    while(1){       
        
        if (ch){
            printf ("\rCH: %c", ch);
            switch (ch){
                case 'F':
                    PWM1 = PWM2 = PWM3 = PWM4 =  MAXSPEED;
                    DIR1_OUT = DIR2_OUT = DIR3_OUT = DIR4_OUT = FORWARD;
                    printf(" FORWARD");
                    break;
                    
                case 'R':
                    PWM1 = PWM2 = PWM3 = PWM4 =  MAXSPEED;
                    DIR1_OUT = DIR2_OUT = DIR3_OUT = DIR4_OUT = REVERSE;
                    printf(" REVERSE");
                    break;     
                    
                case 'D':
                    if (displayMode) displayMode = FALSE;
                    else displayMode = TRUE;
                    printf("\rDISPLAY MODE = %d", displayMode);
                    break;                    
                    
                case ' ':                
                    PWM1 = PWM2 = PWM3 = PWM4 = 0;
                    printf(" HALT");
                    break;
            }
            ch = 0;
        }
        
        long tempEncoder;
        
        tempEncoder = (long) EncoderOne; EncoderOne = 0;
        velocity1 = velocity1 + tempEncoder;
        if (MOT1Direction == FORWARD) wheel1 = wheel1 + tempEncoder;
        else wheel1 = wheel1 - tempEncoder;
        
        tempEncoder = (long) EncoderTwo; EncoderTwo = 0;
        velocity2 = velocity2 + tempEncoder;
        if (MOT2Direction == FORWARD) wheel2 = wheel2 + tempEncoder;
        else wheel2 = wheel2 - tempEncoder;
        
        tempEncoder = (long) EncoderThree; EncoderThree = 0;
        velocity3 = velocity3 + tempEncoder;
        if (MOT3Direction == FORWARD) wheel3 = wheel3 + tempEncoder;
        else wheel3 = wheel3 - tempEncoder;
        
        tempEncoder = (long) EncoderFour; EncoderFour = 0;
        velocity4 = velocity4 + tempEncoder;
        if (MOT4Direction == FORWARD) wheel4 = wheel4 + tempEncoder;
        else wheel4 = wheel4 - tempEncoder;
        
        DelayMs(1);
        if (displayMode){
            displayCounter++;
            #define MAXLOOP 128
            if (displayCounter >= MAXLOOP){
                printf("\rM1: %d V: %d, M2: %d V: %d , M3: %d V: %d, M4: %d V: %d", wheel1, velocity1, wheel2, velocity2, wheel3, velocity3, wheel4, velocity4);
                displayCounter = 0;
                velocity1 = velocity2 = velocity3 = velocity4 = 0;
            }
        } else displayCounter = 0;
        
        if (ENC1_LATCH){
            NewDirection = ENC1_DIR;
            if (ENC1_LATCH){
                MOT1Direction = NewDirection;
                if (PrevMOT1Direction != MOT1Direction){
                    EncoderOne = 0;
                    if (MOT1Direction == FORWARD) printf ("\rMOT1 = FORWARD");
                    else  printf ("\rMOT1 = REVERSE");
                } 
                PrevMOT1Direction = MOT1Direction;
            }
        }        
        
        if (ENC2_LATCH){
            NewDirection = ENC2_DIR;
            if (ENC2_LATCH){
                MOT2Direction = NewDirection;
                if (PrevMOT2Direction != MOT2Direction){
                    EncoderTwo = 0;
                    if (MOT2Direction == FORWARD) printf ("\rMOT2 = FORWARD");
                    else  printf ("\rMOT2 = REVERSE");
                } 
                PrevMOT2Direction = MOT2Direction;
            }
        }
        
        if (ENC3_LATCH){
            NewDirection = ENC3_DIR;
            if (ENC3_LATCH){
                MOT3Direction = NewDirection;
                if (PrevMOT3Direction != MOT3Direction){
                    EncoderThree = 0;
                    if (MOT3Direction == FORWARD) printf ("\rMOT3 = FORWARD");
                    else  printf ("\rMOT3 = REVERSE");
                } 
                PrevMOT3Direction = MOT3Direction;
            }
        }

        if (ENC4_LATCH){
            NewDirection = ENC4_DIR;
            if (ENC4_LATCH){
                MOT4Direction = NewDirection;
                if (PrevMOT4Direction != MOT4Direction){
                    EncoderFour = 0;
                    if (MOT4Direction == FORWARD) printf ("\rMOT4 = FORWARD");
                    else  printf ("\rMOT4 = REVERSE");
                } 
                PrevMOT4Direction = MOT4Direction;
            }
        }
        
#ifdef USE_USB           
        #if defined(USB_INTERRUPT)
            if(USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE))
            {
                USBDeviceAttach();
            }
        #endif
        

        #if defined(USB_POLLING)
		// Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        				  // this function periodically.  This function will take care
        				  // of processing and responding to SETUP transactions 
        				  // (such as during the enumeration process when you first
        				  // plug in).  USB hosts require that USB devices should accept
        				  // and process SETUP packets in a timely fashion.  Therefore,
        				  // when using polling, this function should be called 
        				  // regularly (such as once every 1.8ms or faster** [see 
        				  // inline code comments in usb_device.c for explanation when
        				  // "or faster" applies])  In most cases, the USBDeviceTasks() 
        				  // function does not take very long to execute (ex: <100 
        				  // instruction cycles) before it returns.
        #endif				  
		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();                 
#endif
    } // end while(1)

#ifdef TEST_EEPROM    
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    DelayMs(200);

#endif
    
}



void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
    static unsigned char startFlag = false;
    static unsigned char escapeFlag = false;
    static unsigned int RxIndex = 0;
    // unsigned char ch;

    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                ch = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;
            RxIndex = 0;
        }

        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = toupper(UARTGetDataByte(HOSTuart));
            if (RxIndex < MAXBUFFER) HOSTRxBuffer[RxIndex++] = ch;
            if (ch == '\r'){
                HOSTRxBufferFull = TRUE;
                HOSTRxBuffer[RxIndex] = '\0';
                RxIndex = 0;
            }
        }
            /*
            //if (ch != 0 && ch != '\n') {
            // Store next char if packet is valid and board number matches
            if (startFlag && RxIndex < MAXBUFFER) HOSTRxBuffer[RxIndex] = ch;

            // If preceding character wasn't an escape char:
            // check whether it is STX, ETX or DLE,
            // otherwise if board number matches then store and advance for next char
            if (escapeFlag == false) {
                if (ch == DLE) escapeFlag = true;
                else if (ch == STX) {
                    RxIndex = 0;
                    startFlag = true;
                } else if (ch == ETX && startFlag) {
                    startFlag = false;
                    RxDataLength = RxIndex;
                    RxIndex = 0;
                } else if (startFlag) RxIndex++;
            }// Otherwise if preceding character was an escape char:	
            else {
                escapeFlag = false;
                if (startFlag) RxIndex++;
            }
            //}            
        }
        */
        if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
            INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
        }
    }
}

/*
void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void) {
    mT1ClearIntFlag(); // Clear interrupt flag
    if (TimerCounter) TimerCounter--;
}
*/
void putch(unsigned char ch) {
    while (!IFS1bits.U2TXIF); // set when register is empty 
    U2TXREG = ch;
}

#define MULTIPLIER 100
long getPositionError(short wheel, short targetSpeed){    
    long error, newPos, lngTargetSpeed;
    
    if (wheel >= NUM_ENCODERS) return (0);

    if (wheel == LEFT) {
        newPos = (long) LEFTREARENC;
        //LEFTREARENC = 0;
    }
    else{
        newPos = (long) RIGHTREARENC;
        //RIGHTREARENC = 0;        
    }

    lngTargetSpeed = (long) abs(targetSpeed);

    actualPos[wheel] = (newPos * (long) MULTIPLIER);
    targetPos[wheel] = targetPos[wheel] + lngTargetSpeed;

    if (wheel == LEFT) posLeft = actualPos[wheel];
    else posRight = actualPos[wheel];
    
    error = actualPos[wheel] - targetPos[wheel];
    
    if (wheel == LEFT) {
      errorLeft = error / MULTIPLIER;
    }
    else {
      errorRight = error / MULTIPLIER;
    }

    return (error);
}

#define DIVIDER 10000
long PIDcontrol(long error) {
    long PIDcorrection;
    long PCorr = 0;
    
    PCorr = error * kP;
    PIDcorrection = PCorr;
    PIDcorrection = PIDcorrection / DIVIDER;
    if (PIDcorrection > PWM_MAX) PIDcorrection = PWM_MAX;
    if (PIDcorrection < -PWM_MAX) PIDcorrection = -PWM_MAX;

    return (PIDcorrection);
}

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 * PPSInput(2,U2RX,RPA9);       // Assign U2RX to pin RPA09
 * PPSInput(3,U2CTS,RPC3);      // Assign U2CTS to pin RPC3
 * PPSOutput(4,RPC4,U2TX);      // Assign U2TX to pin RPC4
 * PPSOutput(1,RPB15,U2RTS);    // Assign U2RTS to pin RPB15    
 *
 *****************************************************************************/


void UserInit(void) {
    // unsigned char ch;

    mJTAGPortEnable(false);
    PORTSetPinsDigitalOut(IOPORT_C, BIT_5 | BIT_7);
    PORTSetPinsDigitalOut(IOPORT_A, BIT_7 | BIT_10);    
    PORTSetPinsDigitalOut(IOPORT_B, BIT_4);  
    
    DIR1_OUT = DIR2_OUT = DIR3_OUT = DIR4_OUT = 0;    
    
    PORTSetPinsDigitalIn(IOPORT_C, BIT_1 | BIT_9);    
    
    ANSELBbits.ANSB15 = 0;
    ANSELBbits.ANSB14 = 0;
    ANSELBbits.ANSB13 = 0;
    PORTSetPinsDigitalIn(IOPORT_B, BIT_13 | BIT_14 | BIT_15);
    DISABLE_OUT = 0;    

    PORTSetPinsDigitalIn(IOPORT_B, BIT_13 | BIT_14 | BIT_15);

    // Set up main UART    
    PPSOutput(4, RPC2, U2TX);
    PPSInput(2, U2RX, RPA8);
    
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    
    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_U2RX, INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);

     do {        
        ch = UARTGetDataByte(HOSTuart);
    } while (ch);
    
    /*
    do {        
        if (UARTReceivedDataIsAvailable(HOSTuart)){
            ch = UARTGetDataByte(HOSTuart);
            if (ch) printf ("\rCH: %c", ch);
        }
        DelayMs(2);
    } while (1);
    */
    
    // Set up Timer 1 interrupt with a priority of 2
    //ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
    //OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_1, 60000);
    
    // Set up counter inputs
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB1 = 0;
    ANSELBbits.ANSB2 = 0;
    
    PPSInput(4, T5CK, RPB0);
    PPSInput(2, T3CK, RPB1);
    PPSInput(3, T4CK, RPB2);       

    // Set up timers as counters
    T1CON = 0x00;
    T1CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T1CONbits.TCKPS1 = 0; // 1:1 Prescaler
    T1CONbits.TCKPS0 = 0;
    T1CONbits.TSYNC = 1;
    PR1 = 0xFFFF;
    T1CONbits.TON = 1; // Let her rip 
    
    T3CON = 0x00;
    T3CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T3CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T3CONbits.TCKPS1 = 0;
    T3CONbits.TCKPS0 = 0;    
    PR3 = 0xFFFF;
    T3CONbits.TON = 1; // Let her rip 

    T4CON = 0x00;
    T4CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T4CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T4CONbits.TCKPS1 = 0;
    T4CONbits.TCKPS0 = 0;
    T4CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR4 = 0xFFFF;
    T4CONbits.TON = 1; // Let her rip 

    T5CON = 0x00;
    T5CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T5CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T5CONbits.TCKPS1 = 0;
    T5CONbits.TCKPS0 = 0;
    PR5 = 0xFFFF;
    T5CONbits.TON = 1; // Let her rip 



#ifdef USE_PWM    
    // Set up Timer 2 for PWM time base    
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;
    PR2 = 1024; // Use 50 microsecond rollover for 20 khz
    T2CONbits.TON = 1; // Let her rip

    // Set up PWM OC1
    PPSOutput(1, RPB7, OC1);
    OC1CON = 0x00;
    OC1CONbits.OC32 = 0; // 16 bit PWM
    OC1CONbits.ON = 1; // Turn on PWM
    OC1CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC1CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC1CONbits.OCM1 = 1;
    OC1CONbits.OCM0 = 0;
    OC1RS = 0;

    // Set up PWM OC2
    PPSOutput(2, RPC8, OC2);
    OC2CON = 0x00;
    OC2CONbits.OC32 = 0; // 16 bit PWM
    OC2CONbits.ON = 1; // Turn on PWM
    OC2CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC2CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC2CONbits.OCM1 = 1;
    OC2CONbits.OCM0 = 0;
    OC2RS = 0;

    // Set up PWM OC3
    PPSOutput(4, RPC4, OC3);
    OC3CON = 0x00;
    OC3CONbits.OC32 = 0; // 16 bit PWM
    OC3CONbits.ON = 1; // Turn on PWM
    OC3CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC3CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC3CONbits.OCM1 = 1;
    OC3CONbits.OCM0 = 0;
    OC3RS = 0;

    // Set up PWM OC4 on D6 on the Olimex 220 board:
    PPSOutput(3, RPC6, OC4);
    OC4CON = 0x00;
    OC4CONbits.OC32 = 0; // 16 bit PWM
    OC4CONbits.ON = 1; // Turn on PWM
    OC4CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC4CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC4CONbits.OCM1 = 1;
    OC4CONbits.OCM0 = 0;
    OC4RS = 0;

    //PORTSetPinsDigitalOut(IOPORT_A, BIT_1 | BIT_10);
    //PORTClearBits(IOPORT_A, BIT_1 | BIT_10);
    //ANSELAbits.ANSA1 = 0;

    OC1RS = 0;
    OC2RS = 0;
    OC4RS = 0;
    OC3RS = 0;
    
    // PWM1 = PWM2 = PWM3 = PWM4 = 1000;
#endif

#ifdef USE_AD    
    ConfigAd();
#endif    

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();
}//end UserInit
