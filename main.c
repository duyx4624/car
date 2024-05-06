#include "stdint.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "Devices/Reflectance.h"
#include "Devices/HR-SC04.h"
#include "Hardware/Clock.h"
#include "Devices/Motor.h"
#include <stdio.h>
#include "Hardware/CortexM.h"
#include "Devices/LQ12864.h"
#include "xunji.h"
#include <Devices/MSPIO.h>
#include <exchange.h>
#include <Devices/key.h>
uint8_t Data=0,PData=0,keyc=0,flag1=0,temp;
uint32_t count=0,count1=0;//计数
float time=0.0;
/* Application Defines  */
#define TIMER_PERIOD1    0x61A7
#define TIMER_PERIOD2    0xEA5F

#define BUFFER_SIZE    128

/*Data Buffer*/
char Buffer[BUFFER_SIZE];

/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 19200 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
//19200
eUSCI_UART_ConfigV1 UART0Config =
{
     EUSCI_A_UART_CLOCKSOURCE_SMCLK,//48mHz
     156,
     4,
     0,
     EUSCI_A_UART_NO_PARITY,
     EUSCI_A_UART_LSB_FIRST,
     EUSCI_A_UART_ONE_STOP_BIT,
     EUSCI_A_UART_MODE,
     EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
};

/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 115200 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
eUSCI_UART_ConfigV1 UART2Config =
{
     EUSCI_A_UART_CLOCKSOURCE_SMCLK,
     26,
     0,
     111,
     EUSCI_A_UART_NO_PARITY,
     EUSCI_A_UART_LSB_FIRST,
     EUSCI_A_UART_ONE_STOP_BIT,
     EUSCI_A_UART_MODE,
     EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
};

const Timer_A_UpModeConfig upConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        TIMER_PERIOD1,
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
        TIMER_A_DO_CLEAR
};

const Timer_A_UpModeConfig upConfig1 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        TIMER_PERIOD1,
        TIMER_A_TAIE_INTERRUPT_DISABLE,
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
        TIMER_A_DO_CLEAR
};

void LCD(){
    uint8_t temp1[100],i;
    char temp2[9];

    decToBinary(Data, temp2);
    for (i = 0; i < 8; ++i) {
        temp1[i] = temp2[i];
    }
    temp1[8] = '\0';
    LCD_P6x8Str(45,2,"4:");
    LCD_P8x16Str(60,2,temp1);

    char temp4[20];
    floatToChar(time,temp4);
    LCD_P6x8Str(45,6,"time:");
    LCD_P6x8Str(80, 6, (uint8_t *)temp4);

    LCD_P14x16Ch(1,2,6);//杜
    LCD_P14x16Ch(15,2,7);//昱
    LCD_P14x16Ch(29,2,8);//兴
    LCD_P14x16Ch(1,4,3);//韩
    LCD_P14x16Ch(15,4,4);//道
    LCD_P14x16Ch(29,4,5);//源
    LCD_P14x16Ch(1,6,0);//王
    LCD_P14x16Ch(15,6,1);//泽
    LCD_P14x16Ch(29,6,2);//麟
}

void Timer_A_init(void)
{
    /* Configuring Timer_A1,Timer_A3 for Up Mode */
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig);
    Timer_A_configureUpMode(TIMER_A3_BASE, &upConfig1);
    /* Enabling interrupts and starting the timer */
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableInterrupt(INT_TA1_0);
    Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    Interrupt_enableInterrupt(INT_TA3_0);
    Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);
    EnableInterrupts();   // clear the I bit
}

int main(void){
    WDT_A_holdTimer(); // 停用看门狗定时器
    Clock_Init48MHz();
    Reflectance_Init();
    Motor_Init(10000);
    LCD_Init();//初始化LCD
    /* Configuring GPIO as an output */
    Timer_A_init();
    key_init();
    UART_Init(EUSCI_A0_BASE, UART0Config);
    UART_Init(EUSCI_A2_BASE, UART2Config);
    Interrupt_enableMaster();
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
    Data = Reflectance_Read();
    while(1){
    WaitForInterrupt();
    }
}

void TA1_0_IRQHandler(void)
{
    Data = Reflectance_Read();
    if(keyc){
       time+=0.025;
       temp=xunji(Data,PData,time);
       if(temp==1){
           Motor(0,0);
           keyc = 0;
       }else if(temp==2){
           time+=0.2;
       }
    }
   PData = Data;
    if(count%15==0){
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
    };
    count++;
    Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);//清空
}

void TA3_0_IRQHandler(void){
    LCD();
    key();
    if(keyc){
        if(!flag1){
            time=0.0;
                }
       LCD_P8x16Str(50,0,"OPEN ");
       flag1=1;
    }else{
        LCD_P8x16Str(50,0,"CLOSE");
        ReXnji();
        if(flag1){
            LCD_P6x8Str(80,6,"      ");
        }
        flag1=0;
    }
    if(count1%10==0){
            GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN2);
        };
        count1++;

    Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);//清空
}
