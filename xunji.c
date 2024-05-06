#include "stdint.h"
#include "Hardware/Clock.h"
#include "xunji.h"
#include "Devices/Motor.h"
#include <stdio.h>
#include <string.h>
#include "Devices/Reflectance.h"
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

//数据端口输出逻辑：
//• 当某一路灰度值接近白场时，这一路的数据端口输出高电平。
//• 当某一路灰度值接近黑场时，这一路的数据端口输出低电平。

enum State
{
    RapidLeft = -2,//直角左弯
    TurnLeft = -1,//
    TurnRight = 1,//
    RapidRight = 2,//直角右弯
    Straight = 0,//直行加小弯
    other = 5,
    stop = 3,//停车
    out = 10//出界
} typedef State;

float Kp = 15, Ki = 0.05, Kd = 3;                      //pid弯道参数参数
float P = 0, I = 0, D = 0, PID_value = 0;  //pid直道参数
float previous_error = 0;             //误差值

State st = Straight;
uint8_t CountRound=0,dir[7]={0,0,1,0,0,0,0};
int16_t StateFlag=0;
int32_t site,Pid;

//PID_value>0右，<0左
float pid(int error)
{
  P = Kp * error;
  I += error;
  D = Kd *(error - previous_error);
  PID_value = P + I*Ki + D;
  previous_error = error;
  return PID_value;

}

void Repid(){
    P = 0;
    I = 0;
    D = 0;
    PID_value = 0;  //pid直道参数
    previous_error = 0;             //误差值
}

void ReXnji(){
    Repid();
    st = Straight;
    CountRound=0;
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN0);

    Motor_Forward(0,0);
}

uint8_t BackToRoad(uint8_t Data){
    switch(Data){
          case 0xfe://1111 1110
          case 0x7e://0111 1111
          case 0x3f://0011 1111
          case 0xfc://1111 1100
          case 0xfd://1111 1101
          case 0xbf://1011 1111
              return 1;
      }
    return 0;
}



//0右转，1左转
uint8_t outflag(uint8_t PData){
    uint8_t i=0,sum_L=0,sum_R=0;
    int8_t b[8] = {0,0,0,0,0,0,0,0};
    for(i=0;i<8;i++){
    b[i] = (PData>>i)&0x01;
    }
    for(i=0;i<4;i++){
        sum_L+=b[i];
        sum_R+=b[i+4];
        }
    if(sum_L<sum_R){
        return 1;
    }else{
        return 0;
    }
}

void TurnDirection(float time,uint8_t PData){
    uint8_t Data;
    Motor(3075,2900);
    Clock_Delay1ms(60);
    if(Reflectance_Read()==0xff){
        if(time<3.5&&time>2.5){
            CountRound=2;
        };

        if(time>21.5&&time<22.5){
            Motor(1675,-2500);
        }
        else if((dir[CountRound]&&time<8.0)){
            Motor(-2775,1600);
        }else if(outflag(PData)&&time>=8.0){
            Motor(-2775,1600);
        }
        else{
            Motor(1675,-2500);
        }
    while(1){
        Data=Reflectance_Read();
        if(BackToRoad(Data)){
            xunji(Data,0xff,time);
            break;
        }
    }
    }
}

uint8_t xunji(uint8_t Data,uint8_t PData,float time)
{
//    0黑1白
       switch(Data){
                 case 0xf9 ://1111 1001
                 case 0xf1 ://1111 0001
                 case 0xc0 ://1100 0000
                 case 0xfd ://1111 1101
                     st = TurnLeft;
                     break;
                 case 0x9f ://1001 1111
                 case 0x8f ://1000 1111
                 case 0x03 ://0000 0011
                 case 0xbf ://1011 1111
                     st = TurnRight;
                     break;

                 case 0x3f ://0011 1111
                 case 0x1f ://0001 1111
                 case 0x7f ://0111 1111
                 case 0x01 ://0000 0001
                     st = RapidRight;
                     break;

                 case 0xfc ://1111 1100
                 case 0xf8 ://1111 1000
                 case 0xfe ://1111 1110
                 case 0x80 ://1000 0000
                     st = RapidLeft;
                     break;
                 case 0xff:
                     st = out;
                     break;
                 case 0x00:
                     st = stop;
                     break;
                 case 0xdf ://1101 1111
                 case 0xfb ://1111 1011
                 case 0xe7 ://1110 0111
                 case 0xcf ://1100 1111
                 case 0xf3 ://1111 0011
                 case 0xc7 ://1100 0111
                 case 0xe3 ://1110 0011
                 case 0xf7 ://1111 0111
                 case 0xef ://1110 1111
                     st = Straight;
                     break;
                 default:
                     st=other;
                     break;
                  }

       site = Reflectance_Position(Data);
       uint32_t F=2800,R=1800,RR=1700,Offset_L=75,Offset_R=100;
       if(time>13.0&&time<18.0){
           F=3500;
           R=2200;
           RR=2000;
       }else if(time>25.0){
           F=2600;
           R=1700;
           RR=1600;
       }else{
           F=2700;
           R=1700;
           RR=1600;
       }

    switch (st)
    {
        case other:
            {
                Motor(F+Offset_L-500,F-500-Offset_R);
            }
            break;
        case Straight:
            {
                if(site>=-31&&site<=31){
                    Pid = pid(site);
                }
                else{
                    Pid=0;
                }
                    Motor(F+Offset_L+400+Pid/2,F-Offset_R+400-Pid/2);
            }
            break;
        case TurnRight:
            {
                if(site>100)site/=3;
                site*=2;
                Motor(R+Offset_L+site,R-Offset_R-100);
//                Motor_Right(2575,0);
//                Motor_Left(3000,3000);
            }
            break;
        case TurnLeft:
            {
                site=0-site;
                if(site>100)site/=3;
                    site*=2;
                Motor(R+Offset_L-100,R-Offset_R+site);
//                Motor_Left(0,2400);
//                Motor_Right(3000,3000);
            }
            break;
        case RapidRight:
            {
                if(site>300)site/=2;
//                Motor_Left(5000,5000);
                Motor(RR+Offset_L+200+site*2,-(RR-Offset_R-site));
            }
            break;
        case RapidLeft:
            {
                site=0-site;
                if(site>300)site/=2;
//                Motor_Right(5000,5000);
                Motor(-(RR+Offset_L-site),RR-Offset_R+200+site*2);
            }
            break;
        case stop:
            {
                if(time>23.0){
//                Motor_Stop();
                return 1;
                }
            }
            break;
        case out:
            {
                TurnDirection(time,PData);
                CountRound++;
                if(CountRound>=7)
                    CountRound=7;
                return 2;
            }
}
    return 0;
}

//避障
void Avoid(float time){
    uint8_t Data;
    Motor(3000,-1500);
    Clock_Delay1ms(350);
    Motor(2775,2300);
    Clock_Delay1ms(750);
    Motor(1200, 3000);
    Clock_Delay1ms(1500);
    Motor(2875,2300);
        while(1){
            Data = Reflectance_Read();
            if(Data!=0xff){
                xunji(Data,0xff,time);
                break;
            }
        }
}
