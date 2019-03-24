// EE-5314 EMBEDDED MICROCONTROLLER
// 1001558830
// PRIYASHU AJMERA

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include<math.h>
#include <stdio.h>
#include<stdbool.h>
#include<string.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"


//#define DataEnable   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))  // PORT C 6
//#define RED_led      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))  // PORT F 1
//#define GREEN_led    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))  // PORT F 3
//#define BLUE_led     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))  // PORT F 2
//#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))  // PORT F 4
//#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4)))  // PORT B 5
//#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))  // PORT E 5

#define MAX_REtrans 5
#define Max_Packet 8
#define BroadcastAddr 255
#define MAX_CHAR 80
#define MAX_MSG 25
#define MAX_DATA 25
#define cmdSet 0x00
#define cmd_GET  0x20
#define cmd_RESET 0x7F
#define cmd_SA 0x7A
#define cmd_RGB 0x48
#define cmd_POLL_report 0x79
#define cmd_POLL_request 0x78
#define cmd_ACK_report 0x70
#define cmd_DATA_report 0x21
#define ACK_requested 0x80
#define cmd_PULSE 0x02;

uint8_t SOURCE_ADDRESS= 10;
uint8_t field=0;
uint8_t Min_Argcount
uint8_t SIZE;
uint8_t g=0;
uint8_t si;
uint8_t n;
uint8_t RANDOM=0;
uint8_t variable=0;
uint8_t variable1=0;
uint8_t dtimer=0;
uint8_t drtimer=0;
uint8_t p=1,x=0;
uint8_t VALUE[MAX_DATA];
uint8_t checksum;
uint8_t COMMAND;
uint8_t i=0;
uint8_t SEQUENCE_ID=0;
uint8_t count=0;
uint8_t c;

uint16_t ADDRESS;
uint16_t CHANNEL;
uint16_t DestAddr[MAX_MSG];
uint16_t timeout[MAX_MSG];
uint16_t Sequence_Id[MAX_MSG];
uint16_t command[MAX_MSG];
uint16_t channel[MAX_MSG];
uint16_t size[MAX_MSG];
uint16_t ReceivedData[MAX_MSG];
uint16_t CHECKSUM[MAX_MSG];
uint16_t Data[MAX_MSG][MAX_DATA];
uint16_t Retranscount[MAX_MSG];
uint16_t currentphase=0;
uint16_t currentindex=0;
uint16_t checkSum = 0;
uint16_t Rx_Phase=0;
uint16_t G_LEDtimer;
uint16_t R_LEDtimer;
uint16_t comm;
uint16_t  P_LEDtimer=0;
uint16_t Rx_command;

bool CS_Enable;
bool Valid[MAX_MSG];
bool  ACKreq[MAX_MSG];
bool inprogress= false;
bool RAND_Enable=false;
bool ISERROR = false;
bool ACKflag= false;

char str [MAX_CHAR+1];
char position[MAX_CHAR];
char type[MAX_CHAR];
char position[MAX_CHAR];
char type[MAX_CHAR];
char str2[40];
char str3[40];

void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S)
        | SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;;

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE ;

    // Configure LED  pins
    //PE5
    GPIO_PORTE_DIR_R |= 0x20;  // bits 5 is outputs, other pins are inputs
    GPIO_PORTE_DR2R_R |= 0x20; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R |= 0x20;  // enable LEDs and pushbuttons

    //PB5
    GPIO_PORTB_DIR_R |= 0x20;  // bits 5 is outputs, other pins are inputs
    GPIO_PORTB_DR2R_R |= 0x20; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R |= 0x20;  // enable LEDs and pushbuttons


    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0E;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1E;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10;  // pin 4 is input(push Button)


    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // configure Uart - 1 and DEN (PC6) pin
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
    GPIO_PORTC_DIR_R |= 0x40;
    // GPIO_PORTC_DR2R_R |= 0x60;
    GPIO_PORTC_DEN_R |= 0x70;
    GPIO_PORTC_AFSEL_R |= 0x30;
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;

    // Configure UART1 to 38400 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART1_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART1_IBRD_R = 65;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART1_FBRD_R = 7;                               // round(fract(r)*64)=45
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_SPS | UART_LCRH_PEN | UART_LCRH_EPS ; // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    //timer 1
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x61A80;                   // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;

    // Configure PWM module0 to drive RGB backlight
        // RED   on M0PWM3 (PB5), M0PWM1b
        // BLUE  on M0PWM4 (PE4), M0PWM2a
        // GREEN on M0PWM5 (PE5), M0PWM2b
        SYSCTL_RCGCPWM_R |= 0x02;             // turn-on PWM0 module
        __asm(" NOP");                                   // wait 3 clocks
        __asm(" NOP");
        __asm(" NOP");
        SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM0 module
        SYSCTL_SRPWM_R = 0;                              // leave reset state
        PWM1_2_CTL_R = 0;                                // turn-off PWM0 generator 1
        PWM1_3_CTL_R = 0;                                // turn-off PWM0 generator 2
        PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
                                                         // output 3 on PWM0, gen 1b, cmpb
        PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;
                                                         // output 4 on PWM0, gen 2a, cmpa
        PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
                                                         // output 5 on PWM0, gen 2b, cmpb
        PWM1_2_LOAD_R = 255;                            // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
        PWM1_3_LOAD_R = 255;
        PWM1_INVERT_R = PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;
                                                         // invert outputs for duty cycle increases with increasing compare values
        PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
        PWM1_3_CMPB_R = 0;                               // green off
        PWM1_3_CMPA_R = 0;                               // blue off

        PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM0 generator 1
        PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM0 generator 2
        PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                         // enable outputs

        SYSCTL_RCGCEEPROM_R|=0x01;
                    __asm(" NOP");                                   // wait 3 clocks
                    __asm(" NOP");
                    __asm(" NOP");
                    __asm(" NOP");                                   // wait 3 clocks
                    __asm(" NOP");
                    __asm(" NOP");
                    while((EEPROM_EEDONE_R & 0x01));
                    if((EEPROM_EESUPP_R & 0x08==0x08)||(EEPROM_EESUPP_R &0x04 ==0x04))
                    {
                        ISERROR=true;
                        return;
                    }
                    else
                    {
                        ISERROR=false;
                    }
}

//put char blocking function
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

void putcUart1(uint8_t c)
{
    //while (UART1_FR_R & UART_FR_TXFF);
    UART1_DR_R = c;
}

//Send char string
void putsUart0(char* str)
{
    uint8_t i;
    for (i=0;i<strlen(str);i++)
    {
        putcUart0(str[i]);
    }
}

//Receive char blocking function
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}

//Receive char blocking function
uint16_t getcUart1()
{
    while (UART1_FR_R & UART_FR_RXFE);
    return UART1_DR_R & 0xFF;
}

//Get command from string
char* getstring(uint8_t field)
{
    return &str[position[field]];
}

//Get numbeer from inserted string
uint16_t getnumber (uint8_t field)
{
    return atoi(&str[position[field]]);
}

uint8_t calculateCheckSum(uint8_t index)
{
    uint16_t checkSum = 0;
    uint8_t i;
    checkSum = DestAddr[index] + SOURCE_ADDRESS + Sequence_Id[index] + command[index] + channel[index] + size[index];
    for(i = 0; i < size[index]; i++)
    {
        checkSum = checkSum + Data[index][i];
    }
    checkSum = checkSum  % 256;
    checkSum = ~checkSum;
    return checkSum;
}

char* Lower2Upper(char *string)
{
    uint16_t i = 0;
    while (string[i] != '\0')
    {
        if (string[i] >= 'a' && string[i] <= 'z')
        {
            string[i] = string[i] - 32;
        }
        i++;
    }
    return string;
}

bool iscommand(char comand[], uint8_t minArgs)
{
    char *stringstr;
    stringstr = &str[position[0]];
    if((strcmp(Lower2Upper(stringstr),comand)== 0) && (field > minArgs))
    {
        return true;
    }
    return false;
}




//function that will send a packet of data
void sendpacket()
{
    for(i=0;i<MAX_MSG;i++)
    {
        if(i!=true)
        {
            DestAddr[i]=ADDRESS;
            Sequence_Id[i]=SEQUENCE_ID;
            SEQUENCE_ID++;
            channel[i]=CHANNEL;
            size[i]=SIZE;

            for(x=0;x<size[i];x++)
            {
                Data[i][x] = VALUE[x];
            }
            ACKreq[i]= ACKflag;
            if(ACKreq[i])
            {
                command[i] = comm | 0x80;
                timeout[i] = 200;
            }
            else
            {
                command[i] = comm;
                timeout[i] = 00;
            }
            CHECKSUM[i]=calculateCheckSum(i);
            Retranscount[i]=0;
            Valid[i]=true;
            break;
        }
        else
        {
            putsUart0("\n\rERROR\n");
        }
    }
}

// Get string from user
void get_string()
{
    while(count<MAX_CHAR)
    {
        c = getcUart0();
        putcUart0(c);
        if(c=='\b')
        {
            if(count>0)
            count-=1;
            else
            {
                putsUart0("\n\rERROR\n\n");
                putsUart0("\n\rInvalid Command\n\n");
            }
        }

        if(c=='\r')
        {
            str[count++]='\0';
            break;
        }
        if(c>=32)
        {
            str[count++]=c;
            if(count>MAX_CHAR)
            {
                str[count++]='\0';
                break;
            }
        }

    }
}


void init_table()
{
    uint8_t i;
    for(i=0;i<MAX_MSG;i++)
    {
        Valid[i]=false;
    }
}

void codeNdiscard()
{
    for(p=0;str[p]!='\0';p++)
    {
        if((str[p]>='a' && str[p]<='z') || (str[p]>='A' && str[p]<='Z')||(str[p]>='0' && str[p]<='9'))
        {
         if((str[p]>='a' && str[p]<='z') || (str[p]>='A' && str[p]<='Z'))
         {
             field+=1;
             type[x]='a';
             x++;
             position[i]=p;
             i++;
             while((str[p]>='a' && str[p]<='z') || (str[p]>='A' && str[p]<='Z'))
             {
                 p++;
             }p-=1;
         }
         else
         {
         field+=1;
         type[x]='n';
         x++;
         position[i]=p;
         i++;
         while((str[p]>='0' && str[p]<='9'))
         {
             p++;
         }
       }
    }
       else
       {
           str[p]='\0';
       }
    }
    for(i=0;i<field;i++)
      {
          if(type[i]=='a')
          {
               sprintf(str3,"%u,alpha,%s\n",i,&str[position[i]]);
          }
          else
          {
              sprintf(str3,"%u,number,%s\n",i,&str[position[i]]);
          }
          putsUart0(str3);
      }

}

//Set 9th bit
void SetBit()
{
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN|UART_LCRH_PEN|UART_LCRH_SPS;
}

//Clear 9th bit
void ClearBit()
{
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN|UART_LCRH_EPS|UART_LCRH_PEN|UART_LCRH_SPS;
}

void tx9()
{
    switch(currentphase)
    {
    case 0:
        if (!(UART1_FR_R & UART_FR_BUSY))
            {
                SetBit();
                putcUart1(DestAddr[currentindex]);
                currentphase++;

                //TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
            }
            break;
    case 1:
        if (!(UART1_FR_R & UART_FR_BUSY))
            {
                ClearBit();
                putcUart1(SOURCE_ADDRESS);
                currentphase++;

            }
        break;

    case 2:


            putcUart1(Sequence_Id[currentindex]);
            currentphase++;
            break;

    case 3:
            putcUart1(command[currentindex]);
            currentphase++;
            break;

    case 4:
            putcUart1(channel[currentindex]);
            currentphase++;
            break;

    case 5:
            putcUart1(size[currentindex]);
            currentphase++;
            g=size[currentindex];
            break;

    case 6:
            if(size[currentindex]!=0)
            {
                uint8_t z;
                for(z=0;z<size[currentindex];z++)
                {
                    putcUart1(Data[currentindex][z]);
                }
            }
            currentphase++;
            break;

    case 7:
            putcUart1(CHECKSUM[currentindex]);
            currentphase++;
            dtimer=20;
            break;

    }
    // RED_LED = 1;
    //    waitMicrosecond(50000);
    //    RED_LED = 0;
    //    waitMicrosecond(50000);
}


bool cmdcheck()
{
    bool error=true;
    /*if((field>6)||(field==1))
    {
        putsUart0("\n\rInvalid Command\n\r");
    }*/
    if(field==4)
        {
            if(iscommand("SET",3))
            {
                //uint8_t val=0;
                SIZE=1;
                comm= cmdSet;
                putsUart0("\n\rCommand entered is SET\n");
                ADDRESS= getnumber(1);
                //putsUart0("\rADDRESS is\t");
                //sprintf(str2,"%u",ADDRESS);
                //putsUart0(str2);
                CHANNEL= getnumber(2);
                //putsUart0("\n\rCHANNEL is\t");
                //sprintf(str2,"%u",CHANNEL);
                //putsUart0(str2);
                VALUE[0] = getnumber(3);
                //val= VALUE[0];
                //putsUart0("\n\rVALUE is\t");
                //sprintf(str2,"%u",val);
                //putsUart0(str2);
                sendpacket( );
                error=false;
            }
        }
        if(field==1)
        {
            if(iscommand("POLL",0))
            {
                putsUart0("\n\rPolling is requested\n");
                ADDRESS= BroadcastAddr;
                comm=cmd_POLL_request;
                SIZE=0;
                VALUE[0]=0;
                error=false;
                sendpacket();
            }
        }

        if(field==6)
        {
            if(iscommand("RGB",5))
            {
                putsUart0("\n\rCommand entered is RGB\n");
                SIZE=3;
                comm=cmd_RGB;
                ADDRESS=getnumber(1);
                CHANNEL=getnumber(2);
                VALUE[0]=getnumber(3);
                VALUE[1]=getnumber(4);
                VALUE[2]=getnumber(5);
                sendpacket();
                error=false;
            }
            if(iscommand("PULSE",5))
                     {
                         putsUart0("\n\rEntered command is PULSE\n");
                         comm=cmd_PULSE;
                         SIZE=3;
                         ADDRESS=getnumber(1);
                         CHANNEL=getnumber(2);
                         VALUE[0]=getnumber(3);
                         VALUE[1]=getnumber(4);
                         VALUE[2]=getnumber(5);
                         sendpacket();
                         error=false;
                     }

        }
        if(field==3)
        {
            if(iscommand("GET",2))
            {
                SIZE=0;
                comm= cmd_GET;
                putsUart0("\n\rCommand entered is GET\n");
                ADDRESS=getnumber(1);
                CHANNEL=getnumber(2);
                VALUE[0]=0;
                sendpacket();
                error=false;
            }
            if(iscommand("SA",2))
            {
                SIZE=1;
                comm=cmd_SA;
                putsUart0("\n\rCommand entered is Set Address\n");
                ADDRESS=getnumber(1);
                VALUE[0]=getnumber(2);
                sendpacket();
                error=false;
            }
        }
        if(field==2)
        {
            if(iscommand("RESET",1))
            {
                SIZE=0;
                comm=cmd_RESET;
                putsUart0("\n\rCommand entered is RESET\n");
                ADDRESS=getnumber(1);
                CHANNEL=0;
                VALUE[0]=0;
                sendpacket();
                error=false;
            }
            if (iscommand("ACK",1))
            {
                putsUart0("\n\rCommand entered is ACK\n");
                //valid=false;
                if((strcmp(getstring(1),"ON")==0)||(strcmp(getstring(1),"on")==0))
                {
                    error=false;
                    putsUart0("\n\rAcknoledgement turned ON");
                    return ACKflag = true;

                }
                if((strcmp(getstring(1),"OFF")==0)||(strcmp(getstring(1),"off")==0))
                {
                    error=false;
                    putsUart0("\n\rrAcknoledgement turned OFF\n");
                    return ACKflag = false;
                }
            }
            if(iscommand("RANDOM",1))
            {
                putsUart0("\n\r Command entered is RANDOM\n");
                if((strcmp(getstring(1),"ON")==0)||(strcmp(getstring(1),"on")==0))
                {
                    error=false;
                    putsUart0("\n\rRandomizer turned ON\n");
                    return RAND_Enable=true;
                }
                if((strcmp(getstring(1),"OFF")==0)||(strcmp(getstring(1),"off")==0))
                {
                    error=false;
                    putsUart0("\n\rRandomizer turned OFF\n");
                    return RAND_Enable=false;
                }
            }
            if (iscommand("CS",1))
            {
                putsUart0("\n\rCommand entered is CS\n");
                if((strcmp(getstring(1),"ON")==0)||(strcmp(getstring(1),"on")==0))
                {
                    putsUart0("\n\rCarrier Sense turned ON\n");
                    error=false;
                    return CS_Enable= true;
                }
                if((strcmp(getstring(1),"OFF")==0)||(strcmp(getstring(1),"off")==0))
                {
                    putsUart0("\n\rCarrier Sense turned OFF\n");
                    error=false;
                    return CS_Enable= false;
                }
            }
        }

        if(error==true)
        {
            putsUart0("\n\rInvalid Command\n");
        }
        return error;
}

void Tx_Data()
{
    if(!inprogress)
    {
        uint16_t foundindex;
        for(foundindex = 0; foundindex < 25; foundindex++)
        {
            if(Valid[foundindex] && timeout[i]==0)
            {
                inprogress = true;
                g=0;
                currentindex = foundindex;
                currentphase = 0;
               // tx9();
                break;
            }

        }
    }
    if(inprogress)
    {
        GPIO_PORTC_DATA_R|=0x40;
        //DataEnable = 1;

        while ((UART1_FR_R & UART_FR_TXFF) == 0 && size[currentindex]+7 > currentphase+g-1)
        {
       // if (!(UART1_FR_R & UART_FR_BUSY))
         //transmit data when buffer is not full

            if(CS_Enable==true)
            {
                if(Rx_Phase==0)
                {
                    tx9();
                }
            }                                                   //CHANGE
            if(CS_Enable==false)
            {
                tx9();
            }

        }
        if(size[currentindex]+7 == currentphase + g-1)
        {
                GPIO_PORTB_DATA_R|=0x20;
                //RED_LED = 1;
                R_LEDtimer = 20;
                while ((UART1_FR_R & UART_FR_BUSY));
                GPIO_PORTC_DATA_R &=~0x40;
                //DataEnable = 0;
                if(!ACKreq[currentindex])
                {
                    Valid[currentindex] = false;
                    inprogress = false;
                }
                else
                {
                    if(Retranscount[currentindex] >= MAX_REtrans)
                    {
                        Valid[currentindex] = false;
                        inprogress = false;
                        //DataEnable = 0;
                    }
                    else
                    {
                        //calculate and store retransmit time out.
                        Retranscount[currentindex]++;
                        if(RAND_Enable==false)
                        {
                            timeout[currentindex] = 200;
                        }                                              //CHANGE
                        if(RAND_Enable==true)
                        {
                            srand(SOURCE_ADDRESS);
                            n=8;
                            RANDOM=pow(2,n)*50*(rand() % 54);
                            sprintf(str2,"%u", RANDOM);
                            putsUart0(str2);
                            timeout[currentindex]=RANDOM;
                        }
                        inprogress = false;
                    }

                }
            }

    }

}
void analog()
{
    GPIO_PORTF_AFSEL_R |= 0x0E;
    GPIO_PORTF_PCTL_R|= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7 ;
}

void digital()
{
    GPIO_PORTF_AFSEL_R &= 0x00;
    GPIO_PORTF_PCTL_R = 0;
}

setRgbColor(uint16_t red, uint16_t green, uint16_t blue)
{
    PWM1_2_CMPB_R = red;
    PWM1_3_CMPA_R = blue;
    PWM1_3_CMPB_R = green;
}

void process_msg()
{
    drtimer=20;
    //uint16_t mul=0;
    uint16_t ca;
    uint16_t CAL;
    uint16_t CA;

    uint16_t Rx_channel;
    uint16_t Rx_data;
    uint16_t Rx_data1;
    uint16_t Rx_data2;
    uint16_t Rx_destaddr;
    uint16_t Rx_seqid;
    uint16_t Req_Data;
    uint8_t Rx_checksum;
    uint8_t Rx_srcaddr;
    uint8_t Rx_size;
    Rx_data1=ReceivedData[7];
           Rx_data2=ReceivedData[8];
    Rx_srcaddr=ReceivedData[0];
    putsUart0("\n\rDestination Address is\t");
    sprintf(str2,"%u",Rx_srcaddr);
    putsUart0(str2);
    Rx_destaddr=ReceivedData[1];
    putsUart0("\n\rSource Address is\t");
    sprintf(str2,"%u",Rx_destaddr);
    putsUart0(str2);
    Rx_seqid=ReceivedData[2];
    putsUart0("\n\rSequence Id is\t");
    sprintf(str2,"%u", Rx_seqid);
    putsUart0(str2);
    Rx_command=ReceivedData[3];
    putsUart0("\n\rCommand is\t");
    sprintf(str2,"%u", Rx_command);
    putsUart0(str2);
    Rx_channel=ReceivedData[4];
    putsUart0("\n\rChannel is\n\r");
    sprintf(str2,"%u", Rx_channel);
    putsUart0(str2);
    Rx_size=ReceivedData[5];
    putsUart0("\n\rSize is\t");
    sprintf(str2,"%u", Rx_size);
    putsUart0(str2);
    Rx_data=ReceivedData[6];
    putsUart0("\n\rData is\t");
    sprintf(str2,"%u", Rx_data);
    putsUart0(str2);
    Rx_checksum=ReceivedData[Max_Packet+si-2];
    putsUart0("\n\rChecksum is\t");
    sprintf(str2,"%u", Rx_checksum);
    putsUart0(str2);
    ca=    Rx_command+ Rx_channel+ Rx_destaddr+ Rx_seqid+ Rx_srcaddr+ Rx_size;
    if(Rx_command==cmd_RGB)
    {
        ca+=Rx_data1+Rx_data2+Rx_data;
    }
    if((Rx_command==cmdSet)||(Rx_command==cmd_DATA_report)||(Rx_command==0x80)||(Rx_command==cmd_POLL_report))
    {
        ca+=Rx_data;
    }
    CA= ca%256;
    CAL=~CA;
    if(Rx_checksum!=(uint8_t )CAL)
    {
        putsUart0("\n\rChecksum Error\n");
        GPIO_PORTE_DATA_R|=0x20;
        //GPIO_PORTB_DATA_R|=0x20;
        //RED_LED=1;
    }
    digital();
    if((Rx_command&0x7F)==0x02)
    {
        analog();
        setRgbColor(Rx_data,0,0);
        putsUart0("\n\rReceived PULSE command\n");
        P_LEDtimer=Rx_data1*Rx_data2;
        //R_LEDtimer = 20;
        //for(i=0;i<mul;i++);
        //setRgbColor(0,0,0);
    }

    if((Rx_command & 0x80) == ACK_requested)
    {
        for(i=0;i<MAX_MSG;i++)
        {
            if(!Valid[i])
            {
                DestAddr[i]=Rx_destaddr;
                Sequence_Id[i]=SEQUENCE_ID;
                SEQUENCE_ID++;
                channel[i]= Rx_channel;
                size[i]=1;
                for(x=0;x<size[i];x++)
                {
                    Data[i][x] = Rx_seqid;
                }
                //ACKreq[i]= ACKflag;
                command[i]=cmd_ACK_report;
                CHECKSUM[i]=calculateCheckSum(i);
                //Retranscount[i]=0;
                Valid[i]=true;
                break;
            }
        }
    }
    /*if((Rx_command&0x7F)==cmd_DATA_report)
    {
        Valid[i]=false;
        putsUart0("\n\rDATA ON CORR\n");
        sprintf(str2,"%u",Rx_data);
        putsUart0(str2);
    }*/
   if((Rx_command&0x7F)==cmd_RGB)
    {

        if(Rx_channel==5)
        {
            analog();
            setRgbColor(Rx_data,Rx_data1,Rx_data2);


            /*if(Rx_data==1)
            {
                GPIO_PORTF_DATA_R|=0x02;
                //RED_led=1;
            }
            else
            {
                GPIO_PORTF_DATA_R&=~0x01;
                //RED_led=0;
            }
            if(Rx_data1==1)
            {
                GPIO_PORTF_DATA_R|=0x08;
                //GREEN_led=1;
            }
            else
            {
                GPIO_PORTF_DATA_R &= ~0x08;
                //GREEN_led=0;
            }
            if(Rx_data2==1)
            {
                GPIO_PORTF_DATA_R |= 0x04;
                //BLUE_led=1;
            }
            else
            {
                GPIO_PORTF_DATA_R &= ~0x04;
                //BLUE_led=0;
            }*/
        }
    }
   if((Rx_command & 0x7F)==cmd_POLL_request)
       {
           putsUart0("\n\rReceived Polling request\n");
           for(i=0;i<MAX_MSG;i++)
           {
               if(!Valid[i])
               {
                   DestAddr[i]=Rx_destaddr;
                   Sequence_Id[i]=SEQUENCE_ID;
                   SEQUENCE_ID++;
                   channel[i]= 0;
                   size[i]=1;
                   for(x=0;x<size[i];x++)
                   {
                       Data[i][x] = SOURCE_ADDRESS;
                   }
                   //ACKreq[i]= ACKflag;
                   command[i]=cmd_POLL_report;
                   CHECKSUM[i]=calculateCheckSum(i);
                   //Retranscount[i]=0;
                   Valid[i]=true;
                   break;
               }
           }
       }

       if((Rx_command & 0x7F)==cmd_DATA_report)
       {
           putsUart0("\n\rReceived a data report command\n");
           Valid[i]=false;
           putsUart0("\n\rData on the corresponding channel is\n\r");
           sprintf(str2,"%u",Rx_data);
           putsUart0(str2);
       }
       if((Rx_command & 0x7F)==cmd_POLL_report)
       {
           putsUart0("\n\r Received POLLING REPORT from node\n");
           sprintf(str2,"%u",Rx_data);
           putsUart0(str2);
       }
       if((Rx_command&0x7F)==cmd_ACK_report)
       {
           Valid[i]=false;
           putsUart0("\n\r Received ACKNOLEDGEMENT REPORT\n");
           //sprintf(str2,"%u",Rx_data);
           //putsUart0(str2)
           //putsUart0("\n\rSuccessfully received Ack\n");
       }

    if((Rx_command&0x7F)==cmd_SA)
    {
        SOURCE_ADDRESS= Rx_data;
        putsUart0("\n\r NEW ADDRESS OF THE NODE IS\t");
        EEPROM_EERDWR_R = SOURCE_ADDRESS;
        sprintf(str2,"%u",Rx_data);
        putsUart0(str2);
    }
    if((Rx_command&0x7F)==cmd_RESET)
    {
        putsUart0("\n\rRECEIVED RESET COMMAND\n");
        //NVIC_APINT_R |=0xFA050004;
        //NVIC_APINT_R |=4;
        ResetISR();
    }
    if((Rx_command&0x7F)==cmd_GET)
    {
        putsUart0("\n\rReceived GET command\n");
        if(Rx_channel==2)
        {
            putsUart0("\n\rProcessing Request\n");
            if((GPIO_PORTF_DATA_R & 0x10)==0x10)
            {
                Req_Data=1;
            }
            if((GPIO_PORTF_DATA_R & 0x10)==~0x10)
            {
                Req_Data=0;
            }
            for(i=0;i<MAX_MSG;i++)
            {
                if(!Valid[i])
                {
                    DestAddr[i]=Rx_destaddr;
                    Sequence_Id[i]=SEQUENCE_ID;
                    SEQUENCE_ID++;
                    channel[i]= Rx_channel;
                    size[i]=1;
                    for(x=0;x<size[i];x++)
                    {
                        Data[i][x] = Req_Data;
                    }
                    //ACKreq[i]= ACKflag;
                    command[i]=0x21;
                    CHECKSUM[i]=calculateCheckSum(i);
                    //Retranscount[i]=0;
                    Valid[i]=true;
                    break;
                }
            }
        }

    }
    if((Rx_command&0x7F)==cmdSet)
    {
        putsUart0("\n\rReceived SET command\n");
        if(Rx_channel==1)
        {
            putsUart0("\n\rCHANNEL 1 SELECTED\n");
            if(Rx_data==1)
            {
                putsUart0("\n\rBLUE LED 'ON'\n");
                GPIO_PORTF_DATA_R |= 0x04;
                //BLUE_led=1;
            }
            if(Rx_data==0)
            {
                putsUart0("\n\rBLUE LED 'OFF'\n");
                GPIO_PORTF_DATA_R &=  ~0x04;
                //BLUE_led=0;
            }
        }
    }
}

void Rx_Data()
{
    uint16_t Rec_Data;
    GPIO_PORTC_DATA_R&=~0x40;
    //DataEnable = 0;
    Rx_Phase=0;
    SetBit();
    while(Rx_Phase < Max_Packet+si-1 && !inprogress && (UART1_FR_R & UART_FR_RXFE) == 0)
    { //flip flop should not be empty
        Rec_Data = UART1_DR_R & 0x2FF;
        if(Rx_Phase != 0)
        {
            ReceivedData[Rx_Phase] = Rec_Data & 0xFF;
            if(Rx_Phase==5)
            {
                si=ReceivedData[Rx_Phase];
            }
            Rx_Phase++;
        }
        if(((Rec_Data & 0x200) == 0))
        {
            if(((Rec_Data & 0x0FF) == SOURCE_ADDRESS) || ((Rec_Data & 0x0FF) == BroadcastAddr) )
            {
                Rx_Phase = 0;
                ReceivedData[Rx_Phase] = Rec_Data & 0x0FF;
                Rx_Phase++;
            }
        }
        if(Rx_Phase == Max_Packet+si-1)
        {
            GPIO_PORTE_DATA_R|=0x20;
            //GREEN_LED = 1;
            G_LEDtimer = 20;
            process_msg();
        }
    }
}




void Decrement_Timeout()
{
    uint8_t i;
    for(i = 0; i < 25; i++)
    {
        if(timeout[i] > 0)
        {
            timeout[i]--;
        }
    }
}
void pulseLed_Timeout()
{
    if(P_LEDtimer > 0)
    {
        P_LEDtimer--;
    }
    if(P_LEDtimer == 0 &&((Rx_command&0x7F)==0x02))
    {
        setRgbColor(0,0,0);
        //GPIO_PORTB_DATA_R&=~0x20;
        //RED_LED = 0;
    }
}
void RLed_Timeout()
{
    if(R_LEDtimer > 0)
    {
        R_LEDtimer--;
    }
    if(R_LEDtimer == 0)
    {
        GPIO_PORTB_DATA_R&=~0x20;
        //RED_LED = 0;
    }
}

void GLed_Timeout()
{
    if(G_LEDtimer > 0)
    {
        G_LEDtimer--;
    }
    if(G_LEDtimer == 0)
    {
        GPIO_PORTE_DATA_R&=~0x20;
        //GREEN_LED = 0;
    }
}


void Clear_Str()
{
    for(i=0;i<MAX_CHAR;i++)
    {
        str[i]='\0';
        position[i]='\0';
        type[i]='\0';
    }
}


void Rdeadlock()
{
    {
        if(Rx_Phase!=0)
        {
            if(variable1==Rx_Phase)
            {
                if(drtimer>0)
                     {
                         drtimer--;
                     }
            }

            if(drtimer==0 )      //CHANGE
            {
                Rx_Phase=0;
            } variable1=Rx_Phase;
        }
    }
}
void deadlock()
{
    if(currentphase!=0)
    {
        if(variable==currentphase)
        {
            if(dtimer>0)
                 {
                     dtimer--;
                 }
        }
        if(dtimer==0 )      //CHANGE
        {
            currentphase=0;
        } variable=currentphase;
    }
}
void Timer1Isr()
{
   // GREEN_LED ^= 1;
    //transmission
    //GREEN_LED=1;
    Tx_Data();
    //receiving
    Rx_Data();
    //timeout in ack
    Decrement_Timeout();
    RLed_Timeout();
    GLed_Timeout();
    pulseLed_Timeout();
    deadlock();
    Rdeadlock();
    //if (!(UART1_FR_R & UART_FR_BUSY))
   // DataEnable = 0;
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;            // clear interrupt flag
}
   /* void Timer1Isr()
    {
        //if (!timeMode)
        //{
                                      // set update flag
            GREEN_LED ^= 1;                              // status
        //}
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
    }*/

void waitMicrosecond(uint32_t us)
{

    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");
}

void In_Str()
{
    count=0;
    field=0;
    x=0;
    i=0;
}

int main(void)
{
    initHw();
    if(ISERROR){
        putsUart0("error\r\n");
    }
    EEPROM_EEBLOCK_R = 0x10;
    EEPROM_EEOFFSET_R = 0x10;
    uint32_t address = EEPROM_EERDWR_R;
    if(address != 0xFFFFFFFF){
        SOURCE_ADDRESS = (uint8_t)address;
    }
    GPIO_PORTF_DATA_R|=0x08;
    //GREEN_led=1;
    waitMicrosecond(500000);
    GPIO_PORTF_DATA_R&=~0x08;
    //GREEN_led=0;

    init_table();
    while(1)
    {

        //while(1)
        putsUart0("\n\rREADY");
        putsUart0("\n\rEnter Command...\n\r");
        get_string();
        codeNdiscard();
        cmdcheck();
        Clear_Str();
        In_Str();
   }
}