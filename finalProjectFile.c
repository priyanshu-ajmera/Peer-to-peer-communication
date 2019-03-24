{\rtf1\ansi\ansicpg1252\deff0\nouicompat\deflang1033{\fonttbl{\f0\fnil\fcharset0 Courier New;}}
{\*\generator Riched20 10.0.17134}\viewkind4\uc1 
\pard\sl240\slmult1\f0\fs22 // EE-5314 EMBEDDED MICROCONTROLLER\par
// 1001558830\par
// PRIYASHU AJMERA\par
\par
//-----------------------------------------------------------------------------\par
// Hardware Target\par
//-----------------------------------------------------------------------------\par
\par
// Target Platform: EK-TM4C123GXL Evaluation Board\par
// Target uC:       TM4C123GH6PM\par
// System Clock:    40 MHz\par
\par
// Hardware configuration:\par
// Red LED:\par
//   PF1 drives an NPN transistor that powers the red LED\par
\par
//-----------------------------------------------------------------------------\par
// Device includes, defines, and assembler directives\par
//-----------------------------------------------------------------------------\par
\lang9\par
#include <stdint.h>\par
#include<math.h>\par
#include <stdio.h>\par
#include<stdbool.h>\par
#include<string.h>\par
#include <stdlib.h>\par
#include "tm4c123gh6pm.h"\par
\par
\par
//#define DataEnable   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))  // PORT C 6\par
//#define RED_led      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))  // PORT F 1\par
//#define GREEN_led    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))  // PORT F 3\par
//#define BLUE_led     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))  // PORT F 2\par
//#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))  // PORT F 4\par
//#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4)))  // PORT B 5\par
//#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))  // PORT E 5\par
\par
#define MAX_REtrans 5\par
#define Max_Packet 8\par
#define BroadcastAddr 255\par
#define MAX_CHAR 80\par
#define MAX_MSG 25\par
#define MAX_DATA 25\par
#define cmdSet 0x00\par
#define cmd_GET  0x20\par
#define cmd_RESET 0x7F\par
#define cmd_SA 0x7A\par
#define cmd_RGB 0x48\par
#define cmd_POLL_report 0x79\par
#define cmd_POLL_request 0x78\par
#define cmd_ACK_report 0x70\par
#define cmd_DATA_report 0x21\par
#define ACK_requested 0x80\par
#define cmd_PULSE 0x02;\par
\par
uint8_t SOURCE_ADDRESS= 10;\par
uint8_t field=0;\par
uint8_t Min_Argcount\par
uint8_t SIZE;\par
uint8_t g=0;\par
uint8_t si;\par
uint8_t n;\par
uint8_t RANDOM=0;\par
uint8_t variable=0;\par
uint8_t variable1=0;\par
uint8_t dtimer=0;\par
uint8_t drtimer=0;\par
uint8_t p=1,x=0;\par
uint8_t VALUE[MAX_DATA];\par
uint8_t checksum;\par
uint8_t COMMAND;\par
uint8_t i=0;\par
uint8_t SEQUENCE_ID=0;\par
uint8_t count=0;\par
uint8_t c;\par
\par
uint16_t ADDRESS;\par
uint16_t CHANNEL;\par
uint16_t DestAddr[MAX_MSG];\par
uint16_t timeout[MAX_MSG];\par
uint16_t Sequence_Id[MAX_MSG];\par
uint16_t command[MAX_MSG];\par
uint16_t channel[MAX_MSG];\par
uint16_t size[MAX_MSG];\par
uint16_t ReceivedData[MAX_MSG];\par
uint16_t CHECKSUM[MAX_MSG];\par
uint16_t Data[MAX_MSG][MAX_DATA];\par
uint16_t Retranscount[MAX_MSG];\par
uint16_t currentphase=0;\par
uint16_t currentindex=0;\par
uint16_t checkSum = 0;\par
uint16_t Rx_Phase=0;\par
uint16_t G_LEDtimer;\par
uint16_t R_LEDtimer;\par
uint16_t comm;\par
uint16_t  P_LEDtimer=0;\par
uint16_t Rx_command;\par
\par
bool CS_Enable;\par
bool Valid[MAX_MSG];\par
bool  ACKreq[MAX_MSG];\par
bool inprogress= false;\par
bool RAND_Enable=false;\par
bool ISERROR = false;\par
bool ACKflag= false;\par
\par
char str [MAX_CHAR+1];\par
char position[MAX_CHAR];\par
char type[MAX_CHAR];\par
char position[MAX_CHAR];\par
char type[MAX_CHAR];\par
char str2[40];\par
char str3[40];\par
\par
void initHw()\par
\{\par
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz\par
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S)\par
        | SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;;\par
\par
    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)\par
    // Note UART on port A must use APB\par
    SYSCTL_GPIOHBCTL_R = 0;\par
\par
    // Enable GPIO port A and F peripherals\par
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOE ;\par
\par
    // Configure LED  pins\par
    //PE5\par
    GPIO_PORTE_DIR_R |= 0x20;  // bits 5 is outputs, other pins are inputs\par
    GPIO_PORTE_DR2R_R |= 0x20; // set drive strength to 2mA (not needed since default configuration -- for clarity)\par
    GPIO_PORTE_DEN_R |= 0x20;  // enable LEDs and pushbuttons\par
\par
    //PB5\par
    GPIO_PORTB_DIR_R |= 0x20;  // bits 5 is outputs, other pins are inputs\par
    GPIO_PORTB_DR2R_R |= 0x20; // set drive strength to 2mA (not needed since default configuration -- for clarity)\par
    GPIO_PORTB_DEN_R |= 0x20;  // enable LEDs and pushbuttons\par
\par
\par
    // Configure LED and pushbutton pins\par
    GPIO_PORTF_DIR_R = 0x0E;  // bits 1 and 3 are outputs, other pins are inputs\par
    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)\par
    GPIO_PORTF_DEN_R = 0x1E;  // enable LEDs and pushbuttons\par
    GPIO_PORTF_PUR_R = 0x10;  // pin 4 is input(push Button)\par
\par
\par
    // Configure UART0 pins\par
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status\par
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity\par
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity\par
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;\par
\par
    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)\par
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming\par
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)\par
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16\par
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45\par
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO\par
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module\par
\par
    // configure Uart - 1 and DEN (PC6) pin\par
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;\par
    GPIO_PORTC_DIR_R |= 0x40;\par
    // GPIO_PORTC_DR2R_R |= 0x60;\par
    GPIO_PORTC_DEN_R |= 0x70;\par
    GPIO_PORTC_AFSEL_R |= 0x30;\par
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;\par
\par
    // Configure UART1 to 38400 baud, 8N1 format (must be 3 clocks from clock enable and config writes)\par
    UART1_CTL_R = 0;                                 // turn-off UART0 to allow safe programming\par
    UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)\par
    UART1_IBRD_R = 65;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16\par
    UART1_FBRD_R = 7;                               // round(fract(r)*64)=45\par
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_SPS | UART_LCRH_PEN | UART_LCRH_EPS ; // configure for 8N1 w/ 16-level FIFO\par
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module\par
\par
    //timer 1\par
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer\par
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring\par
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)\par
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)\par
    TIMER1_TAILR_R = 0x61A80;                   // set load value to 40e6 for 1 Hz interrupt rate\par
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts\par
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)\par
    TIMER1_CTL_R |= TIMER_CTL_TAEN;\par
\par
    // Configure PWM module0 to drive RGB backlight\par
        // RED   on M0PWM3 (PB5), M0PWM1b\par
        // BLUE  on M0PWM4 (PE4), M0PWM2a\par
        // GREEN on M0PWM5 (PE5), M0PWM2b\par
        SYSCTL_RCGCPWM_R |= 0x02;             // turn-on PWM0 module\par
        __asm(" NOP");                                   // wait 3 clocks\par
        __asm(" NOP");\par
        __asm(" NOP");\par
        SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM0 module\par
        SYSCTL_SRPWM_R = 0;                              // leave reset state\par
        PWM1_2_CTL_R = 0;                                // turn-off PWM0 generator 1\par
        PWM1_3_CTL_R = 0;                                // turn-off PWM0 generator 2\par
        PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;\par
                                                         // output 3 on PWM0, gen 1b, cmpb\par
        PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;\par
                                                         // output 4 on PWM0, gen 2a, cmpa\par
        PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;\par
                                                         // output 5 on PWM0, gen 2b, cmpb\par
        PWM1_2_LOAD_R = 255;                            // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz\par
        PWM1_3_LOAD_R = 255;\par
        PWM1_INVERT_R = PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;\par
                                                         // invert outputs for duty cycle increases with increasing compare values\par
        PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)\par
        PWM1_3_CMPB_R = 0;                               // green off\par
        PWM1_3_CMPA_R = 0;                               // blue off\par
\par
        PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM0 generator 1\par
        PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM0 generator 2\par
        PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;\par
                                                         // enable outputs\par
\par
        SYSCTL_RCGCEEPROM_R|=0x01;\par
                    __asm(" NOP");                                   // wait 3 clocks\par
                    __asm(" NOP");\par
                    __asm(" NOP");\par
                    __asm(" NOP");                                   // wait 3 clocks\par
                    __asm(" NOP");\par
                    __asm(" NOP");\par
                    while((EEPROM_EEDONE_R & 0x01));\par
                    if((EEPROM_EESUPP_R & 0x08==0x08)||(EEPROM_EESUPP_R &0x04 ==0x04))\par
                    \{\par
                        ISERROR=true;\par
                        return;\par
                    \}\par
                    else\par
                    \{\par
                        ISERROR=false;\par
                    \}\par
\}\par
\par
//put char blocking function\par
void putcUart0(char c)\par
\{\par
    while (UART0_FR_R & UART_FR_TXFF);\par
    UART0_DR_R = c;\par
\}\par
\par
void putcUart1(uint8_t c)\par
\{\par
    //while (UART1_FR_R & UART_FR_TXFF);\par
    UART1_DR_R = c;\par
\}\par
\par
//Send char string\par
void putsUart0(char* str)\par
\{\par
    uint8_t i;\par
    for (i=0;i<strlen(str);i++)\par
    \{\par
        putcUart0(str[i]);\par
    \}\par
\}\par
\par
//Receive char blocking function\par
char getcUart0()\par
\{\par
    while (UART0_FR_R & UART_FR_RXFE);\par
    return UART0_DR_R & 0xFF;\par
\}\par
\par
//Receive char blocking function\par
uint16_t getcUart1()\par
\{\par
    while (UART1_FR_R & UART_FR_RXFE);\par
    return UART1_DR_R & 0xFF;\par
\}\par
\par
//Get command from string\par
char* getstring(uint8_t field)\par
\{\par
    return &str[position[field]];\par
\}\par
\par
//Get numbeer from inserted string\par
uint16_t getnumber (uint8_t field)\par
\{\par
    return atoi(&str[position[field]]);\par
\}\par
\par
uint8_t calculateCheckSum(uint8_t index)\par
\{\par
    uint16_t checkSum = 0;\par
    uint8_t i;\par
    checkSum = DestAddr[index] + SOURCE_ADDRESS + Sequence_Id[index] + command[index] + channel[index] + size[index];\par
    for(i = 0; i < size[index]; i++)\par
    \{\par
        checkSum = checkSum + Data[index][i];\par
    \}\par
    checkSum = checkSum  % 256;\par
    checkSum = ~checkSum;\par
    return checkSum;\par
\}\par
\par
char* Lower2Upper(char *string)\par
\{\par
    uint16_t i = 0;\par
    while (string[i] != '\\0')\par
    \{\par
        if (string[i] >= 'a' && string[i] <= 'z')\par
        \{\par
            string[i] = string[i] - 32;\par
        \}\par
        i++;\par
    \}\par
    return string;\par
\}\par
\par
bool iscommand(char comand[], uint8_t minArgs)\par
\{\par
    char *stringstr;\par
    stringstr = &str[position[0]];\par
    if((strcmp(Lower2Upper(stringstr),comand)== 0) && (field > minArgs))\par
    \{\par
        return true;\par
    \}\par
    return false;\par
\}\par
\par
\par
\par
\par
//function that will send a packet of data\par
void sendpacket()\par
\{\par
    for(i=0;i<MAX_MSG;i++)\par
    \{\par
        if(i!=true)\par
        \{\par
            DestAddr[i]=ADDRESS;\par
            Sequence_Id[i]=SEQUENCE_ID;\par
            SEQUENCE_ID++;\par
            channel[i]=CHANNEL;\par
            size[i]=SIZE;\par
\par
            for(x=0;x<size[i];x++)\par
            \{\par
                Data[i][x] = VALUE[x];\par
            \}\par
            ACKreq[i]= ACKflag;\par
            if(ACKreq[i])\par
            \{\par
                command[i] = comm | 0x80;\par
                timeout[i] = 200;\par
            \}\par
            else\par
            \{\par
                command[i] = comm;\par
                timeout[i] = 00;\par
            \}\par
            CHECKSUM[i]=calculateCheckSum(i);\par
            Retranscount[i]=0;\par
            Valid[i]=true;\par
            break;\par
        \}\par
        else\par
        \{\par
            putsUart0("\\n\\rERROR\\n");\par
        \}\par
    \}\par
\}\par
\par
// Get string from user\par
void get_string()\par
\{\par
    while(count<MAX_CHAR)\par
    \{\par
        c = getcUart0();\par
        putcUart0(c);\par
        if(c=='\\b')\par
        \{\par
            if(count>0)\par
            count-=1;\par
            else\par
            \{\par
                putsUart0("\\n\\rERROR\\n\\n");\par
                putsUart0("\\n\\rInvalid Command\\n\\n");\par
            \}\par
        \}\par
\par
        if(c=='\\r')\par
        \{\par
            str[count++]='\\0';\par
            break;\par
        \}\par
        if(c>=32)\par
        \{\par
            str[count++]=c;\par
            if(count>MAX_CHAR)\par
            \{\par
                str[count++]='\\0';\par
                break;\par
            \}\par
        \}\par
\par
    \}\par
\}\par
\par
\par
void init_table()\par
\{\par
    uint8_t i;\par
    for(i=0;i<MAX_MSG;i++)\par
    \{\par
        Valid[i]=false;\par
    \}\par
\}\par
\par
void codeNdiscard()\par
\{\par
    for(p=0;str[p]!='\\0';p++)\par
    \{\par
        if((str[p]>='a' && str[p]<='z') || (str[p]>='A' && str[p]<='Z')||(str[p]>='0' && str[p]<='9'))\par
        \{\par
         if((str[p]>='a' && str[p]<='z') || (str[p]>='A' && str[p]<='Z'))\par
         \{\par
             field+=1;\par
             type[x]='a';\par
             x++;\par
             position[i]=p;\par
             i++;\par
             while((str[p]>='a' && str[p]<='z') || (str[p]>='A' && str[p]<='Z'))\par
             \{\par
                 p++;\par
             \}p-=1;\par
         \}\par
         else\par
         \{\par
         field+=1;\par
         type[x]='n';\par
         x++;\par
         position[i]=p;\par
         i++;\par
         while((str[p]>='0' && str[p]<='9'))\par
         \{\par
             p++;\par
         \}\par
       \}\par
    \}\par
       else\par
       \{\par
           str[p]='\\0';\par
       \}\par
    \}\par
    for(i=0;i<field;i++)\par
      \{\par
          if(type[i]=='a')\par
          \{\par
               sprintf(str3,"%u,alpha,%s\\n",i,&str[position[i]]);\par
          \}\par
          else\par
          \{\par
              sprintf(str3,"%u,number,%s\\n",i,&str[position[i]]);\par
          \}\par
          putsUart0(str3);\par
      \}\par
\par
\}\par
\par
//Set 9th bit\par
void SetBit()\par
\{\par
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN|UART_LCRH_PEN|UART_LCRH_SPS;\par
\}\par
\par
//Clear 9th bit\par
void ClearBit()\par
\{\par
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN|UART_LCRH_EPS|UART_LCRH_PEN|UART_LCRH_SPS;\par
\}\par
\par
void tx9()\par
\{\par
    switch(currentphase)\par
    \{\par
    case 0:\par
        if (!(UART1_FR_R & UART_FR_BUSY))\par
            \{\par
                SetBit();\par
                putcUart1(DestAddr[currentindex]);\par
                currentphase++;\par
\par
                //TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag\par
            \}\par
            break;\par
    case 1:\par
        if (!(UART1_FR_R & UART_FR_BUSY))\par
            \{\par
                ClearBit();\par
                putcUart1(SOURCE_ADDRESS);\par
                currentphase++;\par
\par
            \}\par
        break;\par
\par
    case 2:\par
\par
\par
            putcUart1(Sequence_Id[currentindex]);\par
            currentphase++;\par
            break;\par
\par
    case 3:\par
            putcUart1(command[currentindex]);\par
            currentphase++;\par
            break;\par
\par
    case 4:\par
            putcUart1(channel[currentindex]);\par
            currentphase++;\par
            break;\par
\par
    case 5:\par
            putcUart1(size[currentindex]);\par
            currentphase++;\par
            g=size[currentindex];\par
            break;\par
\par
    case 6:\par
            if(size[currentindex]!=0)\par
            \{\par
                uint8_t z;\par
                for(z=0;z<size[currentindex];z++)\par
                \{\par
                    putcUart1(Data[currentindex][z]);\par
                \}\par
            \}\par
            currentphase++;\par
            break;\par
\par
    case 7:\par
            putcUart1(CHECKSUM[currentindex]);\par
            currentphase++;\par
            dtimer=20;\par
            break;\par
\par
    \}\par
    // RED_LED = 1;\par
    //    waitMicrosecond(50000);\par
    //    RED_LED = 0;\par
    //    waitMicrosecond(50000);\par
\}\par
\par
\par
bool cmdcheck()\par
\{\par
    bool error=true;\par
    /*if((field>6)||(field==1))\par
    \{\par
        putsUart0("\\n\\rInvalid Command\\n\\r");\par
    \}*/\par
    if(field==4)\par
        \{\par
            if(iscommand("SET",3))\par
            \{\par
                //uint8_t val=0;\par
                SIZE=1;\par
                comm= cmdSet;\par
                putsUart0("\\n\\rCommand entered is SET\\n");\par
                ADDRESS= getnumber(1);\par
                //putsUart0("\\rADDRESS is\\t");\par
                //sprintf(str2,"%u",ADDRESS);\par
                //putsUart0(str2);\par
                CHANNEL= getnumber(2);\par
                //putsUart0("\\n\\rCHANNEL is\\t");\par
                //sprintf(str2,"%u",CHANNEL);\par
                //putsUart0(str2);\par
                VALUE[0] = getnumber(3);\par
                //val= VALUE[0];\par
                //putsUart0("\\n\\rVALUE is\\t");\par
                //sprintf(str2,"%u",val);\par
                //putsUart0(str2);\par
                sendpacket( );\par
                error=false;\par
            \}\par
        \}\par
        if(field==1)\par
        \{\par
            if(iscommand("POLL",0))\par
            \{\par
                putsUart0("\\n\\rPolling is requested\\n");\par
                ADDRESS= BroadcastAddr;\par
                comm=cmd_POLL_request;\par
                SIZE=0;\par
                VALUE[0]=0;\par
                error=false;\par
                sendpacket();\par
            \}\par
        \}\par
\par
        if(field==6)\par
        \{\par
            if(iscommand("RGB",5))\par
            \{\par
                putsUart0("\\n\\rCommand entered is RGB\\n");\par
                SIZE=3;\par
                comm=cmd_RGB;\par
                ADDRESS=getnumber(1);\par
                CHANNEL=getnumber(2);\par
                VALUE[0]=getnumber(3);\par
                VALUE[1]=getnumber(4);\par
                VALUE[2]=getnumber(5);\par
                sendpacket();\par
                error=false;\par
            \}\par
            if(iscommand("PULSE",5))\par
                     \{\par
                         putsUart0("\\n\\rEntered command is PULSE\\n");\par
                         comm=cmd_PULSE;\par
                         SIZE=3;\par
                         ADDRESS=getnumber(1);\par
                         CHANNEL=getnumber(2);\par
                         VALUE[0]=getnumber(3);\par
                         VALUE[1]=getnumber(4);\par
                         VALUE[2]=getnumber(5);\par
                         sendpacket();\par
                         error=false;\par
                     \}\par
\par
        \}\par
        if(field==3)\par
        \{\par
            if(iscommand("GET",2))\par
            \{\par
                SIZE=0;\par
                comm= cmd_GET;\par
                putsUart0("\\n\\rCommand entered is GET\\n");\par
                ADDRESS=getnumber(1);\par
                CHANNEL=getnumber(2);\par
                VALUE[0]=0;\par
                sendpacket();\par
                error=false;\par
            \}\par
            if(iscommand("SA",2))\par
            \{\par
                SIZE=1;\par
                comm=cmd_SA;\par
                putsUart0("\\n\\rCommand entered is Set Address\\n");\par
                ADDRESS=getnumber(1);\par
                VALUE[0]=getnumber(2);\par
                sendpacket();\par
                error=false;\par
            \}\par
        \}\par
        if(field==2)\par
        \{\par
            if(iscommand("RESET",1))\par
            \{\par
                SIZE=0;\par
                comm=cmd_RESET;\par
                putsUart0("\\n\\rCommand entered is RESET\\n");\par
                ADDRESS=getnumber(1);\par
                CHANNEL=0;\par
                VALUE[0]=0;\par
                sendpacket();\par
                error=false;\par
            \}\par
            if (iscommand("ACK",1))\par
            \{\par
                putsUart0("\\n\\rCommand entered is ACK\\n");\par
                //valid=false;\par
                if((strcmp(getstring(1),"ON")==0)||(strcmp(getstring(1),"on")==0))\par
                \{\par
                    error=false;\par
                    putsUart0("\\n\\rAcknoledgement turned ON");\par
                    return ACKflag = true;\par
\par
                \}\par
                if((strcmp(getstring(1),"OFF")==0)||(strcmp(getstring(1),"off")==0))\par
                \{\par
                    error=false;\par
                    putsUart0("\\n\\rrAcknoledgement turned OFF\\n");\par
                    return ACKflag = false;\par
                \}\par
            \}\par
            if(iscommand("RANDOM",1))\par
            \{\par
                putsUart0("\\n\\r Command entered is RANDOM\\n");\par
                if((strcmp(getstring(1),"ON")==0)||(strcmp(getstring(1),"on")==0))\par
                \{\par
                    error=false;\par
                    putsUart0("\\n\\rRandomizer turned ON\\n");\par
                    return RAND_Enable=true;\par
                \}\par
                if((strcmp(getstring(1),"OFF")==0)||(strcmp(getstring(1),"off")==0))\par
                \{\par
                    error=false;\par
                    putsUart0("\\n\\rRandomizer turned OFF\\n");\par
                    return RAND_Enable=false;\par
                \}\par
            \}\par
            if (iscommand("CS",1))\par
            \{\par
                putsUart0("\\n\\rCommand entered is CS\\n");\par
                if((strcmp(getstring(1),"ON")==0)||(strcmp(getstring(1),"on")==0))\par
                \{\par
                    putsUart0("\\n\\rCarrier Sense turned ON\\n");\par
                    error=false;\par
                    return CS_Enable= true;\par
                \}\par
                if((strcmp(getstring(1),"OFF")==0)||(strcmp(getstring(1),"off")==0))\par
                \{\par
                    putsUart0("\\n\\rCarrier Sense turned OFF\\n");\par
                    error=false;\par
                    return CS_Enable= false;\par
                \}\par
            \}\par
        \}\par
\par
        if(error==true)\par
        \{\par
            putsUart0("\\n\\rInvalid Command\\n");\par
        \}\par
        return error;\par
\}\par
\par
void Tx_Data()\par
\{\par
    if(!inprogress)\par
    \{\par
        uint16_t foundindex;\par
        for(foundindex = 0; foundindex < 25; foundindex++)\par
        \{\par
            if(Valid[foundindex] && timeout[i]==0)\par
            \{\par
                inprogress = true;\par
                g=0;\par
                currentindex = foundindex;\par
                currentphase = 0;\par
               // tx9();\par
                break;\par
            \}\par
\par
        \}\par
    \}\par
    if(inprogress)\par
    \{\par
        GPIO_PORTC_DATA_R|=0x40;\par
        //DataEnable = 1;\par
\par
        while ((UART1_FR_R & UART_FR_TXFF) == 0 && size[currentindex]+7 > currentphase+g-1)\par
        \{\par
       // if (!(UART1_FR_R & UART_FR_BUSY))\par
         //transmit data when buffer is not full\par
\par
            if(CS_Enable==true)\par
            \{\par
                if(Rx_Phase==0)\par
                \{\par
                    tx9();\par
                \}\par
            \}                                                   //CHANGE\par
            if(CS_Enable==false)\par
            \{\par
                tx9();\par
            \}\par
\par
        \}\par
        if(size[currentindex]+7 == currentphase + g-1)\par
        \{\par
                GPIO_PORTB_DATA_R|=0x20;\par
                //RED_LED = 1;\par
                R_LEDtimer = 20;\par
                while ((UART1_FR_R & UART_FR_BUSY));\par
                GPIO_PORTC_DATA_R &=~0x40;\par
                //DataEnable = 0;\par
                if(!ACKreq[currentindex])\par
                \{\par
                    Valid[currentindex] = false;\par
                    inprogress = false;\par
                \}\par
                else\par
                \{\par
                    if(Retranscount[currentindex] >= MAX_REtrans)\par
                    \{\par
                        Valid[currentindex] = false;\par
                        inprogress = false;\par
                        //DataEnable = 0;\par
                    \}\par
                    else\par
                    \{\par
                        //calculate and store retransmit time out.\par
                        Retranscount[currentindex]++;\par
                        if(RAND_Enable==false)\par
                        \{\par
                            timeout[currentindex] = 200;\par
                        \}                                              //CHANGE\par
                        if(RAND_Enable==true)\par
                        \{\par
                            srand(SOURCE_ADDRESS);\par
                            n=8;\par
                            RANDOM=pow(2,n)*50*(rand() % 54);\par
                            sprintf(str2,"%u", RANDOM);\par
                            putsUart0(str2);\par
                            timeout[currentindex]=RANDOM;\par
                        \}\par
                        inprogress = false;\par
                    \}\par
\par
                \}\par
            \}\par
\par
    \}\par
\par
\}\par
void analog()\par
\{\par
    GPIO_PORTF_AFSEL_R |= 0x0E;\par
    GPIO_PORTF_PCTL_R|= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7 ;\par
\}\par
\par
void digital()\par
\{\par
    GPIO_PORTF_AFSEL_R &= 0x00;\par
    GPIO_PORTF_PCTL_R = 0;\par
\}\par
\par
setRgbColor(uint16_t red, uint16_t green, uint16_t blue)\par
\{\par
    PWM1_2_CMPB_R = red;\par
    PWM1_3_CMPA_R = blue;\par
    PWM1_3_CMPB_R = green;\par
\}\par
\par
void process_msg()\par
\{\par
    drtimer=20;\par
    //uint16_t mul=0;\par
    uint16_t ca;\par
    uint16_t CAL;\par
    uint16_t CA;\par
\par
    uint16_t Rx_channel;\par
    uint16_t Rx_data;\par
    uint16_t Rx_data1;\par
    uint16_t Rx_data2;\par
    uint16_t Rx_destaddr;\par
    uint16_t Rx_seqid;\par
    uint16_t Req_Data;\par
    uint8_t Rx_checksum;\par
    uint8_t Rx_srcaddr;\par
    uint8_t Rx_size;\par
    Rx_data1=ReceivedData[7];\par
           Rx_data2=ReceivedData[8];\par
    Rx_srcaddr=ReceivedData[0];\par
    putsUart0("\\n\\rDestination Address is\\t");\par
    sprintf(str2,"%u",Rx_srcaddr);\par
    putsUart0(str2);\par
    Rx_destaddr=ReceivedData[1];\par
    putsUart0("\\n\\rSource Address is\\t");\par
    sprintf(str2,"%u",Rx_destaddr);\par
    putsUart0(str2);\par
    Rx_seqid=ReceivedData[2];\par
    putsUart0("\\n\\rSequence Id is\\t");\par
    sprintf(str2,"%u", Rx_seqid);\par
    putsUart0(str2);\par
    Rx_command=ReceivedData[3];\par
    putsUart0("\\n\\rCommand is\\t");\par
    sprintf(str2,"%u", Rx_command);\par
    putsUart0(str2);\par
    Rx_channel=ReceivedData[4];\par
    putsUart0("\\n\\rChannel is\\n\\r");\par
    sprintf(str2,"%u", Rx_channel);\par
    putsUart0(str2);\par
    Rx_size=ReceivedData[5];\par
    putsUart0("\\n\\rSize is\\t");\par
    sprintf(str2,"%u", Rx_size);\par
    putsUart0(str2);\par
    Rx_data=ReceivedData[6];\par
    putsUart0("\\n\\rData is\\t");\par
    sprintf(str2,"%u", Rx_data);\par
    putsUart0(str2);\par
    Rx_checksum=ReceivedData[Max_Packet+si-2];\par
    putsUart0("\\n\\rChecksum is\\t");\par
    sprintf(str2,"%u", Rx_checksum);\par
    putsUart0(str2);\par
    ca=    Rx_command+ Rx_channel+ Rx_destaddr+ Rx_seqid+ Rx_srcaddr+ Rx_size;\par
    if(Rx_command==cmd_RGB)\par
    \{\par
        ca+=Rx_data1+Rx_data2+Rx_data;\par
    \}\par
    if((Rx_command==cmdSet)||(Rx_command==cmd_DATA_report)||(Rx_command==0x80)||(Rx_command==cmd_POLL_report))\par
    \{\par
        ca+=Rx_data;\par
    \}\par
    CA= ca%256;\par
    CAL=~CA;\par
    if(Rx_checksum!=(uint8_t )CAL)\par
    \{\par
        putsUart0("\\n\\rChecksum Error\\n");\par
        GPIO_PORTE_DATA_R|=0x20;\par
        //GPIO_PORTB_DATA_R|=0x20;\par
        //RED_LED=1;\par
    \}\par
    digital();\par
    if((Rx_command&0x7F)==0x02)\par
    \{\par
        analog();\par
        setRgbColor(Rx_data,0,0);\par
        putsUart0("\\n\\rReceived PULSE command\\n");\par
        P_LEDtimer=Rx_data1*Rx_data2;\par
        //R_LEDtimer = 20;\par
        //for(i=0;i<mul;i++);\par
        //setRgbColor(0,0,0);\par
    \}\par
\par
    if((Rx_command & 0x80) == ACK_requested)\par
    \{\par
        for(i=0;i<MAX_MSG;i++)\par
        \{\par
            if(!Valid[i])\par
            \{\par
                DestAddr[i]=Rx_destaddr;\par
                Sequence_Id[i]=SEQUENCE_ID;\par
                SEQUENCE_ID++;\par
                channel[i]= Rx_channel;\par
                size[i]=1;\par
                for(x=0;x<size[i];x++)\par
                \{\par
                    Data[i][x] = Rx_seqid;\par
                \}\par
                //ACKreq[i]= ACKflag;\par
                command[i]=cmd_ACK_report;\par
                CHECKSUM[i]=calculateCheckSum(i);\par
                //Retranscount[i]=0;\par
                Valid[i]=true;\par
                break;\par
            \}\par
        \}\par
    \}\par
    /*if((Rx_command&0x7F)==cmd_DATA_report)\par
    \{\par
        Valid[i]=false;\par
        putsUart0("\\n\\rDATA ON CORR\\n");\par
        sprintf(str2,"%u",Rx_data);\par
        putsUart0(str2);\par
    \}*/\par
   if((Rx_command&0x7F)==cmd_RGB)\par
    \{\par
\par
        if(Rx_channel==5)\par
        \{\par
            analog();\par
            setRgbColor(Rx_data,Rx_data1,Rx_data2);\par
\par
\par
            /*if(Rx_data==1)\par
            \{\par
                GPIO_PORTF_DATA_R|=0x02;\par
                //RED_led=1;\par
            \}\par
            else\par
            \{\par
                GPIO_PORTF_DATA_R&=~0x01;\par
                //RED_led=0;\par
            \}\par
            if(Rx_data1==1)\par
            \{\par
                GPIO_PORTF_DATA_R|=0x08;\par
                //GREEN_led=1;\par
            \}\par
            else\par
            \{\par
                GPIO_PORTF_DATA_R &= ~0x08;\par
                //GREEN_led=0;\par
            \}\par
            if(Rx_data2==1)\par
            \{\par
                GPIO_PORTF_DATA_R |= 0x04;\par
                //BLUE_led=1;\par
            \}\par
            else\par
            \{\par
                GPIO_PORTF_DATA_R &= ~0x04;\par
                //BLUE_led=0;\par
            \}*/\par
        \}\par
    \}\par
   if((Rx_command & 0x7F)==cmd_POLL_request)\par
       \{\par
           putsUart0("\\n\\rReceived Polling request\\n");\par
           for(i=0;i<MAX_MSG;i++)\par
           \{\par
               if(!Valid[i])\par
               \{\par
                   DestAddr[i]=Rx_destaddr;\par
                   Sequence_Id[i]=SEQUENCE_ID;\par
                   SEQUENCE_ID++;\par
                   channel[i]= 0;\par
                   size[i]=1;\par
                   for(x=0;x<size[i];x++)\par
                   \{\par
                       Data[i][x] = SOURCE_ADDRESS;\par
                   \}\par
                   //ACKreq[i]= ACKflag;\par
                   command[i]=cmd_POLL_report;\par
                   CHECKSUM[i]=calculateCheckSum(i);\par
                   //Retranscount[i]=0;\par
                   Valid[i]=true;\par
                   break;\par
               \}\par
           \}\par
       \}\par
\par
       if((Rx_command & 0x7F)==cmd_DATA_report)\par
       \{\par
           putsUart0("\\n\\rReceived a data report command\\n");\par
           Valid[i]=false;\par
           putsUart0("\\n\\rData on the corresponding channel is\\n\\r");\par
           sprintf(str2,"%u",Rx_data);\par
           putsUart0(str2);\par
       \}\par
       if((Rx_command & 0x7F)==cmd_POLL_report)\par
       \{\par
           putsUart0("\\n\\r Received POLLING REPORT from node\\n");\par
           sprintf(str2,"%u",Rx_data);\par
           putsUart0(str2);\par
       \}\par
       if((Rx_command&0x7F)==cmd_ACK_report)\par
       \{\par
           Valid[i]=false;\par
           putsUart0("\\n\\r Received ACKNOLEDGEMENT REPORT\\n");\par
           //sprintf(str2,"%u",Rx_data);\par
           //putsUart0(str2)\par
           //putsUart0("\\n\\rSuccessfully received Ack\\n");\par
       \}\par
\par
    if((Rx_command&0x7F)==cmd_SA)\par
    \{\par
        SOURCE_ADDRESS= Rx_data;\par
        putsUart0("\\n\\r NEW ADDRESS OF THE NODE IS\\t");\par
        EEPROM_EERDWR_R = SOURCE_ADDRESS;\par
        sprintf(str2,"%u",Rx_data);\par
        putsUart0(str2);\par
    \}\par
    if((Rx_command&0x7F)==cmd_RESET)\par
    \{\par
        putsUart0("\\n\\rRECEIVED RESET COMMAND\\n");\par
        //NVIC_APINT_R |=0xFA050004;\par
        //NVIC_APINT_R |=4;\par
        ResetISR();\par
    \}\par
    if((Rx_command&0x7F)==cmd_GET)\par
    \{\par
        putsUart0("\\n\\rReceived GET command\\n");\par
        if(Rx_channel==2)\par
        \{\par
            putsUart0("\\n\\rProcessing Request\\n");\par
            if((GPIO_PORTF_DATA_R & 0x10)==0x10)\par
            \{\par
                Req_Data=1;\par
            \}\par
            if((GPIO_PORTF_DATA_R & 0x10)==~0x10)\par
            \{\par
                Req_Data=0;\par
            \}\par
            for(i=0;i<MAX_MSG;i++)\par
            \{\par
                if(!Valid[i])\par
                \{\par
                    DestAddr[i]=Rx_destaddr;\par
                    Sequence_Id[i]=SEQUENCE_ID;\par
                    SEQUENCE_ID++;\par
                    channel[i]= Rx_channel;\par
                    size[i]=1;\par
                    for(x=0;x<size[i];x++)\par
                    \{\par
                        Data[i][x] = Req_Data;\par
                    \}\par
                    //ACKreq[i]= ACKflag;\par
                    command[i]=0x21;\par
                    CHECKSUM[i]=calculateCheckSum(i);\par
                    //Retranscount[i]=0;\par
                    Valid[i]=true;\par
                    break;\par
                \}\par
            \}\par
        \}\par
\par
    \}\par
    if((Rx_command&0x7F)==cmdSet)\par
    \{\par
        putsUart0("\\n\\rReceived SET command\\n");\par
        if(Rx_channel==1)\par
        \{\par
            putsUart0("\\n\\rCHANNEL 1 SELECTED\\n");\par
            if(Rx_data==1)\par
            \{\par
                putsUart0("\\n\\rBLUE LED 'ON'\\n");\par
                GPIO_PORTF_DATA_R |= 0x04;\par
                //BLUE_led=1;\par
            \}\par
            if(Rx_data==0)\par
            \{\par
                putsUart0("\\n\\rBLUE LED 'OFF'\\n");\par
                GPIO_PORTF_DATA_R &=  ~0x04;\par
                //BLUE_led=0;\par
            \}\par
        \}\par
    \}\par
\}\par
\par
void Rx_Data()\par
\{\par
    uint16_t Rec_Data;\par
    GPIO_PORTC_DATA_R&=~0x40;\par
    //DataEnable = 0;\par
    Rx_Phase=0;\par
    SetBit();\par
    while(Rx_Phase < Max_Packet+si-1 && !inprogress && (UART1_FR_R & UART_FR_RXFE) == 0)\par
    \{ //flip flop should not be empty\par
        Rec_Data = UART1_DR_R & 0x2FF;\par
        if(Rx_Phase != 0)\par
        \{\par
            ReceivedData[Rx_Phase] = Rec_Data & 0xFF;\par
            if(Rx_Phase==5)\par
            \{\par
                si=ReceivedData[Rx_Phase];\par
            \}\par
            Rx_Phase++;\par
        \}\par
        if(((Rec_Data & 0x200) == 0))\par
        \{\par
            if(((Rec_Data & 0x0FF) == SOURCE_ADDRESS) || ((Rec_Data & 0x0FF) == BroadcastAddr) )\par
            \{\par
                Rx_Phase = 0;\par
                ReceivedData[Rx_Phase] = Rec_Data & 0x0FF;\par
                Rx_Phase++;\par
            \}\par
        \}\par
        if(Rx_Phase == Max_Packet+si-1)\par
        \{\par
            GPIO_PORTE_DATA_R|=0x20;\par
            //GREEN_LED = 1;\par
            G_LEDtimer = 20;\par
            process_msg();\par
        \}\par
    \}\par
\}\par
\par
\par
\par
\par
void Decrement_Timeout()\par
\{\par
    uint8_t i;\par
    for(i = 0; i < 25; i++)\par
    \{\par
        if(timeout[i] > 0)\par
        \{\par
            timeout[i]--;\par
        \}\par
    \}\par
\}\par
void pulseLed_Timeout()\par
\{\par
    if(P_LEDtimer > 0)\par
    \{\par
        P_LEDtimer--;\par
    \}\par
    if(P_LEDtimer == 0 &&((Rx_command&0x7F)==0x02))\par
    \{\par
        setRgbColor(0,0,0);\par
        //GPIO_PORTB_DATA_R&=~0x20;\par
        //RED_LED = 0;\par
    \}\par
\}\par
void RLed_Timeout()\par
\{\par
    if(R_LEDtimer > 0)\par
    \{\par
        R_LEDtimer--;\par
    \}\par
    if(R_LEDtimer == 0)\par
    \{\par
        GPIO_PORTB_DATA_R&=~0x20;\par
        //RED_LED = 0;\par
    \}\par
\}\par
\par
void GLed_Timeout()\par
\{\par
    if(G_LEDtimer > 0)\par
    \{\par
        G_LEDtimer--;\par
    \}\par
    if(G_LEDtimer == 0)\par
    \{\par
        GPIO_PORTE_DATA_R&=~0x20;\par
        //GREEN_LED = 0;\par
    \}\par
\}\par
\par
\par
void Clear_Str()\par
\{\par
    for(i=0;i<MAX_CHAR;i++)\par
    \{\par
        str[i]='\\0';\par
        position[i]='\\0';\par
        type[i]='\\0';\par
    \}\par
\}\par
\par
\par
void Rdeadlock()\par
\{\par
    \{\par
        if(Rx_Phase!=0)\par
        \{\par
            if(variable1==Rx_Phase)\par
            \{\par
                if(drtimer>0)\par
                     \{\par
                         drtimer--;\par
                     \}\par
            \}\par
\par
            if(drtimer==0 )      //CHANGE\par
            \{\par
                Rx_Phase=0;\par
            \} variable1=Rx_Phase;\par
        \}\par
    \}\par
\}\par
void deadlock()\par
\{\par
    if(currentphase!=0)\par
    \{\par
        if(variable==currentphase)\par
        \{\par
            if(dtimer>0)\par
                 \{\par
                     dtimer--;\par
                 \}\par
        \}\par
        if(dtimer==0 )      //CHANGE\par
        \{\par
            currentphase=0;\par
        \} variable=currentphase;\par
    \}\par
\}\par
void Timer1Isr()\par
\{\par
   // GREEN_LED ^= 1;\par
    //transmission\par
    //GREEN_LED=1;\par
    Tx_Data();\par
    //receiving\par
    Rx_Data();\par
    //timeout in ack\par
    Decrement_Timeout();\par
    RLed_Timeout();\par
    GLed_Timeout();\par
    pulseLed_Timeout();\par
    deadlock();\par
    Rdeadlock();\par
    //if (!(UART1_FR_R & UART_FR_BUSY))\par
   // DataEnable = 0;\par
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;            // clear interrupt flag\par
\}\par
   /* void Timer1Isr()\par
    \{\par
        //if (!timeMode)\par
        //\{\par
                                      // set update flag\par
            GREEN_LED ^= 1;                              // status\par
        //\}\par
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag\par
    \}*/\par
\par
void waitMicrosecond(uint32_t us)\par
\{\par
\par
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1\par
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6\par
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3\par
    __asm("             NOP");                  // 5\par
    __asm("             NOP");                  // 5\par
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)\par
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1\par
    __asm("             CBZ  R0, WMS_DONE0");   // 1\par
    __asm("             NOP");                  // 1\par
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)\par
    __asm("WMS_DONE0:");\par
\}\par
\par
void In_Str()\par
\{\par
    count=0;\par
    field=0;\par
    x=0;\par
    i=0;\par
\}\par
\par
int main(void)\par
\{\par
    initHw();\par
    if(ISERROR)\{\par
        putsUart0("error\\r\\n");\par
    \}\par
    EEPROM_EEBLOCK_R = 0x10;\par
    EEPROM_EEOFFSET_R = 0x10;\par
    uint32_t address = EEPROM_EERDWR_R;\par
    if(address != 0xFFFFFFFF)\{\par
        SOURCE_ADDRESS = (uint8_t)address;\par
    \}\par
    GPIO_PORTF_DATA_R|=0x08;\par
    //GREEN_led=1;\par
    waitMicrosecond(500000);\par
    GPIO_PORTF_DATA_R&=~0x08;\par
    //GREEN_led=0;\par
\par
    init_table();\par
    while(1)\par
    \{\par
\par
        //while(1)\par
        putsUart0("\\n\\rREADY");\par
        putsUart0("\\n\\rEnter Command...\\n\\r");\par
        get_string();\par
        codeNdiscard();\par
        cmdcheck();\par
        Clear_Str();\par
        In_Str();\par
   \}\par
\}\par
}
 