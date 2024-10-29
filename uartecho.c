/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== uartecho.c ========
 */

#include <headerfilesfuncdef.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/gates/gateSwi.h>

#define tickerCount 16
#define scriptCount 64
#define regCount    32

extern GateSwi_Handle gateSwi0; //MP Queue read and write gate
extern GateSwi_Handle gateSwi1; //Ticker gate

/*---------------------------------------------------------------*/

extern void     *MP                 ();
extern void     *MPabout            ();
extern void     *MPcallback         ();
extern void     *MPcallbackSpace    ();
extern void     *MPenterKeyOrSpace  ();
extern void     *MPerror            ();
extern void     *MPerrorClear       ();
extern void     *MPerrorSpace       ();
extern void     *MPgpio             ();
extern void     *MPgpioSpace        ();
extern void     *MPhelp             ();
extern void     *MPif               ();
extern void     *MPmemr             ();
extern void     *MPmemrSpace        ();
extern void     *MPmsgNotUnderstood ();
extern void     *MPoverflow         ();
extern void     *MPprint            ();
extern void     *MPreg              ();
extern void     *MPregSpace         ();
extern void     *MPscript           ();
extern void     *MPscriptSpace      ();
extern void     *MPticker           ();
extern void     *MPtickerKill       ();
extern void     *MPtickerSpace      ();
extern void     *MPtimer            ();
extern void     *MPtimerZero        ();
extern void     *MPtimerSpace       ();
extern uint8_t  strCheckNext       (char* str);
extern uint8_t  strClear           (char* str, uint32_t len);
extern uint8_t  strStartsWith      (char* str, char* subStr);

extern void *MPcallbackStr();

extern void     MPcallbackRequest   (char* fourthStr,uint32_t fourthStrLen);
extern void     callbackTimer0(Timer_Handle timer0,int_fast16_t statusTimer0);
extern void     callbackTimer1(Timer_Handle timer1,int_fast16_t statusTimer1);
extern void     callbackSW1Left     (uint_least8_t index6);
extern void     callbackSW2Right    (uint_least8_t index7);

extern void SWIgpioButtonSW1();
extern void SWIgpioButtonSW2();
extern void SWItimer0();
extern void SWItimer1();
extern void AddPayload(char *payload);

typedef struct Rec {
Queue_Elem elem;
char payload[320];
} Rec;
Queue_Handle myQ;
Rec r[32];
Rec* writeIndex;
Rec* readIndex;

typedef struct _payloadQueue   // MP queue
{
    int32_t payloadReading;
    int32_t payloadWriting;
    char payloads[32][320];
}PQ;
PQ ParseQueue;

char script[scriptCount][320];

typedef struct ticker{
//uint32_t Number;
uint32_t delay;
uint32_t period;
int32_t count;
char     function[320];
}tic;
tic ticker[tickerCount];

int32_t reg[regCount];

uint32_t wIndex;
uint32_t rIndex;
int32_t tickerNum;
int32_t scriptNum;
//int32_t regNum;
int32_t callbackCount;
uint32_t MPQueueWriteCycles;
uint32_t MPQueueReadCycles;
Queue_Elem *elem;

Swi_Handle Timer0SWI;
Swi_Handle Timer1SWI;
Swi_Handle SW1SWI;
Swi_Handle SW2SWI;
Swi_Params swiParams;
xdc_runtime_Error_Block eb;

uint32_t gatekey;
uint32_t timerPeriod;
/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    int i;
    myQ = Queue_create(NULL, NULL);
    wIndex=0;
    rIndex=0;
    typedMsgLen=strClear(typedMsg,msgMaxLen);

    /* Call driver init functions */
    GPIO_init();
    UART_init();
    Timer_init();

    /*----------------------------------------------------------------------------------------------------------------------*/
    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_2, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_3, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_0, CONFIG_GPIO_LED_ON);
    GPIO_write(CONFIG_GPIO_1, CONFIG_GPIO_LED_ON);
    GPIO_write(CONFIG_GPIO_2, CONFIG_GPIO_LED_ON);
    GPIO_write(CONFIG_GPIO_3, CONFIG_GPIO_LED_ON);

    /*GPIO SW callback*/
    // install Button callback
    GPIO_setConfig(CONFIG_GPIO_6, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_7, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    GPIO_setCallback(CONFIG_GPIO_6, callbackSW1Left);
    GPIO_setCallback(CONFIG_GPIO_7, callbackSW2Right);
    // Enable interrupts
    GPIO_enableInt(CONFIG_GPIO_6);
    GPIO_enableInt(CONFIG_GPIO_7);
    /*----------------------------------------------------------------------------------------------------------------------*/
    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;//UART_MODE_CALLBACK;//;
    uartParams.baudRate = 115200;

    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL)
    {
        /* UART_open() failed */
        UART_write(uart, failedPromt, sizeof(failedPromt));
        while (1);
    }
    UART_write(uart, echoPrompt, sizeof(echoPrompt));
/*----------------------------------------------------------------------------------------------------------------------*/
    /* Create a TIMER0 with data processing off. */
    Timer_Params_init(&timer0Params);
    timer0Params.periodUnits = Timer_PERIOD_US;
    timer0Params.period = 1000000;
    timer0Params.timerMode  =Timer_CONTINUOUS_CALLBACK;//Timer_ONESHOT_CALLBACK
    timer0Params.timerCallback = callbackTimer0;

    timer0 = Timer_open(CONFIG_TIMER_0, &timer0Params);
    if (timer0 == NULL)
    {
        // Timer_open() failed
        while (1);
    }
    statusTimer0=Timer_start(timer0);
    if (statusTimer0 == Timer_STATUS_ERROR)
    {
        // Timer_start() failed
        sprintf(outputMsg, "\r\ntimer0 start for callback0 failed\r\n");
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        while (1);
    }
/*----------------------------------------------------------------------------------------------------------------------*/
    /* Create a TIMER1 with data processing off. */
    Timer_Params_init(&timer1Params);
    timer1Params.periodUnits = Timer_PERIOD_US;
    timer1Params.period = 10000;
    timer1Params.timerMode  =Timer_CONTINUOUS_CALLBACK;//Timer_ONESHOT_CALLBACK
    timer1Params.timerCallback = callbackTimer1;

    timer1 = Timer_open(CONFIG_TIMER_1, &timer1Params);
    if (timer1 == NULL)
    {
        // Timer_open() failed
        while (1);
    }
    statusTimer1=Timer_start(timer1);
    if (statusTimer1 == Timer_STATUS_ERROR)
    {
        // Timer_start() failed
        sprintf(outputMsg, "\r\ntimer1 start for tickers failed\r\n");
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        while (1);
    }


/*----------------------------------------------------------------------------------------------------------------------*/
    //SWI creations

  //  Error_init(&eb);
    Swi_Params_init(&swiParams);
    Timer0SWI = Swi_create(SWItimer0, &swiParams, &eb);
    if (Timer0SWI == NULL)
    {
  //  System_abort("Timer0SWI create failed");
        while(1);
    }
    Timer1SWI = Swi_create(SWItimer1, &swiParams, &eb);
    if (Timer1SWI == NULL)
    {
  //  System_abort("Timer1SWI create failed");
        while(1);
    }
    SW1SWI = Swi_create(SWIgpioButtonSW1, &swiParams, &eb);
    if (SW1SWI == NULL)
    {
  //  System_abort("SW1SWI create failed");
        while(1);
    }
    SW2SWI = Swi_create(SWIgpioButtonSW2, &swiParams, &eb);
    if (SW2SWI == NULL)
    {
  //  System_abort("SW2SWI create failed");
        while(1);
    }
/*----------------------------------------------------------------------------------------------------------------------*/
    for (i=0;i<scriptCount;i++)
    {
        script[i][0]='\0'; // setting all the 64 instences of scripts to empty strings.
    }
    strcpy(script[ 0],"-gpio 0 w 0");
    strcpy(script[ 1],"-gpio 1 w 0");
    strcpy(script[ 2],"-gpio 2 w 0");
    strcpy(script[ 3],"-gpio 3 w 0");
    //script[4] is empty
    strcpy(script[ 5],"-gpio 0 w 1");
    strcpy(script[ 6],"-gpio 1 w 1");
    strcpy(script[ 7],"-gpio 2 w 1");
    strcpy(script[ 8],"-gpio 3 w 1");
    //script[9] is empty
    strcpy(script[10],"-gpio 0 t");
    strcpy(script[11],"-gpio 1 t");
    strcpy(script[12],"-gpio 2 t");
    strcpy(script[13],"-gpio 3 t");
    //script[14] is empty
    strcpy(script[15],"-ticker 1 100 700 1 -gpio 0 t");
    strcpy(script[16],"-ticker 2 200 500 1 -gpio 1 t");
    strcpy(script[17],"-ticker 3 300 300 1 -gpio 2 t");
    strcpy(script[18],"-ticker 4 400 100 1 -gpio 3 t");
    //script[19] is empty
    strcpy(script[20],"-callback 0 10 -script 10");
    strcpy(script[21],"-callback 1 -1 -reg 10 + 10 #1");
    strcpy(script[22],"-callback 2 1 -print hello there");
    strcpy(script[23],"-reg 10 =0");
    //script[24] is empty
    //strcpy(script[25],"-shadow n");
    //strcpy(script[26],"-shadow 3 t");
    //strcpy(script[27],"-stream 1");
    strcpy(script[28],"-ticker 15 100 0 0 -if 3 > # 899 ? -print hot : -print cold");
   // strcpy(script[29],"-ticker 15 101 0 0 -stream 0 0");
    //script[30] is empty
    //strcpy(script[31],"-stream 0");
    strcpy(script[32],"-timer 125");
    strcpy(script[33],"-gpio 6 w 0");
    //strcpy(script[34],"--sine 200");
    //strcpy(script[35],"-callback 0 -1 -sine");
    //script[36] is empty
    strcpy(script[37],"-timer 125");
    strcpy(script[38],"-gpio 6 w 0");
    strcpy(script[39],"-gpio 7 w 1");
    //script[40] is empty
    strcpy(script[41],"-timer 100000");
    strcpy(script[42],"-reg 10 = 0");
    strcpy(script[43],"-callback 0 -1 -script 46");
    strcpy(script[44],"-callback 1 -1 -reg 10 ++ 10");
    strcpy(script[45],"-callback 2 -1 -reg 10 -- 10");
    strcpy(script[46],"-if 10 > #0 ? -script 5 : -script 47");
    strcpy(script[47],"-if 10 < #0 ? -script 0 :");
  /*  strcpy(script[ 0],"-print exe script 0");
    strcpy(script[ 1],"-print exe script 1");
    strcpy(script[ 2],"-print exe script 2");
    strcpy(script[ 3],"-print exe script 3");
    strcpy(script[ 4],"-print exe script 4");
    strcpy(script[ 5],"-print exe script 5");
    strcpy(script[ 6],"-print exe script 6");
    strcpy(script[ 7],"-print exe script 7");
    strcpy(script[ 8],"-print exe script 8");
    strcpy(script[ 9],"-print exe script 9");
    strcpy(script[10],"-print exe script 10");
    strcpy(script[11],"-print exe script 11");
    strcpy(script[12],"-print exe script 12");
    strcpy(script[13],"-print exe script 13");
    strcpy(script[14],"-print exe script 14");
    strcpy(script[15],"-print exe script 15");
    strcpy(script[16],"-print exe script 16");
    strcpy(script[17],"-print exe script 17");
    strcpy(script[18],"-print exe script 18");
    strcpy(script[19],"-print exe script 19");
    strcpy(script[20],"-print exe script 20");
    strcpy(script[21],"-print exe script 21");
    strcpy(script[22],"-print exe script 22");
    strcpy(script[23],"-print exe script 23");
    strcpy(script[24],"-print exe script 24");
    strcpy(script[25],"-print exe script 25");
    strcpy(script[26],"-print exe script 26");
    strcpy(script[27],"-print exe script 27");
    strcpy(script[28],"-print exe script 28");
    strcpy(script[29],"-print exe script 29");
    strcpy(script[30],"-print exe script 30");
    strcpy(script[31],"-print exe script 31");
    strcpy(script[32],"-print exe script 32");
    strcpy(script[33],"-print exe script 33");
    strcpy(script[34],"-print exe script 34");
    strcpy(script[35],"-print exe script 35");
    strcpy(script[36],"-print exe script 36");
    strcpy(script[37],"-print exe script 37");
    strcpy(script[38],"-print exe script 38");
    strcpy(script[39],"-print exe script 39");
    strcpy(script[40],"-print exe script 40");  */

/*----------------------------------------------------------------------------------------------------------------------*/
    /* Loop forever echoing */

    for(;;)
    {
        UART_read(uart, &input, 1);
        UART_write(uart, &input, 1);
/*---------------------------------------------------------------------------------------------------------------------------------*/
        if(input==127) //if bacspace key is pressed delete the previous read input message and decrement its len.
        {
            conTypedMsgLen--;
            conTypedMsg[conTypedMsgLen]='\0';
        }//it was only if
/*---------------------------------------------------------------------------------------------------------------------------------*/
        else if((input != '\r') && (input!='\n'))  // untill enter key is detected every read char is stored into string tyedMsg
        {
            conTypedMsg[conTypedMsgLen]=input;
            conTypedMsgLen++;
        }
/*---------------------------------------------------------------------------------------------------------------------------------*/
        else if(conTypedMsgLen>msgMaxLen)  // typed message crossed conTypedMsgLen(320), overflow is detected
        {
            MPoverflow();
            conTypedMsgLen=strClear(conTypedMsg,msgMaxLen);
        }
/*---------------------------------------------------------------------------------------------------------------------------------*/
        else if(input == '\r' || input == '\n') // when enter key is detected
        {  //below code checks if all the typed charactars before enter key are only empty spaces
            allSpaces=0;
            for(i=0;i<=conTypedMsgLen;i++)
            {
                if(conTypedMsg[i]==' ')
                {
                     allSpaces++; //increamented in spaces is found and if allSpaces==conTypedMsgLen then all typed messages are spaces and do nothing in end of the code lines.
                }
            }
            for(i=0;i<conTypedMsgLen;i++)
            {
                r[wIndex].payload[i]=conTypedMsg[i];
            }
            r[wIndex].payload[i]='\0';
        //    sprintf(outputMsg,"\r\nmain thread payload is %s\r\n",r[wIndex].payload);
       //     UART_write(uart,outputMsg, strlen(outputMsg));
          //  gatekey= GateSwi_enter(gateSwi0);
         //   Queue_enqueue(myQ, &(r[wIndex].elem));
         //   strcpy(ParseQueue.payloads[ParseQueue.payloadWriting],conTypedMsg);
         //   ParseQueue.payloadWriting++;
         //   if(ParseQueue.payloadWriting>31)
         //   {
         //       ParseQueue.payloadWriting=0;
         //       MPQueueWriteCycles++
         //   }
         //   Semaphore_post(semaphore0);
         //   GateSwi_leave(gateSwi0,gatekey);
            AddPayload(conTypedMsg);
         //   wIndex++;
            conTypedMsgLen=strClear(conTypedMsg,msgMaxLen);

        }
    }
}//Task0 UART read and write, push payload to Queue.


//Task1 message parsing, get payload form Queue and do message parsing
void *MP()
{
   for(;;)
   {
    Semaphore_pend(semaphore0,BIOS_WAIT_FOREVER);
   // int i;
   // if(ParseQueue.payloadWriting==ParseQueue.payloadReading)
   // {
   //     sprintf(outputMsg,"\r\n MP queue is empty\r\n");
   //     UART_write(uart,outputMsg, strlen(outputMsg));
   // }
   // if((MPQueueWriteCycles>MPQueueReadCycles)&&(ParseQueue.payloadWriting>ParseQueue.payloadReading))
   // {
   //     sprintf(outputMsg,"\r\n MP queue overflow occured, %d instructions overwritten\r\n",(MPQueueWriteCycles>MPQueueReadCycles));
   //     UART_write(uart,outputMsg, strlen(outputMsg));
   //     MPQueueWriteCycles=0;
   //     MPQueueReadCycles=0;
   // }
    gatekey= GateSwi_enter(gateSwi0);
  //  readIndex=Queue_dequeue(myQ);
    strcpy(typedMsg,ParseQueue.payloads[ParseQueue.payloadReading]);
    ParseQueue.payloadReading++;
    if(ParseQueue.payloadReading>31)
        ParseQueue.payloadReading=0;
    GateSwi_leave(gateSwi0,gatekey);
    typedMsgLen=strlen(typedMsg);


    if(strStartsWith(typedMsg,          "-about")==0) // typed message is checked if it begins with -about.
    {
        MPabout();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if((strStartsWith(typedMsg,    "-callback")==0)&&(strlen(typedMsg)==strlen("-callback")))
    {
        MPcallback();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if(strStartsWith(typedMsg,     "-callback ")==0)
    {
        MPcallbackSpace();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if(((strStartsWith(typedMsg,   "-error c")==0) || (strStartsWith(typedMsg,"-error C")==0)) &&(strlen(typedMsg)==strlen("-error c")))
    {
        MPerrorClear();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if((strStartsWith(typedMsg,    "-error")==0) && (strlen(typedMsg)==strlen("-error")))
    {
        MPerror();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if(strStartsWith(typedMsg,     "-error ")==0)
    {
        MPerrorSpace();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if((strStartsWith(typedMsg,    "-gpio")==0) && (strlen(typedMsg)==strlen("-gpio")))
    {
        MPgpio();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if((strStartsWith(typedMsg,    "-gpio ")==0))
    {
        MPgpioSpace();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if(strStartsWith(typedMsg,     "-help")==0) // typed message is checked if it begins with -help.
    {
        MPhelp();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if(strStartsWith(typedMsg,     "-if")==0) // typed message is checked if it begins with -help.
    {
        MPif();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if(strStartsWith(typedMsg,     "-memr ")==0)
    {
        MPmemrSpace();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if((strStartsWith(typedMsg,    "-memr")==0)&&(typedMsgLen==strlen("-memr")))
    {   // if typed message is only "-memr" and no address mentioned, then consider 0 as default address.
        MPmemr();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if(strStartsWith(typedMsg,     "-print")==0) // typed message is checked if it is -print
    {
        MPprint();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if((strStartsWith(typedMsg,     "-reg")==0)&&(typedMsgLen==strlen("-reg"))) // typed message is checked if it is -reg
    {
        MPreg();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if(strStartsWith(typedMsg,     "-reg ")==0) // typed message is checked if it is -reg
    {
        MPregSpace();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if(((strStartsWith(typedMsg,    "-script")==0)&&(typedMsgLen==strlen("-script")))||((strStartsWith(typedMsg,    "-scripts")==0)&&(typedMsgLen==strlen("-scripts"))))
    {
        MPscript();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if(strStartsWith(typedMsg,    "-script ")==0)
    {
        MPscriptSpace();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if((strStartsWith(typedMsg,    "-ticker")==0)&&(typedMsgLen==strlen("-ticker")))
    {
        MPticker();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if((strStartsWith(typedMsg,    "-ticker -1")==0)&&(typedMsgLen==strlen("-ticker -1")))
    {
        MPticker();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if(((strStartsWith(typedMsg,"k")==0)||(strStartsWith(typedMsg,"K"))) &&(typedMsgLen==strlen("-ticker k")))
    {
        MPtickerKill();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if(strStartsWith(typedMsg,     "-ticker ")==0)
    {
        MPtickerSpace();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if((strStartsWith(typedMsg,    "-timer")==0)&&(typedMsgLen==strlen("-timer")))
    {
        MPtimer();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if((strStartsWith(typedMsg,    "-timer 0")==0)&&(typedMsgLen==strlen("-timer 0")))
    {
        MPtimerZero();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if(strStartsWith(typedMsg,     "-timer ")==0)
    {
        MPtimerSpace();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else if((typedMsgLen<=0) || (allSpaces==typedMsgLen))
    {
        MPenterKeyOrSpace();
    }
/*---------------------------------------------------------------------------------------------------------------------------------*/
    else // if begining of typed message is not any of the above messages the message is said to be invalid.
    {
        MPmsgNotUnderstood();
    }
    typedMsgLen=strClear(typedMsg,msgMaxLen);
    //UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
   }
}









/*---------------------------------------------------------------------------------------------------------------------*/
void *MPabout()
{
    sprintf(outputMsg,  "\r\nAuthor     : %s\r\n",                AUTHOR);
    UART_write(uart,    outputMsg,                            strlen(outputMsg));
    sprintf(outputMsg,  "Version    : %d Subversion: %d\r\n", VERSION,SUBVERSION);
    UART_write(uart,    outputMsg,                            strlen(outputMsg));
    sprintf(outputMsg,  "Assignment : %s\r\n",                ASSIGNMENT);
    UART_write(uart,    outputMsg,                            strlen(outputMsg));
    sprintf(outputMsg,  "Time       : %s Date: %s\r\n",       __TIME__,__DATE__);
    UART_write(uart,    outputMsg,                            strlen(outputMsg));
    sprintf(outputMsg,  "Board      : %s\r\n",                BOARD);
    UART_write(uart,    outputMsg,                            strlen(outputMsg));
    sprintf(outputMsg,  "Config     : %s\r\n",                CONFIG);
    UART_write(uart,    outputMsg,                            strlen(outputMsg));
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPcallback()
{
    char        outputMsg[msgMaxLen];
    sprintf(outputMsg, "\r\ncallback 0 c:%d payload:%s",callback0Count,callback0Payload);
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    sprintf(outputMsg, "\r\ncallback 1 c:%d payload:%s",callback1Count,callback1Payload);
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    sprintf(outputMsg, "\r\ncallback 2 c:%d payload:%s",callback2Count,callback2Payload);
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    sprintf(outputMsg, "\r\ncallback 3 c:0\r\n");
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPcallbackSpace()
{
    int i;
    int j=0;
    char secondStr[50];//="\0";
    char thirdStr[50];//="\0";
    char fourthStr[50];//="\0";
    strClear(secondStr,50);
    strClear(thirdStr,50);
    strClear(fourthStr,50);
    for(i=(strlen("-callback "));((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)//find if second str is present
    {
        secondStr[j]=typedMsg[i];
        j++;
    }
    num=atol(secondStr);
    //no third and fourth string in the typedMsg(example: typedMsg is only "-callback #")
    // and there is no space after second string
    if(!(typedMsg[((strlen("-callback "))+(strlen(secondStr)))]==' '))
    {
        if((num==-1)||(num>=4))
        {
             MPcallback();
        }
        else if(num==0)
        {
        sprintf(outputMsg, "\r\ncallback 0 c:%d payload:%s\r\n",callback0Count,callback0Payload);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        }
        else if(num==1)
        {
            sprintf(outputMsg, "\r\ncallback 1 c:%d payload:%s\r\n",callback1Count,callback1Payload);
            UART_write(uart,    outputMsg,  strlen(outputMsg));
        }
        else if(num==2)
        {
            sprintf(outputMsg, "\r\ncallback 2 c:%d payload:%s\r\n",callback2Count,callback2Payload);
            UART_write(uart,    outputMsg,  strlen(outputMsg));
        }
        else if(num==3)
        {
            sprintf(outputMsg, "\r\ncallback 3 c:0\r\n");
            UART_write(uart,    outputMsg,  strlen(outputMsg));
        }
    }
    else
    {
        if(typedMsg[((strlen("-callback "))+(strlen(secondStr)))]==' ')// if space after second string
        {
            j=0;
            for(i=((strlen("-callback "))+(strlen(secondStr))+1/* 1 is needed to skip the space after 2nd str*/);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
            {
                thirdStr[j]=typedMsg[i];
                j++;
            }
            if (typedMsg[((strlen("-callback "))+(strlen(secondStr))+(strlen(thirdStr)))+1]==' ')// +1 is to skip space after second str
            {
                j=0;
                for(i=((strlen("-callback "))+(strlen(secondStr))+(strlen(thirdStr))+2/* 2 is needed to skip the 2 space after 2nd str and 3rd str*/);(i<typedMsgLen); i++)
                {
                    fourthStr[j]=typedMsg[i];
                    j++;
                }
            }
            fourthStrLen=j;
        }
        callbackCount=atol(thirdStr);
        if(((strStartsWith(thirdStr,"c")==0) || (strStartsWith(thirdStr,"C")==0)) && (strlen(thirdStr)==1))
        {
            if(num==0)
            {
                callback0Count = strClear(callback0Payload,msgMaxLen);
            }
            else if(num==1)
            {
                callback1Count = strClear(callback1Payload,msgMaxLen);
            }
            else if(num==2)
            {
                callback2Count = strClear(callback2Payload,msgMaxLen);
            }
           // UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
        }
        else
        {
            if(callbackCount==0)
            {
                sprintf(outputMsg, "\r\ninvalid:callback count cannot be 0\r\n");
                UART_write(uart,    outputMsg,  strlen(outputMsg));
                parsingError++;
            }
            else if(num==0)
            {
                sprintf(callback0Payload,"%s",fourthStr);
                callback0Count=callbackCount;
                sprintf(thirdStr0Callback,"%s",thirdStr);
            }
            else if(num==1)
            {
                sprintf(callback1Payload,"%s",fourthStr);
                callback1Count=callbackCount;
                sprintf(thirdStr1Callback,"%s",thirdStr);
            }
            else if(num==2)
            {
                sprintf(callback2Payload,"%s",fourthStr);
                callback2Count=callbackCount;
                sprintf(thirdStr2Callback,"%s",thirdStr);
            }
            else if(num==3)
            {
                sprintf(outputMsg, "\r\ncallback 3 is not set up yet\r\n");
                UART_write(uart,    outputMsg,  strlen(outputMsg));
            }
            else
            {
                sprintf(outputMsg, "\r\ninvalid callback id\r\n");
                UART_write(uart,    outputMsg,  strlen(outputMsg));
                parsingError++;
            }
           // UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
            strClear(secondStr,50);
            strClear(thirdStr,50);
            strClear(fourthStr,50);
            return(0);
        }
    }
return 0;
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPenterKeyOrSpace()
{
    UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));//when just enter key or spaces followed by enter key is pressed do nothing
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPerror()
{
    UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
    sprintf(outputMsg, " 0 : %d : %s\r\n", bufferOverflow, ERROR_0);
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    sprintf(outputMsg, " 1 : %d : %s\r\n", messageOverflow, ERROR_1);
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    sprintf(outputMsg, " 2 : %d : %s\r\n", messageUnknown, ERROR_2);
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    sprintf(outputMsg, " 3 : %d : %s\r\n", messageFail, ERROR_3);
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    sprintf(outputMsg, " 4 : %d : %s\r\n", adcError, ERROR_4);
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    sprintf(outputMsg, " 5 : %d : %s\r\n", udpNetworkError, ERROR_5);
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    sprintf(outputMsg, " 6 : %d : %s\r\n", memrError, ERROR_6);
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    sprintf(outputMsg, " 7 : %d : %s\r\n", queueOverflow, ERROR_7);
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    sprintf(outputMsg, " 8 : %d : %s\r\n", voiceStreamingError, ERROR_8);
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    sprintf(outputMsg, " 9 : %d : %s\r\n", parsingError, ERROR_9);
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    sprintf(outputMsg, "10 : %d : %s\r\n", zeroError, ERROR_10);
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPerrorClear()
{
    bufferOverflow      = 0; // clear all the error conuts when -error c is typed
    messageOverflow     = 0;
    messageUnknown      = 0;
    messageFail         = 0;
    adcError            = 0;
    udpNetworkError     = 0;
    memrError           = 0;
    queueOverflow       = 0;
    voiceStreamingError = 0;
    parsingError        = 0;
    sprintf(outputMsg, "\r\nall error counts cleared\r\n");
    UART_write(uart,    outputMsg,  strlen(outputMsg));
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPerrorSpace()
{
    int i;
    int j=0;
    for(i=(strlen("-error ")); i<=typedMsgLen; i++)
    {
    printOutput[j]=typedMsg[i];
    j++;
    }// the above code saves all the charac after "-error" and same will be used to compare further
    num = atol(printOutput);
    if(num==0)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
        sprintf(outputMsg, "0 : %d : %s\r\n", bufferOverflow, ERROR_0);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(num==1)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
        sprintf(outputMsg, "1 : %d : %s\r\n", messageOverflow, ERROR_1);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(num==2)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
        sprintf(outputMsg, "2 : %d : %s\r\n", messageUnknown, ERROR_2);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(num==3)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
        sprintf(outputMsg, "3 : %d : %s\r\n", messageFail, ERROR_3);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(num==4)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
        sprintf(outputMsg, "4 : %d : %s\r\n", adcError, ERROR_4);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(num==5)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
        sprintf(outputMsg, "5 : %d : %s\r\n", udpNetworkError, ERROR_5);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(num==6)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
        sprintf(outputMsg, "6 : %d : %s\r\n", memrError, ERROR_6);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(num==7)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
        sprintf(outputMsg, "7 : %d : %s\r\n", queueOverflow, ERROR_7);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(num==8)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
        sprintf(outputMsg, "8 : %d : %s\r\n", voiceStreamingError, ERROR_8);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(num==9)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
        sprintf(outputMsg, "9 : %d : %s\r\n", parsingError, ERROR_9);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
        sprintf(outputMsg, "0 : %d : %s\r\n", bufferOverflow,  ERROR_0);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "1 : %d : %s\r\n", messageOverflow, ERROR_1);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "2 : %d : %s\r\n", messageUnknown,  ERROR_2);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "3 : %d : %s\r\n", messageFail,     ERROR_3);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "4 : %d : %s\r\n", adcError,        ERROR_4);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "5 : %d : %s\r\n", udpNetworkError, ERROR_5);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "6 : %d : %s\r\n", memrError,       ERROR_6);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "7 : %d : %s\r\n", queueOverflow,   ERROR_7);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "8 : %d : %s\r\n", voiceStreamingError, ERROR_8);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "9 : %d : %s\r\n", parsingError,    ERROR_9);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPgpio()
{
    int i;
    //UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
    for(i=0;i<8;i++)
    {
        val =GPIO_read(i);
        sprintf(outputMsg, "gpio %d : %d\r\n",i,val);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/

void *MPgpioSpace()
{
    int j=0;
    int i;
    char secondStr[50];//="\0";
    char thirdStr[50];//="\0";
    char fourthStr[50];//="\0";
    strClear(secondStr,50);
    strClear(thirdStr,50);
    strClear(fourthStr,50);
    for(i=(strlen("-gpio "));((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)//find if second str is present
    {
        secondStr[j]=typedMsg[i];
        j++;
    }
    secondStr[j]='\0';
    if(typedMsg[((strlen("-gpio "))+(strlen(secondStr)))]==' ')// if space after second string
    {
        num=atol(secondStr);
        j=0;
        for(i=((strlen("-gpio "))+(strlen(secondStr))+1/* 1 is needed to skip the space after 2nd str*/);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
        {
            thirdStr[j]=typedMsg[i];
            j++;
        }
        thirdStr[j]='\0';
        if (typedMsg[((strlen("-gpio "))+(strlen(secondStr))+(strlen(thirdStr)))+1]==' ')// +1 is to skip space after second str
        {
            j=0;
            for(i=((strlen("-gpio "))+(strlen(secondStr))+(strlen(thirdStr))+2/* 2 is needed to skip the 2 space after 2nd str and 3rd str*/);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
            {
                fourthStr[j]=typedMsg[i];
                j++;
            }
            fourthStr[j]='\0';
            val=atol(fourthStr);
        }
        if((num>=0) && (num<=7))
        {
            if(strcmp(thirdStr,"t")==0)
            {
                GPIO_toggle(num);
              //  UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
            }
            else if(strcmp(thirdStr,"w")==0)
            {
                if(val==1)
                {
                    GPIO_write(num,1);
                //    UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
                }
                else if(val==0)
                {
                    GPIO_write(num,0);
               //     UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
                }
                else if(val>1)
                {
                    GPIO_write(num,(val & 0x1));
                //    UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
                }
            }
            else
            {
               // UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
                sprintf(outputMsg, "type valid instruction 't' or 'w VAL' VAL= 0/1\r\n");
                UART_write(uart,    outputMsg,  strlen(outputMsg));
                parsingError++;
            }
        }
        else
        {
           // UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
            sprintf(outputMsg, "typed gpio number %d is invalid\r\n",num);
            UART_write(uart,    outputMsg,  strlen(outputMsg));
            parsingError++;
        }
        strClear(secondStr,50);
        strClear(thirdStr,50);
        strClear(fourthStr,50);
        return(0);
    }
    else
    {
        num = atol(secondStr); // num has the GPIO number
        if((num>=0) && (num<=7)) // check for gpio number is from 0 to 7
        {
            val = GPIO_read(num);
           // UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
            sprintf(outputMsg, "\r\ngpio %d : %d",num,val);
            UART_write(uart,    outputMsg,  strlen(outputMsg));
        }
        else if(!((num>=0) && (num<=7))) // if gpio number is not from 0 to 7
        {
            //UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
            sprintf(outputMsg, "\r\ntyped gpio number %s %d is invalid",secondStr,num);
            UART_write(uart,    outputMsg,  strlen(outputMsg));
            parsingError++;
        }
        strClear(secondStr,50);
        strClear(thirdStr,50);
        strClear(fourthStr,50);
        return(0);
    }
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPhelp()
{
    if(strCheckNext(typedMsg)==0) // second string not present
    {

        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
        sprintf(outputMsg, "%s\r\n",    HELP_ABOUT);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_CALLBACK);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_ERROR);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_HELP);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_MEMADDR);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_PRINT);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_REG);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_SCRIPT);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_TICKER);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_TIMER);
        UART_write(uart,    outputMsg,  strlen(outputMsg));

    }
    else if(strStartsWith(typedMsg,"-help about")==0)
    {
        sprintf(outputMsg, "\r\n-about\r\n%s\r\n",HELP_ABOUT_1);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(strStartsWith(typedMsg,"-help callback")==0)
    {
        sprintf(outputMsg, "\r\n%s\r\n",HELP_CALLBACK_1);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_CALLBACK_2);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_CALLBACK_3);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_CALLBACK_4);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_CALLBACK_5);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_CALLBACK_6);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_CALLBACK_7);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_CALLBACK_8);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_CALLBACK_9);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_CALLBACK_10);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(strStartsWith(typedMsg,"-help error")==0)
    {
        sprintf(outputMsg, "\r\n%s\r\n",HELP_ERROR_0);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_ERROR_1);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_ERROR_2);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_ERROR_3);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_ERROR_4);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_ERROR_5);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_ERROR_6);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_ERROR_7);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_ERROR_8);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_ERROR_9);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_ERROR_10);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_ERROR_11);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_ERROR_12);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(strStartsWith(typedMsg,"-help gpio")==0)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));// one empty line
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO_1);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO_2);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO_3);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO_4);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO_5);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO_6);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO_7);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO_8);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO_9);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO_10);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO_11);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO_12);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO_13);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_GPIO_14);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(strStartsWith(typedMsg,"-help help")==0)
    {
        sprintf(outputMsg, "\r\n%s\r\n",HELP_HELP_1);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_HELP_2);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_HELP_3);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_HELP_4);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(strStartsWith(typedMsg,"-help if")==0)
    {
        UART_write(uart,    HELP_IF_1,  strlen(HELP_IF_1));
        UART_write(uart,    HELP_IF_2,  strlen(HELP_IF_2));
        UART_write(uart,    HELP_IF_3,  strlen(HELP_IF_3));
        UART_write(uart,    HELP_IF_4,  strlen(HELP_IF_4));
        UART_write(uart,    HELP_IF_5,  strlen(HELP_IF_5));
        UART_write(uart,    HELP_IF_6,  strlen(HELP_IF_6));
        UART_write(uart,    HELP_IF_7,  strlen(HELP_IF_7));
        UART_write(uart,    HELP_IF_8,  strlen(HELP_IF_8));
        UART_write(uart,    HELP_IF_9,  strlen(HELP_IF_9));
        UART_write(uart,    HELP_IF_10,  strlen(HELP_IF_10));
        UART_write(uart,    HELP_IF_11,  strlen(HELP_IF_11));
        UART_write(uart,    HELP_IF_12,  strlen(HELP_IF_12));
    }

    else if(strStartsWith(typedMsg,"-help memr")==0)
    {
        sprintf(outputMsg, "\r\n%s\r\n",HELP_MEMR_1);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_MEMR_2);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_MEMR_3);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_MEMR_4);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_MEMR_5);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(strStartsWith(typedMsg,"-help print")==0)
    {
        sprintf(outputMsg, "\r\n%s\r\n",HELP_PRINT_1);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_PRINT_2);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_PRINT_3);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else if(strStartsWith(typedMsg,"-help reg")==0)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
        UART_write(uart, HELP_REG_1,strlen(HELP_REG_1));
        UART_write(uart, HELP_REG_2,strlen(HELP_REG_2));
        UART_write(uart, HELP_REG_3,strlen(HELP_REG_3));
        UART_write(uart, HELP_REG_4,strlen(HELP_REG_4));
        UART_write(uart, HELP_REG_5,strlen(HELP_REG_5));
        UART_write(uart, HELP_REG_6,strlen(HELP_REG_6));
        UART_write(uart, HELP_REG_7,strlen(HELP_REG_7));
        UART_write(uart, HELP_REG_8,strlen(HELP_REG_8));
        UART_write(uart, HELP_REG_9,strlen(HELP_REG_9));
        UART_write(uart, HELP_REG_10,strlen(HELP_REG_10));
        UART_write(uart, HELP_REG_11,strlen(HELP_REG_11));
        UART_write(uart, HELP_REG_12,strlen(HELP_REG_12));
        UART_write(uart, HELP_REG_13,strlen(HELP_REG_13));
        UART_write(uart, HELP_REG_14,strlen(HELP_REG_14));
        UART_write(uart, HELP_REG_15,strlen(HELP_REG_15));

    }
    else if(strStartsWith(typedMsg,"-help script")==0)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
        UART_write(uart, HELP_SCRIPT_1,strlen(HELP_SCRIPT_1));
        UART_write(uart, HELP_SCRIPT_2,strlen(HELP_SCRIPT_2));
        UART_write(uart, HELP_SCRIPT_3,strlen(HELP_SCRIPT_3));
        UART_write(uart, HELP_SCRIPT_4,strlen(HELP_SCRIPT_4));
        UART_write(uart, HELP_SCRIPT_5,strlen(HELP_SCRIPT_5));
        UART_write(uart, HELP_SCRIPT_6,strlen(HELP_SCRIPT_6));
        UART_write(uart, HELP_SCRIPT_7,strlen(HELP_SCRIPT_6));
    }
    else if(strStartsWith(typedMsg,"-help ticker")==0)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
        UART_write(uart, HELP_TICKER_1,strlen(HELP_TICKER_1));
        UART_write(uart, HELP_TICKER_2,strlen(HELP_TICKER_2));
        UART_write(uart, HELP_TICKER_3,strlen(HELP_TICKER_3));
        UART_write(uart, HELP_TICKER_4,strlen(HELP_TICKER_4));
        UART_write(uart, HELP_TICKER_5,strlen(HELP_TICKER_5));
        UART_write(uart, HELP_TICKER_6,strlen(HELP_TICKER_6));
        UART_write(uart, HELP_TICKER_7,strlen(HELP_TICKER_7));
        UART_write(uart, HELP_TICKER_8,strlen(HELP_TICKER_8));
        UART_write(uart, HELP_TICKER_9,strlen(HELP_TICKER_9));
        UART_write(uart, HELP_TICKER_10,strlen(HELP_TICKER_10));
        UART_write(uart, HELP_TICKER_11,strlen(HELP_TICKER_11));
        UART_write(uart, HELP_TICKER_12,strlen(HELP_TICKER_12));
        UART_write(uart, HELP_TICKER_13,strlen(HELP_TICKER_13));
    }
    else if(strStartsWith(typedMsg,"-help timer")==0)
    {
        sprintf(outputMsg, "\r\n%s\r\n",HELP_TIMER_1);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_TIMER_2);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_TIMER_3);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_TIMER_4);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
        sprintf(outputMsg, "%s\r\n",    HELP_TIMER_5);
        UART_write(uart,    outputMsg,  strlen(outputMsg));
    }
    else
    {
        int i;
        int j=0;
        for(i=(strlen("-help")); i<=typedMsgLen; i++)
        {
            printOutput[j]=typedMsg[i];
            j++;
        }// the above code saves all the charac after "-help" and same will be used to print in below code lines
        sprintf(outputMsg, "\r\ncannot help junk: %s\r\n",printOutput);
        UART_write(uart,outputMsg,strlen(outputMsg));
        parsingError++;
    }
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPif()
{
    int i=0,j=0,SAnumPresent=0,SBnumPresent=0,SA=0,SB=0,condition=0;
    int SAnum=0;
    int SBnum=0;
    char secondStr[50];
    char thirdStr[50];
    char fourthStr[50];
    char fifthStr[50];
    char sixthStr[50];
    char seventhStr[50];
    char eighthStr[50];
    char OP[10];
    char SAbuff[10];
    char SBbuff[10];
    char funT[50];
    char funF[50];
    strClear(secondStr,50);
    strClear(thirdStr,50);
    strClear(fourthStr,50);
    strClear(fifthStr,50);
    strClear(sixthStr,50);
    strClear(seventhStr,50);
    strClear(eighthStr,50);
    strClear(OP,10);
    strClear(SAbuff,10);
    strClear(SBbuff,10);
    strClear(funT,50);
    strClear(funF,50);

    if(typedMsgLen==3)
    {
        sprintf(outputMsg,"\r\n-if fail\r\n");
        UART_write(uart,outputMsg,strlen(outputMsg));
        messageFail++;
        return 0;
    }
    for(i=(strlen("-if "));((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)//find if second str is present
    {
        secondStr[j]=typedMsg[i];
        j++;
    }
    secondStr[j]='\0';
    if(typedMsg[((strlen("-if "))+(strlen(secondStr)))]==' ')// if space after second string
    {
        j=0;
        for(i=((strlen("-if "))+(strlen(secondStr))+1/* 1 is needed to skip the space after 2nd str*/);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
        {
            thirdStr[j]=typedMsg[i];
            j++;
        }
        thirdStr[j]='\0';
        if (typedMsg[((strlen("-if "))+(strlen(secondStr))+(strlen(thirdStr)))+1]==' ')// +1 is to skip space after second str
        {
            j=0;
            for(i=((strlen("-if "))+(strlen(secondStr))+(strlen(thirdStr))+2/* 2 is needed to skip the 2 space after 2nd str and 3rd str*/);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
            {
                fourthStr[j]=typedMsg[i];
                j++;
            }
            fourthStr[j]='\0';
        }
        if (typedMsg[((strlen("-if "))+(strlen(secondStr))+(strlen(thirdStr))+(strlen(fourthStr)))+2]==' ')
        {
            j=0;
            for(i=(((strlen("-if "))+(strlen(secondStr))+(strlen(thirdStr))+(strlen(fourthStr)))+3);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
            {
                fifthStr[j]=typedMsg[i];
                j++;
            }
            fifthStr[j]='\0';
        }
        if (typedMsg[((strlen("-if "))+(strlen(secondStr))+(strlen(thirdStr))+(strlen(fourthStr))+(strlen(fifthStr)))+3]==' ')
        {
            j=0;
            for(i=(((strlen("-if "))+(strlen(secondStr))+(strlen(thirdStr))+(strlen(fourthStr))+(strlen(fifthStr)))+4);((typedMsg[i]!=':') &&(i<typedMsgLen)); i++)
            {
                sixthStr[j]=typedMsg[i];
                j++;
            }
            sixthStr[j-1]='\0';
        }
        if (typedMsg[((strlen("-if "))+(strlen(secondStr))+(strlen(thirdStr))+(strlen(fourthStr))+(strlen(fifthStr))+(strlen(sixthStr)))+4]==' ')
        {
            j=0;
            for(i=(((strlen("-if "))+(strlen(secondStr))+(strlen(thirdStr))+(strlen(fourthStr))+(strlen(fifthStr))+(strlen(sixthStr)))+5);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
            {
                seventhStr[j]=typedMsg[i];
                j++;
            }
            seventhStr[j]='\0';
        }
        if (typedMsg[((strlen("-if "))+(strlen(secondStr))+(strlen(thirdStr))+(strlen(fourthStr))+(strlen(fifthStr))+(strlen(sixthStr))+(strlen(seventhStr)))+5]==' ')
        {
            j=0;
            for(i=(((strlen("-if "))+(strlen(secondStr))+(strlen(thirdStr))+(strlen(fourthStr))+(strlen(fifthStr))+(strlen(sixthStr))+(strlen(seventhStr)))+6);(i<typedMsgLen); i++)
            {
                eighthStr[j]=typedMsg[i];
                j++;
            }
            eighthStr[j]='\0';
        }
        if((strlen(secondStr)==0)||(strlen(thirdStr)==0)||(strlen(fourthStr)==0)||(strlen(fifthStr)==0))
        {
            sprintf(outputMsg,"\r\ninvalid/missing instructions for -if operations\r\n");
            UART_write(uart,outputMsg,strlen(outputMsg));
            parsingError++;
            //UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
            return 0;
        }
    }
    strcpy(OP,thirdStr);
    if(secondStr[0]=='#')
    {
        j=0;
        for(i=1;i<strlen(secondStr);i++)
        {
            SAbuff[j]=secondStr[i];
            j++;
        }
        SAnum=atoi(SAbuff);
        SAnumPresent=1;
        strClear(SAbuff,10);
    }
    else
    {
        SAnumPresent=0;
        SA=atoi(secondStr);
    }
    if(fourthStr[0]=='#')
    {
        j=0;
        for(i=1;i<strlen(fourthStr);i++)
        {
            SBbuff[j]=fourthStr[i];
            j++;
        }
        SBnum=atoi(SBbuff);
        SBnumPresent=1;
        strClear(SBbuff,10);
    }
    else
    {
        SBnumPresent=0;
        SB=atoi(fourthStr);
    }
    if(sixthStr==":")
        strcpy(funT,"\0");
    else
        strcpy(funT,sixthStr);
    if(strlen(eighthStr)==0)
        strcpy(funF,"\0");
    else
        strcpy(funF,eighthStr);
    if(strStartsWith(OP,">")==0)
    {
        condition=((SAnumPresent==0)?reg[SA]:SAnum) > ((SBnumPresent==0)?reg[SB]:SBnum);
    }
    else if(strStartsWith(OP,"=")==0)
    {
        condition=((SAnumPresent==0)?reg[SA]:SAnum) == ((SBnumPresent==0)?reg[SB]:SBnum);
    }
    else if(strStartsWith(OP,"<")==0)
    {
        condition=((SAnumPresent==0)?reg[SA]:SAnum) < ((SBnumPresent==0)?reg[SB]:SBnum);
    }
    if(condition==1)
        AddPayload(funT);
    else
        AddPayload(funF);
//    sprintf(outputMsg,"\r\nsecondStr:%s\r\n",secondStr);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"thirdStr:%s\r\n",thirdStr);
 //   UART_write(uart,outputMsg,strlen(outputMsg));
 //   sprintf(outputMsg,"fourthStr:%s\r\n",fourthStr);
 //   UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"fifthStr:%s\r\n",fifthStr);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"sixthStr:%s\r\n",sixthStr);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"seventhStr:%s\r\n",seventhStr);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"eighthStr:%s\r\n",eighthStr);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"SA:%d\r\n",SA);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"SB:%d\r\n",SB);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"SAbuff:%s\r\n",SAbuff);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"SBbuff:%s\r\n",SBbuff);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"SAnum:%d\r\n",SAnum);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"SBnum:%d\r\n",SBnum);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"SAnumPresent:%d\r\n",SAnumPresent);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"SBnumPresent:%d\r\n",SBnumPresent);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"OP:%s\r\n",OP);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"condition:%d\r\n",condition);
//    UART_write(uart,outputMsg,strlen(outputMsg));
//    sprintf(outputMsg,"funT:%s\r\n",funT);
//    UART_write(uart,outputMsg,strlen(outputMsg));
   // UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
    return 0;
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPmemr()
{
    int i;
    memlocAddr=(uint32_t*)0;
    sprintf(outputMsg, "\r\n0x%08X 0x%08X 0x%08X 0x%08X\r\n",memlocAddr,memlocAddr+1,memlocAddr+2,memlocAddr+3);
    UART_write(uart,    outputMsg, strlen(outputMsg)); // to print 4 address in a line
    for(i=0;i<4;i++)
    {
        memlocData=*memlocAddr;
        sprintf(outputMsg, "0x%08X ",memlocData);
        UART_write(uart,    outputMsg,   strlen(outputMsg));
        memlocAddr++; // to print data in 4 address in a line
    }
    UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPmemrSpace()
{
    int i;
    int j=0;
    for(i=(strlen("-memr ")); i<=typedMsgLen; i++)
    {
        printOutput[j]=typedMsg[i];
        j++;
    }// the above code saves all the charac after "-memr" and same will be used to print in below code lines
    memloc=strtol(printOutput,&dummy,16);// converting char values into intiger using string to long.
    memlocAddr=(uint32_t*)memloc;        // saving intiger value as address into a pointer
    if(((0x00000000<=memlocAddr)&&(memlocAddr<=(0x000FFFFF-0x0000000F))) || ((0x20000000<=memlocAddr)&&(memlocAddr<=(0x2003FFFF-0x0000000F))))
    {//if all 4 quads of memory locations are inside the flash or SRAM else data cannot be fetched.
        sprintf(outputMsg, "\r\n0x%08X 0x%08X 0x%08X 0x%08X\r\n",memlocAddr,memlocAddr+1,memlocAddr+2,memlocAddr+3);
        UART_write(uart,    outputMsg, strlen(outputMsg)); // to print 4 address in a line
        for(i=0;i<4;i++)
        {
            memlocData=*memlocAddr;
            sprintf(outputMsg, "0x%08X ",memlocData);
            UART_write(uart,    outputMsg,   strlen(outputMsg));
            memlocAddr++; // to print data in 4 address in a line
        }
      //  UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
    }
    else
    {//if all 4 quads of memory locations are not inside the flash or SRAM else data cannot be fetched.Print invalid message.
       // UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
        UART_write(uart,    MEMR_INVLD,sizeof(MEMR_INVLD));
        memrError++;
    }
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPmsgNotUnderstood()
{
    sprintf(outputMsg, "\r\ntyped message ''%s'' not understood\r\n",typedMsg);
    UART_write(uart, outputMsg, strlen(outputMsg));
    messageUnknown++;
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPoverflow()
{
    UART_write(uart, overFlowPromt, sizeof(overFlowPromt));
    bufferOverflow++;
    typedMsgLen=0;
    typedMsg[typedMsgLen]=0;// after overflow is detected, the typedMsg and its length is reset to 0.
                            // allowing typedMsg to start storing new typed message from index 0.

    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPprint()
{
    int i;
    int j=0;
    if(typedMsgLen==6)
    {
        sprintf(outputMsg,"\r\n-print fail\r\n");
        UART_write(uart,outputMsg,strlen(outputMsg));
        messageFail++;
        return 0;
    }
    if(typedMsg[strlen("-print")]!=' ')
    {
        sprintf(outputMsg,"\r\n-print fail: needed space after -print and before printable message\r\n");
        UART_write(uart,outputMsg,strlen(outputMsg));
        messageFail++;
        return 0;
    }
    for(i=(strlen("-print ")); i<=typedMsgLen; i++)
    {
        printOutput[j]=typedMsg[i];
        j++;
    }// the above code saves all the charac after "-print" and same will be used to print in below code lines
    sprintf(outputMsg, "\r\n%s\r\n",printOutput);
    UART_write(uart,outputMsg,strlen(outputMsg));
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPreg()
{
    int i;
    for(i=0;i<regCount;i++)
    {
        //UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
        sprintf(outputMsg,"\r\nREG %2d %3d",i,reg[i]);
        UART_write(uart, outputMsg,strlen(outputMsg));
    }
    //UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
    return 0;
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPregSpace()
{
    int i=0,j=0,DST=0,SAnumPresent=0,SBnumPresent=0,SA=0,SB=0;
    int SAnum=0;
    int SBnum=0;
    uint32_t tempReg;
    char secondStr[50];
    char thirdStr[50];
    char fourthStr[50];
    char fifthStr[50];
    char OP[10];
    char SAbuff[10];
    char SBbuff[10];
    strClear(secondStr,50);
    strClear(thirdStr,50);
    strClear(fourthStr,50);
    strClear(fifthStr,50);
    strClear(OP,10);
    strClear(SAbuff,10);
    strClear(SBbuff,10);

    for(i=(strlen("-reg "));((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)//find if second str is present
    {
        secondStr[j]=typedMsg[i];
        j++;
    }
    secondStr[j]='\0';
    DST=atol(secondStr);
    if(typedMsg[((strlen("-reg "))+(strlen(secondStr)))]==' ')// if space after second string
    {
        j=0;
        for(i=((strlen("-reg "))+(strlen(secondStr))+1/* 1 is needed to skip the space after 2nd str*/);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
        {
            thirdStr[j]=typedMsg[i];
            j++;
        }
        thirdStr[j]='\0';
        if (typedMsg[((strlen("-reg "))+(strlen(secondStr))+(strlen(thirdStr)))+1]==' ')// +1 is to skip space after second str
        {
            j=0;
            for(i=((strlen("-reg "))+(strlen(secondStr))+(strlen(thirdStr))+2/* 2 is needed to skip the 2 space after 2nd str and 3rd str*/);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
            {
                fourthStr[j]=typedMsg[i];
                j++;
            }
            fourthStr[j]='\0';
        }
        if (typedMsg[((strlen("-reg "))+(strlen(secondStr))+(strlen(thirdStr))+(strlen(fourthStr)))+2]==' ')
        {
            j=0;
            for(i=(((strlen("-reg "))+(strlen(secondStr))+(strlen(thirdStr))+(strlen(fourthStr)))+3);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
            {
                fifthStr[j]=typedMsg[i];
                j++;
            }
            fifthStr[j]='\0';
        }
        if(((strlen(thirdStr)==0)||(strlen(fourthStr)==0)||(strlen(fifthStr)==0))&&(((strStartsWith(OP,"?")==0)||(strStartsWith(OP,"=")==0)||(strStartsWith(OP,"x")==0)||(strStartsWith(OP,"c")==0))))
        {
            sprintf(outputMsg,"\r\ninvalid/missing instructions for reg operations\r\n");
            UART_write(uart,outputMsg,strlen(outputMsg));
            parsingError++;
           // UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
            return 0;
        }
        else
        {
            strcpy(OP,thirdStr);
            if(fourthStr[0]=='#')
            {
                j=0;
                for(i=1;i<strlen(fourthStr);i++)
                {
                    SAbuff[j]=fourthStr[i];
                    j++;
                }
                SAnum=atoi(SAbuff);
                SAnumPresent=1;
                strClear(SAbuff,10);
            }
            else
            {
                SAnumPresent=0;
                SA=atoi(fourthStr);
            }
            if(fifthStr[0]=='#')
            {
                j=0;
                for(i=1;i<strlen(fifthStr);i++)
                {
                    SBbuff[j]=fifthStr[i];
                    j++;
                }
                SBnum=atoi(SBbuff);
                SBnumPresent=1;
                strClear(SBbuff,10);
            }
            else
            {
                SBnumPresent=0;
                SB=atoi(fifthStr);
            }
            if((strStartsWith(OP,"+")==0)&&(strlen(OP)==1))
            {
                reg[DST]=((SAnumPresent==0)?reg[SA]:SAnum) + ((SBnumPresent==0)?reg[SB]:SBnum);
            }
            else if((strStartsWith(OP,"-")==0)&&(strlen(OP)==1))
            {
                reg[DST]=((SAnumPresent==0)?reg[SA]:SAnum) - ((SBnumPresent==0)?reg[SB]:SBnum);
            }
            else if(strStartsWith(OP,"*")==0)
            {
                reg[DST]=((SAnumPresent==0)?reg[SA]:SAnum) * ((SBnumPresent==0)?reg[SB]:SBnum);
            }
            else if(strStartsWith(OP,"/")==0)
            {
                if(((SBnumPresent==0)?reg[SB]:SBnum)==0)
                {
                   // sprintf(outputMsg,"\r\ndivision by 0 is not allowed\r\n");
                   // UART_write(uart, outputMsg,strlen(outputMsg));
                    reg[DST]=0x7FFFFFFF;
                    //return 0;
                }
                else
                    reg[DST]=((SAnumPresent==0)?reg[SA]:SAnum) / ((SBnumPresent==0)?reg[SB]:SBnum);
            }
            else if(strStartsWith(OP,"%")==0)
            {
                if(((SBnumPresent==0)?reg[SB]:SBnum)==0)
                {
                    zeroError++;
                    sprintf(outputMsg,"\r\ndivision by 0 is not allowed\r\n");
                    UART_write(uart, outputMsg,strlen(outputMsg));
                    return 0;
                }
                else
                    reg[DST]=((SAnumPresent==0)?reg[SA]:SAnum) % ((SBnumPresent==0)?reg[SB]:SBnum);
            }
            else if(strStartsWith(OP,"&")==0)
            {
                reg[DST]=((SAnumPresent==0)?reg[SA]:SAnum) & ((SBnumPresent==0)?reg[SB]:SBnum);
            }
            else if(strStartsWith(OP,"|")==0)
            {
                reg[DST]=((SAnumPresent==0)?reg[SA]:SAnum) | ((SBnumPresent==0)?reg[SB]:SBnum);
            }
            else if(strStartsWith(OP,"^")==0)
            {
                reg[DST]=((SAnumPresent==0)?reg[SA]:SAnum) ^ ((SBnumPresent==0)?reg[SB]:SBnum);
            }
            else if(strStartsWith(OP,"^")==0)
            {
                reg[DST]=((SAnumPresent==0)?reg[SA]:SAnum) ^ ((SBnumPresent==0)?reg[SB]:SBnum);
            }
            else if(strStartsWith(OP,">")==0)
            {
                reg[DST]=(((SAnumPresent==0)?reg[SA]:SAnum) > ((SBnumPresent==0)?reg[SB]:SBnum))?((SAnumPresent==0)?reg[SA]:SAnum):((SBnumPresent==0)?reg[SB]:SBnum);
            }
            else if(strStartsWith(OP,"<")==0)
            {
                reg[DST]=(((SAnumPresent==0)?reg[SA]:SAnum) < ((SBnumPresent==0)?reg[SB]:SBnum))?((SAnumPresent==0)?reg[SA]:SAnum):((SBnumPresent==0)?reg[SB]:SBnum);
            }
            else if(strStartsWith(OP,"++")==0)
            {
                reg[DST]=((SAnumPresent==0)?reg[SA]:SAnum)+1;
            }
            else if(strStartsWith(OP,"--")==0)
            {
                reg[DST]=((SAnumPresent==0)?reg[SA]:SAnum)-1;
            }
            else if((strStartsWith(OP,"?")==0)||(strStartsWith(OP,"r")==0))
            {
                goto end;
            }
            else if(strStartsWith(OP,"~")==0)
            {
                reg[DST]=~((SAnumPresent==0)?reg[SA]:SAnum);
            }
            else if(strStartsWith(OP,"_")==0)
            {
                tempReg=~((SAnumPresent==0)?reg[SA]:SAnum);
                reg[DST]=tempReg+1;
            }
            else if((strStartsWith(OP,"=")==0)||(strStartsWith(OP,"c")==0)||(strStartsWith(OP,"w")==0))
            {
                reg[DST]=((SAnumPresent==0)?reg[SA]:SAnum) ;
            }
            else if(strStartsWith(OP,"x")==0)
            {
                if(SAnumPresent==1)
                {
                    sprintf(outputMsg,"\r\nsecond source for exchange should be register not a number");
                    UART_write(uart, outputMsg,strlen(outputMsg));
                    return 0;
                }
                else
                {
                    tempReg=reg[SA];
                    reg[SA]=reg[DST];
                    reg[DST]=tempReg;
                    sprintf(outputMsg,"\r\nREG %2d %3d\r\n",SA,reg[SA]);
                    UART_write(uart, outputMsg,strlen(outputMsg));
                }
            }
            else
            {
                sprintf(outputMsg,"\r\nno operator found in the command");
                UART_write(uart, outputMsg,strlen(outputMsg));
                return 0;
            }
end:        sprintf(outputMsg,"\r\nREG %2d %3d\r\n",DST,reg[DST]);
            UART_write(uart, outputMsg,strlen(outputMsg));
        }
    }
    else
    {
        if((DST==-1)||(DST>31))
        {
            MPreg();
        }
        else if((DST>=0)&&(DST<=31))
        {
            sprintf(outputMsg,"\r\nREG %2d %3d\r\n",DST,reg[DST]);
            UART_write(uart, outputMsg,strlen(outputMsg));
        }
    }
//sprintf(outputMsg,"\r\nsecondStr:%s\r\n",secondStr);
//UART_write(uart,outputMsg,strlen(outputMsg));
//sprintf(outputMsg,"thirdStr:%s\r\n",thirdStr);
//UART_write(uart,outputMsg,strlen(outputMsg));
//sprintf(outputMsg,"fourthStr:%s\r\n",fourthStr);
//UART_write(uart,outputMsg,strlen(outputMsg));
//sprintf(outputMsg,"fifthStr:%s\r\n",fifthStr);
//UART_write(uart,outputMsg,strlen(outputMsg));
//sprintf(outputMsg,"SA:%d\r\n",SA);
//UART_write(uart,outputMsg,strlen(outputMsg));
//sprintf(outputMsg,"SB:%d\r\n",SB);
//UART_write(uart,outputMsg,strlen(outputMsg));
//sprintf(outputMsg,"SAbuff:%s\r\n",SAbuff);
//UART_write(uart,outputMsg,strlen(outputMsg));
//sprintf(outputMsg,"SBbuff:%s\r\n",SBbuff);
//UART_write(uart,outputMsg,strlen(outputMsg));
//sprintf(outputMsg,"SAnum:%d\r\n",SAnum);
//UART_write(uart,outputMsg,strlen(outputMsg));
//sprintf(outputMsg,"SBnum:%d\r\n",SBnum);
//UART_write(uart,outputMsg,strlen(outputMsg));
//sprintf(outputMsg,"SAnumPresent:%d\r\n",SAnumPresent);
//UART_write(uart,outputMsg,strlen(outputMsg));
//sprintf(outputMsg,"SBnumPresent:%d\r\n",SBnumPresent);
//UART_write(uart,outputMsg,strlen(outputMsg));
//sprintf(outputMsg,"DST:%d\r\n",DST);
//UART_write(uart,outputMsg,strlen(outputMsg));
//sprintf(outputMsg,"\r\nOP:%s",OP);
//UART_write(uart,outputMsg,strlen(outputMsg));
//UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
return 0;
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPscript()
{
    int i;
    for(i=0;i<scriptCount;i++)
    {
        //UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
        sprintf(outputMsg,"\r\nscript %2d %s",i,script[i]);
        UART_write(uart, outputMsg,strlen(outputMsg));
    }
    UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
    return 0;
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPscriptSpace()
{
    int i,j=0;
    char secondStr[50];
    char thirdStr[50];
    char fourthStr[320];
    strClear(secondStr,50);
    strClear(thirdStr,50);
    strClear(fourthStr,320);
    for(i=(strlen("-script "));((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)//find if second str is present
    {
        secondStr[j]=typedMsg[i];
        j++;
    }
    secondStr[j]='\0';
    scriptNum=atol(secondStr);
    if(typedMsg[((strlen("-script "))+(strlen(secondStr)))]==' ')// if space after second string
    {
        j=0;
        for(i=((strlen("-script "))+(strlen(secondStr))+1/* 1 is needed to skip the space after 2nd str*/);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
        {
            thirdStr[j]=typedMsg[i];
            j++;
        }
        thirdStr[j]='\0';
        if(((strStartsWith(thirdStr,"w")==0)||(strStartsWith(thirdStr,"W")==0))&&(strlen(thirdStr)==1))
        {
            if (typedMsg[((strlen("-script "))+(strlen(secondStr))+(strlen(thirdStr)))+1]==' ')// +1 is to skip space after second str
            {
                j=0;
                for(i=((strlen("-script "))+(strlen(secondStr))+(strlen(thirdStr))+2/* 2 is needed to skip the 2 space after 2nd str and 3rd str*/);(i<typedMsgLen); i++)
                {
                    fourthStr[j]=typedMsg[i];
                    j++;
                }
                fourthStr[j]='\0';
            }
            if(strlen(fourthStr)==0)
            {
                sprintf(outputMsg,"\r\nmissing write string after 'w' for script write operation\r\n");
                UART_write(uart,outputMsg,strlen(outputMsg));
                parsingError++;
               // UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
                return 0;
            }
        }
        if(((strStartsWith(thirdStr,"w")==0)||(strStartsWith(thirdStr,"W")==0))&&(strlen(thirdStr)==1))
        {
            strClear(script[scriptNum],msgMaxLen);
            strcpy(script[scriptNum],fourthStr);
            sprintf(outputMsg,"\r\nscript %d %s\r\n",scriptNum,script[scriptNum]);
            UART_write(uart,outputMsg,strlen(outputMsg));
        }
        else if((strStartsWith(thirdStr,"r")==0)&&(strlen(thirdStr)==1))
        {
            sprintf(outputMsg,"\r\nscript %d %s\r\n",scriptNum,script[scriptNum]);
            UART_write(uart,outputMsg,strlen(outputMsg));
        }
        else if(((strStartsWith(thirdStr,"c")==0)||(strStartsWith(thirdStr,"C")==0))&&(strlen(thirdStr)==1))
        {
            strClear(script[scriptNum],msgMaxLen);
            sprintf(outputMsg,"\r\nscript %d %s\r\n",scriptNum,script[scriptNum]);
            UART_write(uart,outputMsg,strlen(outputMsg));
        }
        else
        {
            sprintf(outputMsg,"\r\nmissing instruction 'w' for script write operation\r\n");
            UART_write(uart,outputMsg,strlen(outputMsg));
            parsingError++;
        }
    }
    else if((scriptNum==-1)||(scriptNum>63))
        MPscript();
    else
    {
       // UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
        while(script[scriptNum][0]!='\0' && scriptNum<scriptCount)
        {
            // gatekey= GateSwi_enter(gateSwi0);
            // Queue_enqueue(myQ, &(r[wIndex].elem));
            // strcpy(ParseQueue.payloads[ParseQueue.payloadWriting],script[scriptNum]);
            // ParseQueue.payloadWriting++;
            // if(ParseQueue.payloadWriting>31)
            //     ParseQueue.payloadWriting=0;
            // Semaphore_post(semaphore0);
            // GateSwi_leave(gateSwi0,gatekey);
            //  sprintf(outputMsg,"script[%d]:%s queuestr:%s\r\n",scriptNum,script[scriptNum],r[wIndex].payload);
            //  UART_write(uart,outputMsg,strlen(outputMsg));
            AddPayload(script[scriptNum]);
           // sprintf(outputMsg,"script %d %s\r\n",scriptNum,script[scriptNum]);
           // UART_write(uart,outputMsg,strlen(outputMsg));
            scriptNum++;
        }
    }
    UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
    return 0;
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPticker()
{
    int i;
    for(i=0;i<tickerCount;i++)
    {
        UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
        sprintf(outputMsg,"ticker %2d   DEL:%d PER:%d CNT:%2d FUN:%s\r",i,ticker[i].delay,ticker[i].period,ticker[i].count,ticker[i].function);
        UART_write(uart, outputMsg,strlen(outputMsg));
    }
    UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
    return 0;
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPtickerKill()
{
    int i;
    for(i=0;i<tickerCount;i++)
    {
        ticker[i].delay=0;
        ticker[i].period=0;
        ticker[i].count=0;
        strClear(ticker[i].function,msgMaxLen);
    }
    //UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
   return 0;
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPtickerSpace()
{
    int i;
    int j=0;
    char secondStr[50];
    char thirdStr[50];
    char fourthStr[50];
    char fifthStr[50];
    char sixthStr[320];
    strClear(secondStr,50);
    strClear(thirdStr,50);
    strClear(fourthStr,50);
    strClear(fifthStr,50);
    strClear(sixthStr,320);
    for(i=(strlen("-ticker "));((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)//find if second str is present
    {
        secondStr[j]=typedMsg[i];
        j++;
    }
    secondStr[j]='\0';
    tickerNum=atol(secondStr);
    //no third and fourth string in the typedMsg(example: typedMsg is only "-ticker #")
    // and there is no space after second string
   // else if(!(typedMsg[((strlen("-ticker "))+(strlen(secondStr)))]==' '))
    if(!(typedMsg[((strlen("-ticker "))+(strlen(secondStr)))]==' '))
    {
        if((tickerNum==-1)||(tickerNum>15))
        {
            MPticker();
        }
        else if((tickerNum>=0)&&(tickerNum<=15))
        {
            sprintf(outputMsg,"\r\nticker %d  DEL:%d PER:%d CNT:%d FUN:%s\r\n",tickerNum,ticker[tickerNum].delay,ticker[tickerNum].period,ticker[tickerNum].count,ticker[tickerNum].function);
            UART_write(uart, outputMsg,strlen(outputMsg));
        }
    }
    else
    {
        if(typedMsg[((strlen("-ticker "))+(strlen(secondStr)))]==' ')// if space after second string
        {
            j=0;
            for(i=((strlen("-ticker "))+(strlen(secondStr))+1/* 1 is needed to skip the space after 2nd str*/);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
            {
                thirdStr[j]=typedMsg[i];
                j++;
            }
            thirdStr[j]='\0';
            if (typedMsg[((strlen("-ticker "))+(strlen(secondStr))+(strlen(thirdStr)))+1]==' ')// +1 is to skip space after second str
            {
                j=0;
                for(i=((strlen("-ticker "))+(strlen(secondStr))+(strlen(thirdStr))+2/* 2 is needed to skip the 2 space after 2nd str and 3rd str*/);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
                {
                    fourthStr[j]=typedMsg[i];
                    j++;
                }
                fourthStr[j]='\0';
                if (typedMsg[((strlen("-ticker "))+(strlen(secondStr))+(strlen(thirdStr))+(strlen(fourthStr)))+2]==' ')
                {
                    j=0;
                    for(i=(((strlen("-ticker "))+(strlen(secondStr))+(strlen(thirdStr))+(strlen(fourthStr)))+3);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
                    {
                        fifthStr[j]=typedMsg[i];
                        j++;
                    }
                    fifthStr[j]='\0';
                    if(typedMsg[((strlen("-ticker "))+(strlen(secondStr))+(strlen(thirdStr))+(strlen(fourthStr))+strlen(fifthStr))+3]==' ')
                    {
                        j=0;
                        for(i=(((strlen("-ticker "))+(strlen(secondStr))+(strlen(thirdStr))+(strlen(fourthStr))+strlen(fifthStr))+4);i<typedMsgLen;i++)
                        {
                            sixthStr[j]=typedMsg[i];
                            j++;
                        }
                        sixthStr[j]='\0';
                    }
                    else
                    {
                        parsingError++;
                        sprintf(outputMsg,"\r\ninvalid instruction\r\n");
                        UART_write(uart,outputMsg,strlen(outputMsg));
                    }
                }
                else
                {
                    parsingError++;
                    sprintf(outputMsg,"\r\ninvalid instruction\r\n");
                    UART_write(uart,outputMsg,strlen(outputMsg));
                }
            }
            else
            {
                parsingError++;
                sprintf(outputMsg,"\r\ninvalid instruction\r\n");
                UART_write(uart,outputMsg,strlen(outputMsg));
            }
        }
        ticker[tickerNum].delay=atol(thirdStr);
        ticker[tickerNum].period=atol(fourthStr);
        ticker[tickerNum].count=atol(fifthStr);
        strcpy(ticker[tickerNum].function,sixthStr);
      //  sprintf(outputMsg,"\r\n ticker %d DEL:%d PER:%d CNT:%d FUN:%s",tickerNum,ticker[tickerNum].delay,ticker[tickerNum].period,ticker[tickerNum].count,ticker[tickerNum].function);
      //  UART_write(uart,outputMsg,strlen(outputMsg));
        //UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
        strClear(secondStr,50);
        strClear(thirdStr,50);
        strClear(fourthStr,50);
        strClear(fifthStr,50);
        strClear(sixthStr,320);
    }
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPtimer()
{
    if(timer0Params.periodUnits==Timer_PERIOD_HZ)
    {
        double a=((1/timer0Params.period)*1000000);
        sprintf(outputMsg, "\r\ntimer %d us\r\n",a);
        UART_write(uart, outputMsg, strlen(outputMsg));
    }
    else if(timer0Params.periodUnits==Timer_PERIOD_COUNTS)
    {
        sprintf(outputMsg, "\r\ntimer %d counts\r\n",timer0Params.period);
        UART_write(uart, outputMsg, strlen(outputMsg));
    }
    else
    {
        sprintf(outputMsg, "\r\ntimer %d us\r\n",timer0Params.period);
        UART_write(uart, outputMsg, strlen(outputMsg));
    }
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPtimerZero()
{
    callback0Count=0;
    UART_write(uart,    NEXTLINE, sizeof(NEXTLINE));
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void *MPtimerSpace()
{
    int i;
    int j=0;
    char secondStr[50];//="\0";
    char thirdStr[50];//="\0";
  //  char fourthStr[50];//="\0";
    strClear(secondStr,50);
    strClear(thirdStr,50);
  //  strClear(fourthStr,50);
    for(i=(strlen("-timer "));((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)//find if second str is present
    {
        secondStr[j]=typedMsg[i];
        j++;
    }
    if(typedMsg[((strlen("-timer "))+(strlen(secondStr)))]==' ')// if space after second string
    {
        j=0;
        for(i=((strlen("-timer "))+(strlen(secondStr))+1/* 1 is needed to skip the space after 2nd str*/);((typedMsg[i]!=' ')&&(i<typedMsgLen)); i++)
        {
            thirdStr[j]=typedMsg[i];
            j++;
        }
        if(strStartsWith(thirdStr,"US")==0)
        {
            timerPeriod=atol(secondStr);
            if(timerPeriod<100)
            {
                sprintf(outputMsg, "\r\ntimer period %d us is too short\r\n",timerPeriod);
                UART_write(uart, outputMsg, strlen(outputMsg));
            }
            else
            {
                Timer_setPeriod(timer0,Timer_PERIOD_US,timerPeriod);
                timer0Params.period=timerPeriod;
                timer0Params.periodUnits=Timer_PERIOD_US;
                sprintf(outputMsg, "\r\ntimer %d us\r\n",timer0Params.period);
                UART_write(uart, outputMsg, strlen(outputMsg));
            }
        }
        else if(strStartsWith(thirdStr,"HZ")==0)
        {
            timerPeriod=atol(secondStr);
            Timer_setPeriod(timer0,Timer_PERIOD_HZ,timerPeriod);
            timer0Params.period=timerPeriod;
            timer0Params.periodUnits=Timer_PERIOD_HZ;
            sprintf(outputMsg, "\r\ntimer %d hz\r\n",timer0Params.period);
            UART_write(uart, outputMsg, strlen(outputMsg));
        }
        else if(strStartsWith(thirdStr,"COUNTS")==0)
        {
            timerPeriod=atol(secondStr);
            Timer_setPeriod(timer0,Timer_PERIOD_COUNTS,timerPeriod);
            timer0Params.period=timerPeriod;
            timer0Params.periodUnits=Timer_PERIOD_COUNTS;
            sprintf(outputMsg, "\r\ntimer %d counts\r\n",timer0Params.period);
            UART_write(uart, outputMsg, strlen(outputMsg));
        }
        else
        {
            sprintf(outputMsg, "\r\ntyped timer period unit is incorrect. Use US,HZ and COUNTS\r\n");
            UART_write(uart, outputMsg, strlen(outputMsg));
            parsingError++;
        }

    }
    else
    {
        timerPeriod=atol(secondStr);
        if(timerPeriod<100)
        {
            sprintf(outputMsg, "\r\ntimer period %d us is too short\r\n",timerPeriod);
            UART_write(uart, outputMsg, strlen(outputMsg));
        }
        else
        {
          //  timerPeriod=atol(secondStr);
            Timer_setPeriod(timer0,Timer_PERIOD_US,timerPeriod);
            timer0Params.period=timerPeriod;
            timer0Params.periodUnits=Timer_PERIOD_US;
            sprintf(outputMsg, "\r\ntimer %d us\r\n",timer0Params.period);
            UART_write(uart, outputMsg, strlen(outputMsg));
        }
    }
    strClear(secondStr,50);
    strClear(thirdStr,50);
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
//function to ceck if next string is present
uint8_t strCheckNext(char* str)
{
    uint8_t strNextPresent=0;
    int i;
    for (i = (strlen("-help")+1);i<=strlen(str);i++) //strt from end of -help inc untill end of typed message len.
    {
        if(str[i]!=' ')
        {
            strNextPresent=1; // second string is present
            break;
        }
        strNextPresent=0;
    }
    return(strNextPresent);
}
/*---------------------------------------------------------------------------------------------------------------------*/
// function to clear all the charac of a string
uint8_t strClear(char* str, uint32_t len)
{
    int i;
    for(i=0;i<=len;i++)
    {
      str[i]='\0';
    }
    return(0);
}
/*---------------------------------------------------------------------------------------------------------------------*/
/* Function to compare begining of the typed message from Putty
   is same as -about or -help and return corresponding vale to mainthread function*/
uint8_t strStartsWith(char* str, char* subStr)
{
    int i;
    uint8_t startsWith = 0;
    printf("%s", str);
    printf("%s", subStr);
    for (i = 0; subStr[i] != '\0'; i++) //increment untill size of -about or -help and compare one char at once
    {
        if (str[i] != subStr[i])
        {
            startsWith = 1;
            break;
        }
        startsWith = 0;
    }
    return(startsWith);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void callbackTimer0(Timer_Handle timer0, int_fast16_t statusTimer0)
{
    Swi_post(Timer0SWI);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void callbackTimer1(Timer_Handle timer1, int_fast16_t statusTimer1)
{
    Swi_post(Timer1SWI);
}
/*----------------------------------------------------------------------------------------------------------------------*/
void callbackSW1Left(uint_least8_t index6)
{
    Swi_post(SW1SWI);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void callbackSW2Right(uint_least8_t index7)
{
    Swi_post(SW2SWI);
}
/*---------------------------------------------------------------------------------------------------------------------*/
void SWIgpioButtonSW1()
{
  //  int i;
  //  for(i=0;i<strlen(callback1Payload);i++)
 //   {
 //       r[wIndex].payload[i]=callback1Payload[i];
 //   }
 //   r[wIndex].payload[i]='\0';
    if(callback1Count!=0)
    {
        {
          //  gatekey= GateSwi_enter(gateSwi0);
          //  Queue_enqueue(myQ, &(r[wIndex].elem));
          //  GateSwi_leave(gateSwi0,gatekey);
            //  wIndex++;
          //  Semaphore_post(semaphore0);
            AddPayload(callback1Payload);
            callback1Count--;
        }
    }

        return;
}
void SWIgpioButtonSW2()
{
  //  int i;
 //   for(i=0;i<strlen(callback2Payload);i++)
 //   {
 //       r[wIndex].payload[i]=callback2Payload[i];
 //   }
 //   r[wIndex].payload[i]='\0';
    if(callback2Count!=0)
    {

 //       {
 //           gatekey= GateSwi_enter(gateSwi0);
 //           Queue_enqueue(myQ, &(r[wIndex].elem));
 //           GateSwi_leave(gateSwi0,gatekey);
            //  wIndex++;
 //           Semaphore_post(semaphore0);
        AddPayload(callback2Payload);
            callback2Count--;
 //       }
    }

        return;
}
void SWItimer0()
{
   // int i;
   // for(i=0;i<strlen(callback0Payload);i++)
  //  {
  //      r[wIndex].payload[i]=callback0Payload[i];
  //  }
  //  r[wIndex].payload[i]='\0';
    if(callback0Count!=0)
    {

        {
         //   gatekey= GateSwi_enter(gateSwi0);
        //    Queue_enqueue(myQ, &(r[wIndex].elem));
        //    GateSwi_leave(gateSwi0,gatekey);
            //  wIndex++;
        //    Semaphore_post(semaphore0);
            AddPayload(callback0Payload);
            callback0Count--;
        }
    }

        return;
}

void SWItimer1()
{
    int i;
    gatekey= GateSwi_enter(gateSwi1);
    for(i=0;i<tickerCount;i++)
    {
        if((ticker[i].count!=0) && (ticker[i].delay==0))
        {
          //  strcpy(r[wIndex].payload,ticker[tickerNum].function);
          //  Queue_enqueue(myQ, &(r[wIndex].elem));
          //  Semaphore_post(semaphore0);
          //  strcpy(ParseQueue.payloads[ParseQueue.payloadWriting],ticker[tickerNum].function);
            strcpy(ParseQueue.payloads[ParseQueue.payloadWriting],ticker[i].function);
            ParseQueue.payloadWriting++;
            Semaphore_post(semaphore0);
            if(ParseQueue.payloadWriting>31)
            {
                ParseQueue.payloadWriting=0;
                MPQueueWriteCycles++;
            }
            if(ticker[i].count>0)
            {
                ticker[i].count--;
            }
            ticker[i].delay=ticker[i].period;
        }
        else if(ticker[i].delay!=0)
        {
            ticker[i].delay--;
        }
    }
    GateSwi_leave(gateSwi1,gatekey);
    return;
}

void AddPayload(char *payload)
{
    gatekey= GateSwi_enter(gateSwi0);
    strcpy(ParseQueue.payloads[ParseQueue.payloadWriting],payload);
    ParseQueue.payloadWriting++;
    Semaphore_post(semaphore0);
    GateSwi_leave(gateSwi0,gatekey);
    if(ParseQueue.payloadWriting>31)
    {
        ParseQueue.payloadWriting=0;
        MPQueueWriteCycles++;
    }

    return;
}






/*void AddPayload(char *payload)
{
    int32_t payloadnext;
    int32_t index;
    if(!payload || payload[0]==0)
        return;
    index=wIndex;
    gatekey= GateSwi_enter(gateSwi0);
    payloadnext=index+1;
    if(payloadnext >= payloadCount)
            payloadnext=0;
    if(payloadnext=rIndex)
    {
        sprintf(outputMsg, "\r\nqueue overflow\r\n");
        UART_write(uart, outputMsg, strlen(outputMsg));
        queueOverflow++;
    }
    else
    {
 //       strcpy(ParseQueue.payloads[index],payload);
 //       ParseQueue.payloadWriting=payloadnext;
    }
    GateSwi_leave(gateSwi0,gatekey);
    Semaphore_post(semaphore0);
    return index;
} */

