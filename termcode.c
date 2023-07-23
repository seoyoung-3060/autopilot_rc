#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_HC-SR04.h"


/********************Current Specification********************

* IR 센서 2개 사용 (좌측 경로 인식, 우측 경로 인식)
* 1회 학습 운행 후 2회차 부터는 목적지 까지 직행 가능
* BT를 통해 학습 운행, 정지, 운행 3가지 명령어 부여함 [인터럽트]
* 적외선 센서로 근거리 (80mm)에 물체인식 시 일시 정지 [인터럽트]
* 좌측 우측 IR 센서 동작시 차체 일시 정지 [인터럽트]

*************************************************************/


/**************************TODO List**************************

* memory allocation 통해 선택지 확장

*************************************************************/



//돌아옴 1  없음 0
int waybackFlag = 0;
int learningFlag = 0;
uint32_t dist = 0;


//기본 방향 지시를 위한 enumeration
enum Direction {UP, DOWN, LEFT, RIGHT, STOP};
//현재 모터 이동 방향 지시용
enum Direction state = STOP;

//갈림길 선택지 기록용 구조체 
typedef __packed struct pathPick{
    enum Direction direction;
    uint32_t visit;
}pathPick;

//갈림길 선택지 기록
pathPick visitPath[10];

//적외선 센서 B1, C0, C2, C3, C4, C5 왼쪽[0]1 중간[1]2 오른쪽[2]3

void USART2_Init(void);
void USART2_IRQHandler(void);
void sendDataUART2(uint16_t data);
void setCMD(char);

//블루투스용 명령어
#define START   'S'
#define HALT    'H'
#define LEARN   'L'

//미로 갈림길 최대 개수
#define MAXCROSS 10

    char startup[] = "BTWIN Slave Mode Start";
    char startScan[] = "AT+BTSCAN";

void RCC_Configure() {
    /* UART TX/RX port clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    // UART TX PA 9, RX PA 10
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* USART2 clock enable datasheet에 보면 APB1을 사용함*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    //적외선 센서
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    //PD 9,10,11,12
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // interrupt
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); //motor driver
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //motor driver

}

void EnableHCSR04PeriphClock() {
      //초음파 센서용 타이머

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,  ENABLE);
}


void GPIO_Configure() {
    //motor driver PD (8, 9)왼쪽 앞, (10,11)오른쪽 앞,
    // (12,13)왼쪽 뒤, (14,15)오른쪽 뒤
    //PC8 +
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //PC9 -
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    //PD10 +
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    //PD11 -
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    //PD12 +
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //PD13 -
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //PD14 +
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //PD15 -
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

         /* UART1 pin setting */
    //TX
    //TX는 PA9 output Push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //RX는 PA10 input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* UART2 pin setting */
    //TX
    //TX는 PA2 output Push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //RX는 PA3 input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //적외선 센서 PC0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //적외선 센서 PC2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

//적외선 인식용 EXTI
void EXTI_config(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    
    //왼쪽 IR센서
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    //오른쪽 IR센서
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

//블루투스 통신용 USART2
void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;
    //stm32f10x_usart.h참고
    // Enable the USART1 peripheral
    USART_Cmd(USART2, ENABLE);

    //저번 실험에서 설정한 값으로 구현한다.
    // TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
    USART2_InitStructure.USART_BaudRate = 9600;
    USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART2_InitStructure.USART_StopBits = USART_StopBits_1;
    USART2_InitStructure.USART_Parity = USART_Parity_No;
    USART2_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART2_InitStructure);

    // TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //RXNE = 'Receive Data register not empty interrupt'
}

//USART2 interrupt = 블루투스
void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    //BT 인터럽트
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    // UART2
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    //SR 센서 인터럽트
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //IR 센서 인터럽트 C0 (왼쪽)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //IR 센서 인터럽트 C2 (오른쪽)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//적외선 좌측 interrupt
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET){
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == Bit_RESET){
            GPIO_ResetBits(GPIOC, GPIO_Pin_8);
          GPIO_ResetBits(GPIOC, GPIO_Pin_9);

          GPIO_ResetBits(GPIOD, GPIO_Pin_5);
          GPIO_ResetBits(GPIOD, GPIO_Pin_6);

          GPIO_ResetBits(GPIOD, GPIO_Pin_12);
          GPIO_ResetBits(GPIOD, GPIO_Pin_13);

          GPIO_ResetBits(GPIOD, GPIO_Pin_14);
          GPIO_ResetBits(GPIOD, GPIO_Pin_15);
        }
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}


//적외선 우측 interrupt
void EXTI2_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line2) != RESET){
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2) == Bit_RESET){
            GPIO_ResetBits(GPIOC, GPIO_Pin_8);
          GPIO_ResetBits(GPIOC, GPIO_Pin_9);

          GPIO_ResetBits(GPIOD, GPIO_Pin_5);
          GPIO_ResetBits(GPIOD, GPIO_Pin_6);

          GPIO_ResetBits(GPIOD, GPIO_Pin_12);
          GPIO_ResetBits(GPIOD, GPIO_Pin_13);

          GPIO_ResetBits(GPIOD, GPIO_Pin_14);
          GPIO_ResetBits(GPIOD, GPIO_Pin_15);
        }
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}

//초음파 센서 interrupt
void TIM3_IRQHandler(){
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){
        dist = HCSR04GetDistance();
        while(dist < 80) {
            GPIO_ResetBits(GPIOC, GPIO_Pin_8);
          GPIO_ResetBits(GPIOC, GPIO_Pin_9);

          GPIO_ResetBits(GPIOD, GPIO_Pin_5);
          GPIO_ResetBits(GPIOD, GPIO_Pin_6);

          GPIO_ResetBits(GPIOD, GPIO_Pin_12);
          GPIO_ResetBits(GPIOD, GPIO_Pin_13);

          GPIO_ResetBits(GPIOD, GPIO_Pin_14);
          GPIO_ResetBits(GPIOD, GPIO_Pin_15);

            dist = HCSR04GetDistance();
        }
    }
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

//블루투스 interrupt
void USART2_IRQHandler(void) {
    uint16_t word;
    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){
        // the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART2);

        //현재 받은 명령어를 echo처럼 보내어 확인 가능하도록 함
        sendDataUART2(word);
 
        //수신 받은 명령어를 적용
        setCMD(word);

        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }

}
                                                        
void sendDataUART2(uint16_t data) {
    /* Wait till TC is set */
    USART_SendData(USART2, data);
}

//블루투스 초기화 작업
void startBT()
{
   char startup[] = "BTWIN Slave Mode Start";
    char startScan[] = "AT+BTSCAN";

    //USART1과 USB 연결의 부재로인해 스스로 slave mod를 동작시키도록 함.
    //자기자신에게 slave mode 구동과 스캔 명령어를 보냄.
    for (int i = 0; i < 23; i ++)
        sendDataUART2(startup[i]);

    for (int i = 0; i < 10; i ++)
        sendDataUART2(startScan[i]);
}

void setCMD(char flag)
{
   //학습 이후 전진
   if (flag == START)
   {
      //사용자 확인용 메시지
        char msg[] = "BT car starts";
        for (int i = 0; i < sizeof(msg); i ++)
         sendDataUART2(msg[i]);

       //각 상태 설정
       state = UP;
       learningFlag = 0;
       waybackFlag = 0;
   }
   //정지 명령어
   else if (flag == HALT)
   {
      char msg[] = "BT car stops";
        for (int i = 0; i < sizeof(msg); i ++)
            sendDataUART2(msg[i]);
        state = STOP;
        learningFlag = 0;
        waybackFlag = 0;
   }
   //학습 시작 명령어
   else if (flag == LEARN)
    {
      char msg[] = "BT car learning starts";
        for (int i = 0; i < sizeof(msg); i ++)
            sendDataUART2(msg[i]);
        state = UP;
        learningFlag = 1;
        waybackFlag = 0;

   }
   else
   {
      char msg[] = "Not a proper command";
        for (int i = 0; i < sizeof(msg); i ++)
            sendDataUART2(msg[i]);
        state = STOP;
        learningFlag = 0;
        waybackFlag = 0;
    }

    for (int i = 0; i < 10; i ++)
        sendDataUART2(startScan[i]);
}

void leftmotor(int set) {
    //앞
    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);

    //뒤
    GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);

    if(set == 1) {
        GPIO_SetBits(GPIOD, GPIO_Pin_13); //뒤
        GPIO_SetBits(GPIOC, GPIO_Pin_9); //앞
    }
    else if(set == 0) {
        GPIO_SetBits(GPIOC, GPIO_Pin_8);
        GPIO_SetBits(GPIOD, GPIO_Pin_12);
    }
}

void rightmotor(int set) {
    //앞바퀴
    GPIO_ResetBits(GPIOD, GPIO_Pin_5);
    GPIO_ResetBits(GPIOD, GPIO_Pin_6);
    
    //뒷바퀴
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
    GPIO_ResetBits(GPIOD, GPIO_Pin_15);

    if(set == 1) {
        //GPIO_SetBits(GPIOD, GPIO_Pin_15);
        GPIO_SetBits(GPIOD, GPIO_Pin_5); 
    }
    if(set == 0) {
        GPIO_SetBits(GPIOD, GPIO_Pin_6); //앞
        //GPIO_SetBits(GPIOD, GPIO_Pin_14); //뒤
    }
}

void motorstop() {
    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);

    GPIO_ResetBits(GPIOD, GPIO_Pin_5);
    GPIO_ResetBits(GPIOD, GPIO_Pin_6);

    GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);

    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
    GPIO_ResetBits(GPIOD, GPIO_Pin_15);
}

void motor() {
    switch(state) {
    case UP:
        leftmotor(1);
        rightmotor(1);
        break;
    case DOWN:
        leftmotor(0);
        rightmotor(0);
        break;
    case LEFT:
        leftmotor(3);
        rightmotor(1);
        break;
    case RIGHT:
        leftmotor(1);
        rightmotor(3);
        break;
    case STOP:
        motorstop();
        break;
    default:
        motorstop();
        break;
    }
}

//갈림길 선택지 기록 초기화
void pathInit()
{
    for (int i = 0; i < MAXCROSS; i ++)
    {
        visitPath[i].direction = STOP;
        visitPath[i].visit = 0;
    }   
}

//최근 선택한 갈림길의 방향을 추가함
void addPath(enum Direction dir)
{
    uint32_t pos;
    
    //최대 교차로 기록 횟수를 넘어설 때의 예외처리
    if (visitPath[MAXCROSS - 1].visit == 1)
        pathInit();


    //비어 있는, 가장 마지막의 갈림길 번호를 선택함
    for (int i = 0; i < MAXCROSS; i ++)
    {
        if (visitPath[i].visit == 1)
            pos = i;
            break;
    }

    visitPath[pos].visit = 1;
    visitPath[pos].direction = dir;
}

//제일 최근 선택된 갈림길의 선택을 제거함
void removePath()
{
    for (int i = MAXCROSS - 1; i > -1; i --)
    {
        if (visitPath[i].visit == 1)
            visitPath[i].direction = STOP;
            visitPath[i].visit = 0;
            break;
    }
}

//제일 최근 방문했던 갈림길의 선택지를 호출함
enum Direction getPath()
{
    for (int i = MAXCROSS - 1; i > -1; i --)
    {
        if (visitPath[i].visit == 1)
            return visitPath[i].direction;
    }
}

//특정 번호의 갈림길에서 선택한 길을 호출함
enum Direction recallPath(uint32_t num)
{
    if (visitPath[num].visit == 1)
        return visitPath[num].direction;
    else
        return STOP;
}

int main() {
    // LCD 관련 설정은 LCD_Init에 구현되어 있으므로 여기서 할 필요 없음
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    EXTI_config();
    USART2_Init();
    InitHCSR04();

    NVIC_Configure();
    
    pathInit();
    startBT();

    state = STOP;
    motor();

    //BT 명령어가 입력되지 않으면 구동되지 않음
    while (state == STOP){}

    //학습 후 갈림길 선택지용
    int cross = 0;

    while(1) {      
        //2개의 적외선 센서 사용 (직선이 발견되지 않으면 직진중)
        uint32_t lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
        uint32_t rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);

        //목적지 도착
        if (lft == 0 && rgt == 0)
        {
            state = STOP;
            motor();

            cross = 0;
            sendDataUART2(HALT);
        }
        //직진
        else if (lft == 1 && rgt == 1)
        {
            state = UP;
            motor();
        }
        //좌회전
        else if (lft == 0 && rgt == 1)
        {
            state = LEFT;
            motor();

            //직진할 때까지 회전함
            while(lft == 1 && rgt == 1)
            {
             lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
             rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
            }
            state = STOP;
            motor();
        }
        //우회전
        else if (lft == 1 && rgt == 0)
        {
            state = RIGHT;
            motor();

            while(lft == 1 && rgt == 1)
            {
             lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
             rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
            }
            state = STOP;
            motor();
        }
        //왼쪽 샛길 (갈림길이므로 학습 유무 체크함)
        else if (lft == 0 && rgt == 1)
        {
            if (learningFlag == 0)
            {
                //갈림길에서 선택한 방향을 호출해 차체의 방향을 전환시킴
                state = recallPath(cross++);
                motor();
                while(lft == 1 && rgt == 1)
                {
                 lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
                 rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
                }
            }
            else
            {
                //직진 중 좌측 샛길이 발견되었음
                if (waybackFlag == 0)
                {
                    //샛길로 이동하고 샛길을 선택지에 기록함
                    state = LEFT;
                    addPath(LEFT);
                    motor();
                    while(lft == 1 && rgt == 1)
                    {
                     lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
                     rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
                    }
                }

                //좌우 갈림길에서 우회전을 골랐을 경우임
                //현재 차체의 좌측 샛길은 직진해온 길, 앞에 놓인 길은 좌우 갈림길의 좌측 길임
                else
                {
                    //좌측 길로 이동
                    state = UP;

                    //좌측을 선택하도록 선택지 업데이트함
                    removePath();
                    addPath(LEFT);
                    motor();
                    while(lft == 1 && rgt == 1)
                    {
                     lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
                     rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
                    }
                    state = STOP;
                    motor();
                }
            }
            
            
        }
        //오른쪽 샛길
        else if (lft == 1 && rgt == 0)
        {
            if (learningFlag == 0)
            {
                state = recallPath(cross++);
                motor();
                while(lft == 1 && rgt == 1)
                {
                 lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
                 rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
                }
            }
            else
            {
                if (waybackFlag == 0)
                {
                    state = RIGHT;
                    addPath(RIGHT);
                    motor();
                    while(lft == 1 && rgt == 1)
                    {
                     lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
                     rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
                    }
                }
                else
                {
                    state = UP;
                    removePath();
                    addPath(RIGHT);
                    motor();
                    while(lft == 1 && rgt == 1)
                    {
                     lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
                     rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
                    }
                }
            }
        }
        //좌우 갈림길
        else if (lft == 0 && rgt == 0)
        {
            if (learningFlag == 0)
            {
                state = recallPath(cross++);
                motor();
                while(lft == 1 && rgt == 1)
                {
                 lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
                 rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
                }
            }

            else
            {

                //좌우 갈림길을 처음으로 마주함
                if (waybackFlag == 0)
                {
                    state = LEFT;
                    addPath(LEFT);
                    motor();
                    while(lft == 1 && rgt == 1)
                    {
                     lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
                     rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
                    }
                    state = STOP;
                    motor();
                }

                //좌측 혹은 우측 샛길을 이용했다가 되돌아온 경우임
                else
                {
                    uint32_t oldDir = getPath();

                    //좌측 샛길을 이용했을 경우
                    if (oldDir == LEFT)
                    {
                        //샛길이 아닌 직진경로로 차체 이동 후 '직진'으로 업데이트
                        waybackFlag = 0;
                        state = LEFT;
                        removePath();
                        addPath(UP);
                        motor();
                        while(lft == 1 && rgt == 1)
                        {
                         lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
                         rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
                        }
                        state = STOP;
                        motor();
                    }
                    //우측 샛길을 이용했을 경우
                    else
                    {
                        waybackFlag = 0;
                        state = RIGHT;
                        removePath();
                        addPath(UP);
                        motor();
                        while(lft == 1 && rgt == 1)
                        {
                         lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
                         rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
                        }
                        state = STOP;
                        motor();
                    }
                }
            }   
        }
        else
        {
            waybackFlag = 0;
            state = STOP;
            motor();
        }

    }

    return 0;
}
