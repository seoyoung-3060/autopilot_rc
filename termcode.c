#include "stm32f10x.h"
#include "core_cm3.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_HC-SR04.h"


/********************Current Specification********************

* IR ���� 2�� ��� (���� ��� �ν�, ���� ��� �ν�)
* 1ȸ �н� ���� �� 2ȸ�� ���ʹ� ������ ���� ���� ����
* BT�� ���� �н� ����, ����, ���� 3���� ��ɾ� �ο��� [���ͷ�Ʈ]
* ���ܼ� ������ �ٰŸ� (80mm)�� ��ü�ν� �� �Ͻ� ���� [���ͷ�Ʈ]
* ���� ���� IR ���� ���۽� ��ü �Ͻ� ���� [���ͷ�Ʈ]

*************************************************************/


/**************************TODO List**************************

* memory allocation ���� ������ Ȯ��

*************************************************************/



//���ƿ� 1  ���� 0
int waybackFlag = 0;
int learningFlag = 0;
uint32_t dist = 0;


//�⺻ ���� ���ø� ���� enumeration
enum Direction {UP, DOWN, LEFT, RIGHT, STOP};
//���� ���� �̵� ���� ���ÿ�
enum Direction state = STOP;

//������ ������ ��Ͽ� ����ü 
typedef __packed struct pathPick{
    enum Direction direction;
    uint32_t visit;
}pathPick;

//������ ������ ���
pathPick visitPath[10];

//���ܼ� ���� B1, C0, C2, C3, C4, C5 ����[0]1 �߰�[1]2 ������[2]3

void USART2_Init(void);
void USART2_IRQHandler(void);
void sendDataUART2(uint16_t data);
void setCMD(char);

//��������� ��ɾ�
#define START   'S'
#define HALT    'H'
#define LEARN   'L'

//�̷� ������ �ִ� ����
#define MAXCROSS 10

    char startup[] = "BTWIN Slave Mode Start";
    char startScan[] = "AT+BTSCAN";

void RCC_Configure() {
    /* UART TX/RX port clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    // UART TX PA 9, RX PA 10
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /* USART2 clock enable datasheet�� ���� APB1�� �����*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    //���ܼ� ����
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    //PD 9,10,11,12
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // interrupt
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE); //motor driver
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //motor driver

}

void EnableHCSR04PeriphClock() {
      //������ ������ Ÿ�̸�

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,  ENABLE);
}


void GPIO_Configure() {
    //motor driver PD (8, 9)���� ��, (10,11)������ ��,
    // (12,13)���� ��, (14,15)������ ��
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
    //TX�� PA9 output Push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //RX�� PA10 input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* UART2 pin setting */
    //TX
    //TX�� PA2 output Push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //RX�� PA3 input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //���ܼ� ���� PC0
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //���ܼ� ���� PC2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

//���ܼ� �νĿ� EXTI
void EXTI_config(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    
    //���� IR����
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    //������ IR����
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

//������� ��ſ� USART2
void USART2_Init(void)
{
    USART_InitTypeDef USART2_InitStructure;
    //stm32f10x_usart.h����
    // Enable the USART1 peripheral
    USART_Cmd(USART2, ENABLE);

    //���� ���迡�� ������ ������ �����Ѵ�.
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

//USART2 interrupt = �������
void NVIC_Configure(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    //BT ���ͷ�Ʈ
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    // UART2
    // 'NVIC_EnableIRQ' is only required for USART setting
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    //SR ���� ���ͷ�Ʈ
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //IR ���� ���ͷ�Ʈ C0 (����)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //IR ���� ���ͷ�Ʈ C2 (������)
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // TODO
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // TODO
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

//���ܼ� ���� interrupt
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


//���ܼ� ���� interrupt
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

//������ ���� interrupt
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

//������� interrupt
void USART2_IRQHandler(void) {
    uint16_t word;
    if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){
        // the most recent received data by the USART1 peripheral
        word = USART_ReceiveData(USART2);

        //���� ���� ��ɾ echoó�� ������ Ȯ�� �����ϵ��� ��
        sendDataUART2(word);
 
        //���� ���� ��ɾ ����
        setCMD(word);

        // clear 'Read data register not empty' flag
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }

}
                                                        
void sendDataUART2(uint16_t data) {
    /* Wait till TC is set */
    USART_SendData(USART2, data);
}

//������� �ʱ�ȭ �۾�
void startBT()
{
   char startup[] = "BTWIN Slave Mode Start";
    char startScan[] = "AT+BTSCAN";

    //USART1�� USB ������ ��������� ������ slave mod�� ���۽�Ű���� ��.
    //�ڱ��ڽſ��� slave mode ������ ��ĵ ��ɾ ����.
    for (int i = 0; i < 23; i ++)
        sendDataUART2(startup[i]);

    for (int i = 0; i < 10; i ++)
        sendDataUART2(startScan[i]);
}

void setCMD(char flag)
{
   //�н� ���� ����
   if (flag == START)
   {
      //����� Ȯ�ο� �޽���
        char msg[] = "BT car starts";
        for (int i = 0; i < sizeof(msg); i ++)
         sendDataUART2(msg[i]);

       //�� ���� ����
       state = UP;
       learningFlag = 0;
       waybackFlag = 0;
   }
   //���� ��ɾ�
   else if (flag == HALT)
   {
      char msg[] = "BT car stops";
        for (int i = 0; i < sizeof(msg); i ++)
            sendDataUART2(msg[i]);
        state = STOP;
        learningFlag = 0;
        waybackFlag = 0;
   }
   //�н� ���� ��ɾ�
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
    //��
    GPIO_ResetBits(GPIOC, GPIO_Pin_8);
    GPIO_ResetBits(GPIOC, GPIO_Pin_9);

    //��
    GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    GPIO_ResetBits(GPIOD, GPIO_Pin_13);

    if(set == 1) {
        GPIO_SetBits(GPIOD, GPIO_Pin_13); //��
        GPIO_SetBits(GPIOC, GPIO_Pin_9); //��
    }
    else if(set == 0) {
        GPIO_SetBits(GPIOC, GPIO_Pin_8);
        GPIO_SetBits(GPIOD, GPIO_Pin_12);
    }
}

void rightmotor(int set) {
    //�չ���
    GPIO_ResetBits(GPIOD, GPIO_Pin_5);
    GPIO_ResetBits(GPIOD, GPIO_Pin_6);
    
    //�޹���
    GPIO_ResetBits(GPIOD, GPIO_Pin_14);
    GPIO_ResetBits(GPIOD, GPIO_Pin_15);

    if(set == 1) {
        //GPIO_SetBits(GPIOD, GPIO_Pin_15);
        GPIO_SetBits(GPIOD, GPIO_Pin_5); 
    }
    if(set == 0) {
        GPIO_SetBits(GPIOD, GPIO_Pin_6); //��
        //GPIO_SetBits(GPIOD, GPIO_Pin_14); //��
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

//������ ������ ��� �ʱ�ȭ
void pathInit()
{
    for (int i = 0; i < MAXCROSS; i ++)
    {
        visitPath[i].direction = STOP;
        visitPath[i].visit = 0;
    }   
}

//�ֱ� ������ �������� ������ �߰���
void addPath(enum Direction dir)
{
    uint32_t pos;
    
    //�ִ� ������ ��� Ƚ���� �Ѿ ���� ����ó��
    if (visitPath[MAXCROSS - 1].visit == 1)
        pathInit();


    //��� �ִ�, ���� �������� ������ ��ȣ�� ������
    for (int i = 0; i < MAXCROSS; i ++)
    {
        if (visitPath[i].visit == 1)
            pos = i;
            break;
    }

    visitPath[pos].visit = 1;
    visitPath[pos].direction = dir;
}

//���� �ֱ� ���õ� �������� ������ ������
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

//���� �ֱ� �湮�ߴ� �������� �������� ȣ����
enum Direction getPath()
{
    for (int i = MAXCROSS - 1; i > -1; i --)
    {
        if (visitPath[i].visit == 1)
            return visitPath[i].direction;
    }
}

//Ư�� ��ȣ�� �����濡�� ������ ���� ȣ����
enum Direction recallPath(uint32_t num)
{
    if (visitPath[num].visit == 1)
        return visitPath[num].direction;
    else
        return STOP;
}

int main() {
    // LCD ���� ������ LCD_Init�� �����Ǿ� �����Ƿ� ���⼭ �� �ʿ� ����
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

    //BT ��ɾ �Էµ��� ������ �������� ����
    while (state == STOP){}

    //�н� �� ������ ��������
    int cross = 0;

    while(1) {      
        //2���� ���ܼ� ���� ��� (������ �߰ߵ��� ������ ������)
        uint32_t lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
        uint32_t rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);

        //������ ����
        if (lft == 0 && rgt == 0)
        {
            state = STOP;
            motor();

            cross = 0;
            sendDataUART2(HALT);
        }
        //����
        else if (lft == 1 && rgt == 1)
        {
            state = UP;
            motor();
        }
        //��ȸ��
        else if (lft == 0 && rgt == 1)
        {
            state = LEFT;
            motor();

            //������ ������ ȸ����
            while(lft == 1 && rgt == 1)
            {
             lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
             rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
            }
            state = STOP;
            motor();
        }
        //��ȸ��
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
        //���� ���� (�������̹Ƿ� �н� ���� üũ��)
        else if (lft == 0 && rgt == 1)
        {
            if (learningFlag == 0)
            {
                //�����濡�� ������ ������ ȣ���� ��ü�� ������ ��ȯ��Ŵ
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
                //���� �� ���� ������ �߰ߵǾ���
                if (waybackFlag == 0)
                {
                    //����� �̵��ϰ� ������ �������� �����
                    state = LEFT;
                    addPath(LEFT);
                    motor();
                    while(lft == 1 && rgt == 1)
                    {
                     lft = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
                     rgt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
                    }
                }

                //�¿� �����濡�� ��ȸ���� ����� �����
                //���� ��ü�� ���� ������ �����ؿ� ��, �տ� ���� ���� �¿� �������� ���� ����
                else
                {
                    //���� ��� �̵�
                    state = UP;

                    //������ �����ϵ��� ������ ������Ʈ��
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
        //������ ����
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
        //�¿� ������
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

                //�¿� �������� ó������ ������
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

                //���� Ȥ�� ���� ������ �̿��ߴٰ� �ǵ��ƿ� �����
                else
                {
                    uint32_t oldDir = getPath();

                    //���� ������ �̿����� ���
                    if (oldDir == LEFT)
                    {
                        //������ �ƴ� ������η� ��ü �̵� �� '����'���� ������Ʈ
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
                    //���� ������ �̿����� ���
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
