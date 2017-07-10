#include <pt.h>    //ProtoThreads必须包含的头文件
#include <TimerOne.h> //使用定时器0来做时间片的定时

//---------------------跌落和碰撞传感器定义---------------------------------------------------
#define dropIsrFL  2  //D2跌落传感器前左,机器人离地距离大于厘米输出高电平
#define dropIsrFR  3  //D3跌落传感器前右,机器人离地距离大于厘米输出高电平
#define dropIsrBL  4  //D4跌落传感器后左,机器人离地距离大于厘米输出高电平
#define dropIsrBR  5  //D5跌落传感器后右,机器人离地距离大于厘米输出高电平
#define crashIsrFL 6  //D6碰撞传感器左,机器人碰撞输出地电平  
#define crashIsrFM 7  //前中间碰撞传感器 接7口
#define crashIsrFR 8  //D8碰撞传感器右,机器人碰撞输出地电平 

//---------------------红外传感器定义---------------------------------------------------------
#define infraredDistanceF1  A6  //HCR机器人 前端从左到右第1只  GP2YOA21红外测距传感器
#define infraredDistanceF2  A7  //第2只  GP2YOA21红外测距传感器
#define infraredDistanceF3  A8
#define infraredDistanceF4  A9
#define infraredDistanceF5  A10
#define infraredDistanceB1  A11 //HCR机器人后端从左到右第1只  GP2YOA21红外测距传感器
#define infraredDistanceB2  A12
#define infraredDistanceB3  A13



static int timer50msCounter1; //timer50msCounter为定时计数器
static int timer50msCounter2;
static int timer50msCounter3;
static int timer50msCounter4;
static int timer50msCounter5;
static int state1=0;          //state为灯的状态
boolean abortStatus = false;

#define dropFlgeFL  0b00000001  //跌落传感器前左 状态字 1 : 传感器动作
#define dropFlgeFR  0b00000010  //跌落传感器前右 状态字 1: 传感器动作
#define dropFlgeBL  0b00000100  //跌落传感器后左 状态字 1: 传感器动作
#define dropFlgeBR  0b00001000  //跌落传感器后右 状态字 1: 传感器动作
#define crashFlgeFL 0b00010000  //碰撞传感器左 状态字 1: 传感器动作 
#define crashFlgeFM 0b00100000  //碰撞传感器中 状态字 1: 传感器动作
#define crashFlgeFR 0b01000000  //碰撞传感器右 状态字 1: 传感器动作
byte crashDropSensorStatus = 0;  //定义碰撞和跌落传感器状态字,每bit代表一个传感器状态 1代表触发

byte infraredRange[8];  //8个红外传感器测距结果,单位 厘米
int batteryVoltage;   //电源电压 123代表 12.3V
long EncoderCount1 = 0;  //编码器计数  范围为：-2147483648~2147483647
long EncoderCount2 = 0;
unsigned int ultrasonicDistance[6];  //6个超声波测量距离值,1-500厘米

int M1Speed = 0;  //电机1速度-164 到 164, -164反转最大速度, 0 电机停止,164正转最大速度
int M2Speed = 0;  //电机2速度-164 到 164

#define VOLTAGE_OUT  1 //电压输出
#define ENCODER_OUT  1 //编码器信息
#define IR_OUT        //红外传感器数据输出
#define U_OUT  1   //超声波输出使能

static struct pt pt1, pt2,pt3,pt4,pt5; 

/*--------------------------------------------------------------------------------------------
函数说明:  底层跌落,碰撞,红外,超声波传感器读取
参数说明:  无
返回值  :  无
---------------------------------------------------------------------------------------------*/
void readSensor(void)
{  
  byte i = 0;
  int val = 0;
  
  if(digitalRead(dropIsrFL) == HIGH)  
  {
     abortHandle();  //机器人异常处理程序 
     crashDropSensorStatus |= dropFlgeFL;
   }
  if(digitalRead(dropIsrFR) == HIGH)  
  {
     abortHandle();  //机器人异常处理程序    
     crashDropSensorStatus |= dropFlgeFR;
   }
  if(digitalRead(dropIsrBL) == HIGH)  
  {
    abortHandle();  //机器人异常处理程序
     crashDropSensorStatus |= dropFlgeBL;
   }
  if(digitalRead(dropIsrBR) == HIGH)  
  {
    abortHandle();  //机器人异常处理程序
     crashDropSensorStatus |= dropFlgeBR;
   }
  if(digitalRead(crashIsrFL) == LOW)  
  {
     abortHandle();  //机器人异常处理程序
     crashDropSensorStatus |= crashFlgeFL;
   }
  if(digitalRead(crashIsrFM) == LOW)  
  {
     abortHandle();  //机器人异常处理程序
     crashDropSensorStatus |= crashFlgeFM;
  }
  if(digitalRead(crashIsrFR) == LOW)  
  {
     abortHandle();  //机器人异常处理程序
     crashDropSensorStatus |= crashFlgeFR;
  }
  for(i = 0;i < 8;i++)
  {
    val = analogRead(i+6);
    val = (6762/(val-9))-4;
    infraredRange[i] = (byte)val;
  }
   #ifdef  IR_OUT   //输出 红外传感器信息
      Serial.print("IR Sensor out:");
    for(i=0;i<8;i++)
    {         
      Serial.print(infraredRange[i],DEC);
      Serial.print("  ");
    }
    Serial.println(" ");
  #endif  
    
  motorGetSystemVoltage(&batteryVoltage);  //获取系统电压
  
  #ifdef  VOLTAGE_OUT     //输出 电压值
        Serial.print("Battery Voltage:");
        Serial.println(batteryVoltage,DEC);
  #endif  
  
  motorGetEncoder(&EncoderCount1,&EncoderCount2);  //获取编码器累计脉冲数
  
  #ifdef  ENCODER_OUT     //输出 电压值
        Serial.print("Encoder:");
        Serial.print(EncoderCount1,DEC);
        Serial.print("  ");
        Serial.println(EncoderCount2,DEC);
        Serial.println("  ");
  #endif    
  
}

/*--------------------------------------------------------------------------------------------
函数说明:  跌落\碰撞\红外传感器读取线程，每50ms读取一次
参数说明:  *pt - ProtoThreads 操作系统指针 
返回值  :  无
---------------------------------------------------------------------------------------------*/
static int ReadSensorThread(struct pt *pt) 
{  
  PT_BEGIN(pt);  //线程开始
  while(1) //每个线程都不会死
  {  
    PT_WAIT_UNTIL(pt, timer50msCounter1>0); //如果时间满了50毫秒，则继续执行，否则记录运行点，退出线程1 
    readSensor();  //读取一次传感器
    timer50msCounter1 = 0; //计数器置零
  } 
  PT_END(pt); //线程结束
} 

/*--------------------------------------------------------------------------------------------
函数说明:  电机控制线程，每500ms发送一次电机控制命令
参数说明:  *pt - ProtoThreads 操作系统指针 
返回值  :  无
---------------------------------------------------------------------------------------------*/
static int MotoControlThread(struct pt *pt) 
{  
  PT_BEGIN(pt);  //线程开始
  while(1) //每个线程都不会死
  {  
    PT_WAIT_UNTIL(pt, timer50msCounter2 >= 10); //如果时间满了500毫秒，则继续执行，否则记录运行点，退出线程1 
     // 测试是否有报警信号,确认是否使能驱动
    Serial3.println("1,CER");    
    motorControl(M1Speed,M2Speed);
    timer50msCounter2 = 0; //计数器置零
  } 
  PT_END(pt); //线程结束
} 

/*--------------------------------------------------------------------------------------------
函数说明:  线程2，控制灯1
参数说明:  *pt - ProtoThreads 操作系统指针 
返回值  :  无
---------------------------------------------------------------------------------------------*/
static int LEDBlinkThread(struct pt *pt) 
{ 
  PT_BEGIN(pt); //线程开始
  while(1) 
  {    //每个线程都不会死
    PT_WAIT_UNTIL(pt, timer50msCounter3 >= 10); //如果时间满了500毫秒，则继续执行，否则记录运行点，退出线程2
    timer50msCounter3 = 0;  //计数清零
    digitalWrite(13,state1);
    state1=!state1; //灯状态反转
  } 
  PT_END(pt);  //线程结束
} 

/*--------------------------------------------------------------------------------------------
函数说明:  电机测试进程
参数说明:  *pt - ProtoThreads 操作系统指针 
返回值  :  无
---------------------------------------------------------------------------------------------*/
static int MotoTestThread(struct pt *pt) 
{ 
  static int i;
  PT_BEGIN(pt); //线程开始
  while(1) 
  {  
    for(i=0;i<164;i++)
    {
      if(abortStatus == true) 
      {
        timer50msCounter4 = 0;  //计数清零delay
        PT_WAIT_UNTIL(pt, timer50msCounter4 >= 40); //如果时间满了2000毫秒，则继续执行，否则记录运行点，退出线程
        abortStatus = false;
      }
      M1Speed = i;  //设置左右两个电机的速度
      M2Speed = i;  
      motorControl(M1Speed,M2Speed);
      timer50msCounter4 = 0;  //计数清零
      PT_WAIT_UNTIL(pt, timer50msCounter4 >= 1); //如果时间满了50毫秒，则继续执行，否则记录运行点，退出线程 
    }
    for(i=0;i>-164;i--)
    {
      if(abortStatus == true) 
      {
        timer50msCounter4 = 0;  //计数清零
        PT_WAIT_UNTIL(pt, timer50msCounter4 >= 40); //如果时间满了2000毫秒，则继续执行，否则记录运行点，退出线程
        abortStatus = false;
      }
      M1Speed = i;  //设置左右两个电机的速度
      M2Speed = i;  
      motorControl(M1Speed,M2Speed);
      timer50msCounter4 = 0;  //计数清零
      PT_WAIT_UNTIL(pt, timer50msCounter4 >= 1); //如果时间满了50毫秒，则继续执行，否则记录运行点，退出线程
    }  
  } 
  PT_END(pt);  //线程结束
} 

extern	struct 	DATA_BUF RXD_BUF;    //接收命令缓冲区
/*--------------------------------------------------------------------------------------------
函数说明:  超声波读取进程,刷新频率5HZ
参数说明:  *pt - ProtoThreads 操作系统指针 
返回值  :  无
---------------------------------------------------------------------------------------------*/
static int UltrasonicReadThread(struct pt *pt) 
{ 
  static byte i;
  static byte temp1;
  static unsigned int distance;
  PT_BEGIN(pt); //线程开始
  while(1) 
  {   
    for(i=0;i<3;i++)
    {
      UltrasonicTriggerCommand(0x11+i);  //触发URM04超声波及其和他相背对方向的另外一个超声波
      UltrasonicTriggerCommand(0x16-i);
      timer50msCounter5 = 0;  //计数清零
      PT_WAIT_UNTIL(pt, timer50msCounter5 >= 1); //如果时间满了50毫秒，则继续执行，否则记录运行点，退出线程 
    }
    for(i=0;i<6;i++)
    {
      if(UltrasonicReadCommand(0x11+i,&distance))
      {
        ultrasonicDistance[i] = distance;      
      }
      else ultrasonicDistance[i] = 0xffff;
    }
    #ifdef  U_OUT
    Serial.print("Ultrasonic:");
    for(i=0;i<6;i++)
    {         
      Serial.print(ultrasonicDistance[i],DEC);
      Serial.print("  ");
    }
    Serial.println(" ");
    #endif 
  }    

  PT_END(pt);  //线程结束
} 

/*--------------------------------------------------------------------------------------------
函数说明:  初始化函数
参数说明:  无
返回值  :  无
---------------------------------------------------------------------------------------------*/
void setup()
{ 
  Serial.begin(57600);    //COM用于debug
  Serial2.begin(19200);  //COM2用于读取URM04超声波
  Serial3.begin(57600);  //COM3用于Veyron电机驱动控制
  pinMode(13,OUTPUT);
  pinMode(dropIsrFL,INPUT);  //D2跌落传感器前左,机器人离地距离大于厘米输出高电平
  pinMode(dropIsrFR,INPUT);  //D3跌落传感器前右,机器人离地距离大于厘米输出高电平
  pinMode(dropIsrBL,INPUT);  //D21跌落传感器后左,机器人离地距离大于厘米输出高电平
  pinMode(dropIsrBR,INPUT);  //D20跌落传感器后右,机器人离地距离大于厘米输出高电平
  pinMode(crashIsrFL,INPUT);  //D19碰撞传感器左,机器人碰撞输出地电平  
  pinMode(crashIsrFM,INPUT);  //D17碰撞传感器前中,机器人碰撞输出地电平  
  pinMode(crashIsrFR,INPUT);  //D18碰撞传感器右,机器人碰撞输出地电平   
  
  Timer1.initialize(50000); //设置定时器每50000微秒,也就是50毫秒钟进入一次中断服务程序
  Timer1.attachInterrupt( timer1Isr ); //定义中断后的服务程序
  PT_INIT(&pt1);  //线程1初始化
  PT_INIT(&pt2);  //线程2初始化
  PT_INIT(&pt3);  //线程3初始化  
  PT_INIT(&pt4);  //线程4初始化 
  PT_INIT(&pt5);  //线程5初始化   
  motorCleanEncoder(1);
  motorCleanEncoder(2);
}

/*--------------------------------------------------------------------------------------------
函数说明:  主函数循环
参数说明:  无
返回值  :  无
---------------------------------------------------------------------------------------------*/
void loop () //这就是进行线程调度的地方
{  
  ReadSensorThread(&pt1);  //执行线程1
  MotoControlThread(&pt2);  //执行线程2
  LEDBlinkThread(&pt3);  //执行线程3
  MotoTestThread(&pt4);  //执行线程4  
  UltrasonicReadThread(&pt5);  //执行线程5 
} 

/*--------------------------------------------------------------------------------------------
函数说明:  机器人异常处理程序
参数说明:  无
返回值  :  无
---------------------------------------------------------------------------------------------*/
void abortHandle()
{
  M1Speed = 0;  //将左右两个电机的速度降低到0
  M2Speed = 0;  
  motorControl(M1Speed,M2Speed);
  abortStatus = true;
}

/*--------------------------测试电机进程------------------------------------------------------------
//--------------------------------------------------------------------------------------------
函数说明:  电机测试进程
参数说明:  *pt - ProtoThreads 操作系统指针 
返回值  :  无
---------------------------------------------------------------------------------------------//
static int MotoTestThread(struct pt *pt) 
{ 
  static int i;
  PT_BEGIN(pt); //线程开始
  while(1) 
  {  
    for(i=0;i<164;i++)
    {
      if(abortStatus == true) 
      {
        timer50msCounter4 = 0;  //计数清零delay
        PT_WAIT_UNTIL(pt, timer50msCounter4 >= 40); //如果时间满了2000毫秒，则继续执行，否则记录运行点，退出线程
        abortStatus = false;
      }
      M1Speed = i;  //设置左右两个电机的速度
      M2Speed = i;  
      motorControl(M1Speed,M2Speed);
      timer50msCounter4 = 0;  //计数清零
      PT_WAIT_UNTIL(pt, timer50msCounter4 >= 1); //如果时间满了50毫秒，则继续执行，否则记录运行点，退出线程 
    }
    for(i=0;i>-164;i--)
    {
      if(abortStatus == true) 
      {
        timer50msCounter4 = 0;  //计数清零
        PT_WAIT_UNTIL(pt, timer50msCounter4 >= 40); //如果时间满了2000毫秒，则继续执行，否则记录运行点，退出线程
        abortStatus = false;
      }
      M1Speed = i;  //设置左右两个电机的速度
      M2Speed = i;  
      motorControl(M1Speed,M2Speed);
      timer50msCounter4 = 0;  //计数清零
      PT_WAIT_UNTIL(pt, timer50msCounter4 >= 1); //如果时间满了50毫秒，则继续执行，否则记录运行点，退出线程
    }  
  } 
  PT_END(pt);  //线程结束
} 
-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------/



