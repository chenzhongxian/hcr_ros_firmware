/////////////////////////////////////////////////////////////////////////////////////////
//Setup Motors() function
/////////////////////////////////////////////////////////////////////////////////////////

// void SetupMotors()
// {
//   //Left motor
//   pinMode(A_1,OUTPUT);
//   pinMode(B_1,OUTPUT); 

//   //Right Motor
//   pinMode(A_2,OUTPUT);
//   pinMode(B_2,OUTPUT);

//   motorCleanEncoder();  
// }

/////////////////////////////////////////////////////////////////////////////////////////
//Will update both motors
/////////////////////////////////////////////////////////////////////////////////////////

void Update_Motors()
{
  // moveRightMotor(motor_right_speed);
  // moveLeftMotor(motor_left_speed);

  motorControl(M1Speed, M2Speed);

  // Serial.print("s");
  // Serial.print("\t");
  // Serial.print(motor_left_speed);
  // Serial.print("\t");
  // Serial.print(motor_right_speed);  
  // Serial.print("\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//Will update both encoder value through serial port
/////////////////////////////////////////////////////////////////////////////////////////

void Update_Encoders()
{
  char inString[14];
  // Serial2.println("1,GEP,1");
  // Serial2.setTimeout(10);  // 串口接收超时设置   8毫秒
  // Serial2.readBytes(inString, 13);
  Serial3.println("1,GEP,1");
  Serial3.setTimeout(10);  //串口接收超时设置   8毫秒
  Serial3.readBytes(inString, 13);
  Left_Encoder_Ticks = atoi(inString);
  
  // Serial2.println("1,GEP,2");
  // Serial2.setTimeout(10);  // 串口接收超时设置   8毫秒
  // Serial2.readBytes(inString, 13);
  Serial3.println("1,GEP,2");
  Serial3.setTimeout(10);  // 串口接收超时设置   8毫秒
  Serial3.readBytes(inString, 13);
  Right_Encoder_Ticks = atoi(inString);

  // Encoder reading bug fix
  if(Left_Encoder_Ticks == 0)
  {
    Left_Encoder_Ticks = Pre_Left_Encoder_Ticks;
  } else {
    Pre_Left_Encoder_Ticks = Left_Encoder_Ticks;
  }

  if(Right_Encoder_Ticks == 0)
  {
    Right_Encoder_Ticks = Pre_Right_Encoder_Ticks;
  } else {
    Pre_Right_Encoder_Ticks = Right_Encoder_Ticks;
  }

  Serial.print("e");
  Serial.print("\t");
  Serial.print(Left_Encoder_Ticks);
  Serial.print("\t");
  Serial.print(Right_Encoder_Ticks);
  Serial.print("\n");
}

/////////////////////////////////////////////////////////////////////////////////////////
//Set speed
/////////////////////////////////////////////////////////////////////////////////////////

void Set_Speed()
{
  // motor_left_speed = Messenger_Handler.readLong();
  // motor_right_speed = Messenger_Handler.readLong();

  M1Speed = Messenger_Handler.readInt();
  M2Speed = Messenger_Handler.readInt();

  // motor_left_speed = Messenger_Handler.readFloat();
  // motor_right_speed = Messenger_Handler.readFloat();

  // M1Speed = (int)(motor_left_speed / wheelSpeedMax * 255.0);  // 设置M1电机的速度
  // M2Speed = (int)(motor_right_speed / wheelSpeedMax * 255.0);

  if(M1Speed > 137)
  {
    M1Speed = 137;
  }
  if(M1Speed < -137)
  {
    M1Speed = -137;
  }

  if(M2Speed > 137)
  {
    M2Speed = 137;
  }
  if(M2Speed < -137)
  {
    M2Speed = -137;
  }
}

/*--------------------------------------------------------------------------------------------
函数说明:  控制两个电机转动
参数说明:  M1Speed，M2Speed ：电机1和2的转速 -164 至 164 数字越大速度越高.负号代表反方向
           0 代表 电机停止
返回值  :  无
---------------------------------------------------------------------------------------------*/
void motorControl(int M1Speed, int M2Speed)
{
  // char string[5];
  if(abortStatus == true) //如果机器人异常,控制速度将不起作用
  {
    M1Speed = 0;
    M2Speed = 0;
  }

  // Serial2.print("1,SVS,1,");
  // Serial2.println(M1Speed, DEC);

  // Serial2.print("1,SVS,2,");
  // Serial2.println(M2Speed, DEC);

  Serial3.print("1,SVS,1,");
  Serial3.println(M1Speed, DEC);

  Serial3.print("1,SVS,2,");
  Serial3.println(M2Speed, DEC);
}


/*--------------------------------------------------------------------------------------------
函数说明:  获取系统电压（ID,GSV\r\n）
参数说明:  voltage ：
返回值  :  无
---------------------------------------------------------------------------------------------*/
// void motorGetSystemVoltage(int *voltage)
// {
//   String inString = "";   
//   Serial2.println("1,GSV");
//   Serial2.setTimeout(10);  //串口接收超时设置   10毫秒
//   inString = Serial3.readString();  //
//   *voltage = inString.toInt(); //
// }

/*--------------------------------------------------------------------------------------------
函数说明:  获取编码器累计脉冲数（ID,GEP,*\r\n）通过该指令可获取编码器的累积脉冲数
           范围为：-2147483648~2147483647，驱动器对相位相差90°的编码器脉冲信号进行4倍频处理，
           即实际编码器若为200线，驱动器通过内部的硬件会将其变为200x4=800线，
           目前该驱动器仅支持增量式编码器。
参数说明:  Left_Encoder_Ticks  Right_Encoder_Ticks ： 编码器脉冲数 范围为：-2147483648~2147483647
返回值  :  无
---------------------------------------------------------------------------------------------*/
// void motorGetEncoder(long *Left_Encoder_Ticks,long *Right_Encoder_Ticks)
// {
//   char inString[14];
//   Serial2.println("1,GEP,1");
//   Serial2.setTimeout(10);  //串口接收超时设置   8毫秒
//   Serial2.readBytes(inString, 13);  //
//   *Left_Encoder_Ticks = atoi(inString); //
  
//   Serial2.println("1,GEP,2");
//   Serial2.setTimeout(10);  //串口接收超时设置   8毫秒
//   Serial2.readBytes(inString, 13);  //
//   *Right_Encoder_Ticks = atoi(inString); //  
// }

/*--------------------------------------------------------------------------------------------
函数说明:  清零编码器累计脉冲数（ID,CEP,*\r\n）
           通过该指令可将指定的某一路电机编码器累计脉冲数清零。
参数说明:  number ： 电机序列号, 1 :清零电机1   2:清零电机2
返回值  :  无
---------------------------------------------------------------------------------------------*/
void motorCleanEncoder()
{
  // Serial2.println("1,CEP,1");

  // Serial2.println("1,CEP,2");

  Serial3.println("1,CEP,1");

  Serial3.println("1,CEP,2");
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
  motorControl(M1Speed, M2Speed);
  abortStatus = true;
}

/////////////////////////////////////////////////////////////////////////////////////////
//Motor running function
/////////////////////////////////////////////////////////////////////////////////////////

// void moveRightMotor(float rightServoValue)
// {
//   if (rightServoValue>0)
//   {
//     digitalWrite(A_1,HIGH);
//     digitalWrite(B_1,LOW);
//     analogWrite(PWM_1,rightServoValue);
//   }
//   else if(rightServoValue<0)
//   {
//     digitalWrite(A_1,LOW);
//     digitalWrite(B_1,HIGH);
//     analogWrite(PWM_1,abs(rightServoValue));
//   }
//   else if(rightServoValue == 0)
//   {
//     digitalWrite(A_1,HIGH);
//     digitalWrite(B_1,HIGH);
//   }
// }

// void moveLeftMotor(float leftServoValue)
// {
//   if (leftServoValue > 0)
//   {
//     digitalWrite(A_2,LOW);
//     digitalWrite(B_2,HIGH);
//     analogWrite(PWM_2,leftServoValue);
//   }
//   else if(leftServoValue < 0)
//   {
//     digitalWrite(A_2,HIGH);
//     digitalWrite(B_2,LOW);
//     analogWrite(PWM_2,abs(leftServoValue));
//   }
//   else if(leftServoValue == 0)
//   {
//     digitalWrite(A_2,HIGH);
//     digitalWrite(B_2,HIGH);
//   }  
// }


