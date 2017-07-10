/*******************************************************************************
* motor driver
*******************************************************************************/
void speedControl(int M1Speed, int M2Speed)
{
  // Serial2.println("1,CER");   // 清除所有故障信息

  // 同步转速，驱动器接收到命令马上驱动电机转动
  // Serial2.print("1,SVS,1,");
  // Serial2.println(M1Speed, DEC);

  // Serial2.print("1,SVS,2,");
  // Serial2.println(M2Speed, DEC);

  // 异步转速，驱动器接收到执行指令才开始驱动电机
  Serial2.print("1,SVR,1,");
  Serial2.println(M1Speed, DEC);

  Serial2.print("1,SVR,2,");
  Serial2.println(M2Speed, DEC);

  Serial2.println("1,AVR,1,1");   // 执行异步转速
}

void readEncoder()
{
  // Serial2.println("1,CER");   // 清除所有故障信息

  char inString[14];
  Serial2.println("1,GEP,1");
  Serial2.setTimeout(10);
  Serial2.readBytes(inString, 13);

  sensor_state_msg.left_encoder = atoi(inString);

  Serial2.println("1,GEP,2");
  Serial2.setTimeout(10);
  Serial2.readBytes(inString, 13);

  sensor_state_msg.right_encoder = atoi(inString);

  // Debug
  Serial4.print("e");
  Serial4.print("\t");
  Serial4.print(sensor_state_msg.left_encoder);
  Serial4.print("\t");
  Serial4.print(sensor_state_msg.right_encoder);
  Serial4.print("\n");
}

void cleanEncoder()
{
  Serial2.println("1,CER");   // 清除所有故障信息

  // 驱动器bug，第一次命令不会起效，所以第一个命令需要发送两次
  // Serial2.println("1,CEP,1");

  Serial2.println("1,CEP,1");
  Serial2.println("1,CEP,2");
}