
/*--------------------------------------------------------------------------------------------
函数说明:  控制两个电机转动
参数说明:  M1Speed，M2Speed ：电机1和2的转速 -164 至 164 数字越大速度越高.负号代表反方向
           0 代表 电机停止
返回值  :  无
---------------------------------------------------------------------------------------------*/
void motorControl(int M1Speed,int M2Speed)
{
  char string[5];
  if(abortStatus == true) //如果机器人异常,控制速度将不起作用
  {
    M1Speed = 0;
    M2Speed = 0;
  }
  Serial3.print("1,SVS,1,");
  Serial3.println(M1Speed,DEC);

  Serial3.print("1,SVS,2,");
  Serial3.println(-M2Speed,DEC);//因为右边电机相对左边电机是 180度安装,所以控制方向需要取反
}


/*--------------------------------------------------------------------------------------------
函数说明:  获取系统电压（ID,GSV\r\n）
参数说明:  voltage ：
返回值  :  无
---------------------------------------------------------------------------------------------*/
void motorGetSystemVoltage(int *voltage)
{
  String inString = "";   
  Serial3.println("1,GSV");
  Serial3.setTimeout(10);  //串口接收超时设置   10毫秒
  inString = Serial3.readString();  //
  *voltage = inString.toInt(); //
}

/*--------------------------------------------------------------------------------------------
函数说明:  获取编码器累计脉冲数（ID,GEP,*\r\n）通过该指令可获取编码器的累积脉冲数
           范围为：-2147483648~2147483647，驱动器对相位相差90°的编码器脉冲信号进行4倍频处理，
           即实际编码器若为200线，驱动器通过内部的硬件会将其变为200x4=800线，
           目前该驱动器仅支持增量式编码器。
参数说明:  EncoderCount  EncoderCount2 ： 编码器脉冲数 范围为：-2147483648~2147483647
返回值  :  无
---------------------------------------------------------------------------------------------*/
void motorGetEncoder(long *EncoderCount1,long *EncoderCount2)
{
  char inString[14];
  Serial3.println("1,GEP,1");
  Serial3.setTimeout(10);  //串口接收超时设置   8毫秒
  Serial3.readBytes(inString,13);  //
  *EncoderCount1 = atoi(inString); //
  
  Serial3.println("1,GEP,2");
  Serial3.setTimeout(10);  //串口接收超时设置   8毫秒
  Serial3.readBytes(inString,13);  //
  *EncoderCount2 = atoi(inString); //  
}

/*--------------------------------------------------------------------------------------------
函数说明:  清零编码器累计脉冲数（ID,CEP,*\r\n）
           通过该指令可将指定的某一路电机编码器累计脉冲数清零。
参数说明:  number ： 电机序列号, 1 :清零电机1   2:清零电机2
返回值  :  无
---------------------------------------------------------------------------------------------*/
void motorCleanEncoder(byte number)
{
  Serial3.print("1,CEP,");
  Serial3.println(number,DEC);
}



