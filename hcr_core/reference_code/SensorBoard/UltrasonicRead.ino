/*--------------------------------------------------------------------------------------------
函数说明:  触发一次超声波测距 
参数说明:  address - URM04超声波传感器测距传感器 
返回值  :  无
---------------------------------------------------------------------------------------------*/
void UltrasonicTriggerCommand(byte address)
{
  byte sendData[6] = {0x55,0xaa,0x11,0x00,0x01,0x11};
  sendData[2] = address;
  sendData[5] = address;  
  Serial2.write(sendData, 6);  //发送超声波触发命令,产生一次测距
}

/*--------------------------------------------------------------------------------------------
函数说明:  超声波测量到的距离读取回
参数说明:  address - URM04超声波传感器测距传感器 
           *distance  返回的测量距离 单位 厘米
返回值  :  true:读取正确  false :读取错误 
---------------------------------------------------------------------------------------------*/
boolean UltrasonicReadCommand(byte address,unsigned int *distance)
{
  byte temp1 = 0;
  byte sum = 0;
  byte sendData[6] = {0x55,0xaa,0x11,0x00,0x02,0x12};
  char dataBuffer[8];
  
  sendData[2] = address;
  sendData[5] = address + 1;  
  while(Serial2.available())  Serial.read();  //清接收缓存
  Serial2.write(sendData, 6);  //超声波测距值读取
 
  Serial2.setTimeout(8);  //串口接收超时设置   8毫秒
  temp1 = Serial2.readBytes(dataBuffer, 8);  //接收8个数据,如果超时将返回0,如果成功返回接收到的数据
  if(temp1 == 8)
  {
    if( ((byte)dataBuffer[0] == 0x55) && ((byte)dataBuffer[1] == 0xaa)  && ((byte)dataBuffer[2] == address) && ((byte)dataBuffer[3] == 0x02)) // 
    {
      sum = 0;
      for(temp1 = 0;temp1 < 7;temp1++)	  sum += (byte)dataBuffer[temp1];
      if(sum == (byte)dataBuffer[7])  //校验和正确
      {
        *distance = (byte)dataBuffer[5];
        *distance = *distance * 256 + (byte)dataBuffer[6];
        return true;
      }
      else return false;
    }
    else return false;    
  }
  else return false;
}


