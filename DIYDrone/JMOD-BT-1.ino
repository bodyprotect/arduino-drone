
SoftwareSerial bluetooth(12,13);    // RX : Digital Pin 12, TX : Digital Pin 13

void bluetooth_initialize() 
{
  bluetooth.begin(115200);
  bluetooth.write("handshake");
}

/*bool isAvailable()
{
    return bluetooth.available();  
}*/

char getCommand()
{
  
  if(bluetooth.available()>0)
  {
    return bluetooth.read();  
  }
  else 
    return -1;
}

/*void send_response(char command)
{
  bluetooth.write(command);
}*/

/*void bluetooth_activate() {
  // put your main code here, to run repeat
  if(bluetooth.available())
    bluetooth.write("hello\n");
}*/
