
SoftwareSerial bluetooth(2,3);    // RX : Digital Pin2, TX : Digital Pin3

void bluetooth_initialize() {
  // put your setup code here, to run once:
  bluetooth.begin(115200);
  
}

bool isAvailable()
{
    return bluetooth.available();  
}

char getCommand()
{
    return bluetooth.read();  
}

void bluetooth_activate() {
  // put your main code here, to run repeat
  if(bluetooth.available())
    bluetooth.write("hello\n");
}
