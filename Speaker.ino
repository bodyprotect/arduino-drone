int speakerPin = 12;
int D_Condition = 0;

void speaker_initialize() {
  pinMode(speakerPin, OUTPUT);
}

void speaker_activate(int command)
{
  
  switch(command){
    case 0: //STOP NOISE
      break;
    
    case 1: //Start Sound
      tone(speakerPin, 500,500);
      delay(300);
      tone(speakerPin, 800,500);
      delay(300);
      tone(speakerPin, 1275,500);
      delay(300);
      tone(speakerPin, 1600,500);
      D_Condition = 2;
      break;

    case 2: //Good
      delay(2000);
      tone(speakerPin, 1500,500);
      delay(2000);
      break;
 
    case 11: //error sound 1
      tone(speakerPin, 2000,300);
      delay(500);
      break;

    case 12: //error sound 2
      tone(speakerPin, 1300,200);
      delay(200);
      tone(speakerPin, 1250,300);
      delay(500);
      break;

    case 13: //error sound 3
      tone(speakerPin, 1000,200);
      delay(400);
      tone(speakerPin, 1000,200);
      delay(400);
      tone(speakerPin, 1000,200);
      delay(400);
      tone(speakerPin, 1000,200);
      delay(3000);
      break;
  
  }
}
