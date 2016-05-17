#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"

void initializeSoftPWM(void);

#if defined(SERVO)
void initializeServo();
#endif

/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins 
// its not possible to change a PWM output pin just by changing the order
#if defined(PROMINI)
  uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};   //for a quad+: rear,right,left,front
#endif


/**************************************************************************************/
/***************         Software PWM & Servo variables            ********************/
/**************************************************************************************/


#if defined(SERVO)
  #if defined(HW_PWM_SERVOS)
    // hw servo pwm does not need atomicServo[]
  #elif defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6))
    #if defined(AIRPLANE) || defined(HELICOPTER)
      // To prevent motor to start at reset. atomicServo[7]=5 or 249 if reversed servo
      volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,5};
    #else
      volatile uint8_t atomicServo[8] = {125,125,125,125,125,125,125,125};
    #endif
  #else
    #if defined(AIRPLANE)|| defined(HELICOPTER)
      // To prevent motor to start at reset. atomicServo[7]=5 or 249 if reversed servo
      volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,320};
    #else
      volatile uint16_t atomicServo[8] = {8000,8000,8000,8000,8000,8000,8000,8000};
    #endif
  #endif
#endif

/**************************************************************************************/
/***************       Calculate first and last used servos        ********************/
/**************************************************************************************/
#if defined(SERVO)
  #if defined(PRI_SERVO_FROM) && defined(SEC_SERVO_FROM)
    #if PRI_SERVO_FROM < SEC_SERVO_FROM
      #define SERVO_START PRI_SERVO_FROM
    #else
      #define SERVO_START SEC_SERVO_FROM
    #endif
  #else
    #if defined(PRI_SERVO_FROM)
      #define SERVO_START PRI_SERVO_FROM
    #endif
    #if defined(SEC_SERVO_FROM)
      #define SERVO_START SEC_SERVO_FROM
    #endif
  #endif
  #if defined(PRI_SERVO_TO) && defined(SEC_SERVO_TO)
    #if PRI_SERVO_TO > SEC_SERVO_TO
      #define SERVO_END PRI_SERVO_TO
    #else
      #define SERVO_END SEC_SERVO_TO
    #endif
  #else
    #if defined(PRI_SERVO_TO)
      #define SERVO_END PRI_SERVO_TO
    #endif
    #if defined(SEC_SERVO_TO)
      #define SERVO_END SEC_SERVO_TO
    #endif
  #endif


#endif

/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/
void writeServos() {
  #if defined(SERVO)
    #if defined(PRI_SERVO_FROM) && !defined(HW_PWM_SERVOS)   // write primary servos
      for(uint8_t i = (PRI_SERVO_FROM-1); i < PRI_SERVO_TO; i++){
        #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6)) || (defined(MEGA) && defined(MEGA_HW_PWM_SERVOS))
          atomicServo[i] = (servo[i]-1000)>>2;
        #else
          atomicServo[i] = (servo[i]-1000)<<4;
        #endif
      }
    #endif
    #if defined(SEC_SERVO_FROM) && !defined(HW_PWM_SERVOS)  // write secundary servos
      #if (defined(SERVO_TILT)|| defined(SERVO_MIX_TILT)) && defined(MMSERVOGIMBAL)
        // Moving Average Servo Gimbal by Magnetron1
        static int16_t mediaMobileServoGimbalADC[3][MMSERVOGIMBALVECTORLENGHT];
        static int32_t mediaMobileServoGimbalADCSum[3];
        static uint8_t mediaMobileServoGimbalIDX;
        uint8_t axis;

        mediaMobileServoGimbalIDX = ++mediaMobileServoGimbalIDX % MMSERVOGIMBALVECTORLENGHT;
        for (axis=(SEC_SERVO_FROM-1); axis < SEC_SERVO_TO; axis++) {
          mediaMobileServoGimbalADCSum[axis] -= mediaMobileServoGimbalADC[axis][mediaMobileServoGimbalIDX];
          mediaMobileServoGimbalADC[axis][mediaMobileServoGimbalIDX] = servo[axis];
          mediaMobileServoGimbalADCSum[axis] += mediaMobileServoGimbalADC[axis][mediaMobileServoGimbalIDX];
          #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6))
            atomicServo[axis] = (mediaMobileServoGimbalADCSum[axis] / MMSERVOGIMBALVECTORLENGHT - 1000)>>2;
          #else
            atomicServo[axis] = (mediaMobileServoGimbalADCSum[axis] / MMSERVOGIMBALVECTORLENGHT - 1000)<<4;
          #endif
        }
      #else
        for(uint8_t i = (SEC_SERVO_FROM-1); i < SEC_SERVO_TO; i++){
          #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6)) || (defined(MEGA) && defined(MEGA_HW_PWM_SERVOS))
            atomicServo[i] = (servo[i]-1000)>>2;
          #else
            atomicServo[i] = (servo[i]-1000)<<4;
          #endif
        }
      #endif
    #endif
    // write HW PWM servos for the mega
    #if defined(MEGA) && defined(MEGA_HW_PWM_SERVOS)
      #if (PRI_SERVO_FROM == 1 || SEC_SERVO_FROM == 1) 
        OCR5C = servo[0];
      #endif 
      #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
        OCR5B = servo[1];
      #endif 
      #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3) 
        OCR5A = servo[2];
      #endif 
      #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4) 
        OCR1A = servo[3];
      #endif 
      #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5) 
        OCR1B = servo[4];
      #endif 
      #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6) 
        OCR4A = servo[5];
      #endif 
      #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7) 
        OCR4B = servo[6];
      #endif 
      #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8) 
        OCR4C = servo[7];
      #endif
    #endif
    // write HW PWM servos for the promicro
    #if defined(PROMICRO) && defined(A32U4_4_HW_PWM_SERVOS)
      #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7)
        OCR1A = servo[6];// Pin 9
      #endif
      #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5)
        OCR1B = servo[4];// Pin 10
      #endif
      #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6)
        OCR3A = servo[5];// Pin 5
      #endif
      #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4)
        OCR1C = servo[3];// Pin 11
      #endif
    #endif
  #endif
}

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]


  /********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
  #if defined(PROMINI)
    #if (NUMBER_MOTOR > 0)
      #ifdef EXT_MOTOR_RANGE            // 490Hz
        OCR1A = ((motor[0]>>2) - 250);
      #elif defined(EXT_MOTOR_32KHZ)
        OCR1A = (motor[0] - 1000) >> 2; //  pin 9
      #elif defined(EXT_MOTOR_4KHZ)
        OCR1A = (motor[0] - 1000) << 1;
      #elif defined(EXT_MOTOR_1KHZ)
        OCR1A = (motor[0] - 1000) << 3;
      #else
        OCR1A = motor[0]>>3; //  pin 9
      #endif
    #endif
    #if (NUMBER_MOTOR > 1)
      #ifdef EXT_MOTOR_RANGE            // 490Hz
        OCR1B = ((motor[1]>>2) - 250);
      #elif defined(EXT_MOTOR_32KHZ)
        OCR1B = (motor[1] - 1000) >> 2; //  pin 10
      #elif defined(EXT_MOTOR_4KHZ)
        OCR1B = (motor[1] - 1000) << 1;
      #elif defined(EXT_MOTOR_1KHZ)
        OCR1B = (motor[1] - 1000) << 3;
      #else
        OCR1B = motor[1]>>3; //  pin 10
      #endif
    #endif
    #if (NUMBER_MOTOR > 2)
      #ifdef EXT_MOTOR_RANGE            // 490Hz
        OCR2A = ((motor[2]>>2) - 250);
      #elif defined(EXT_MOTOR_32KHZ)
        OCR2A = (motor[2] - 1000) >> 2; //  pin 11
      #elif defined(EXT_MOTOR_4KHZ)
        OCR2A = (motor[2] - 1000) >> 2;
      #elif defined(EXT_MOTOR_1KHZ)
        OCR2A = (motor[2] - 1000) >> 2;
      #else
        OCR2A = motor[2]>>3; //  pin 11
      #endif
    #endif
    #if (NUMBER_MOTOR > 3)
      #ifdef EXT_MOTOR_RANGE            // 490Hz
        OCR2B = ((motor[3]>>2) - 250);
      #elif defined(EXT_MOTOR_32KHZ)
        OCR2B = (motor[3] - 1000) >> 2; //  pin 3
      #elif defined(EXT_MOTOR_4KHZ)
        OCR2B = (motor[3] - 1000) >> 2;
      #elif defined(EXT_MOTOR_1KHZ)
        OCR2B = (motor[3] - 1000) >> 2;
      #else
        OCR2B = motor[3]>>3; //  pin 3
      #endif
    #endif
   
  #endif
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
  /****************            mark all PWM pins as Output             ******************/
  for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
    pinMode(PWM_PIN[i],OUTPUT);
  }
  
  /********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
  #if defined(PROMINI)
    #if defined(EXT_MOTOR_32KHZ)
      TCCR1A = (1<<WGM11); // phase correct mode & no prescaler
      TCCR1B = (1<<WGM13) | (1<<CS10);
      ICR1   = 0x00FF; // TOP to 255;
      TCCR2B =  (1<<CS20);
    #elif defined(EXT_MOTOR_4KHZ)
      TCCR1A = (1<<WGM11); // phase correct mode & no prescaler
      TCCR1B = (1<<WGM13) | (1<<CS10);
      ICR1   = 0x07F8; // TOP to 1023;     
      TCCR2B =  (1<<CS21);
    #elif defined(EXT_MOTOR_1KHZ)
      TCCR1A = (1<<WGM11); // phase correct mode & no prescaler
      TCCR1B = (1<<WGM13) | (1<<CS10);
      ICR1   = 0x1FE0; // TOP to 8184;     
      TCCR2B =  (1<<CS20) | (1<<CS21);
    #endif

    #if (NUMBER_MOTOR > 0)
      TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
    #endif
    #if (NUMBER_MOTOR > 1)
      TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    #endif
    #if (NUMBER_MOTOR > 2)
      TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
    #endif
    #if (NUMBER_MOTOR > 3)
      TCCR2A |= _BV(COM2B1); // connect pin 3 to timer 2 channel B
    #endif
  #endif

  /********  special version of MultiWii to calibrate all attached ESCs ************/
  #if defined(ESC_CALIB_CANNOT_FLY)
    writeAllMotors(ESC_CALIB_HIGH);
    blinkLED(2,20, 2);
    delay(4000);
    writeAllMotors(ESC_CALIB_LOW);
    blinkLED(3,20, 2);
    while (1) {
      delay(5000);
      blinkLED(4,20, 2);
      SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_2);
    }
    exit; // statement never reached
  #endif
  
  writeAllMotors(MINCOMMAND);
  delay(300);
  #if defined(SERVO)
    initializeServo();
  #endif
}


#if defined(SERVO)
/**************************************************************************************/
/************                Initialize the PWM Servos               ******************/
/**************************************************************************************/
void initializeServo() {
  #if !defined(HW_PWM_SERVOS)
  // do pins init
    #if (PRI_SERVO_FROM == 1) || (SEC_SERVO_FROM == 1)
      SERVO_1_PINMODE;
    #endif
    #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
      SERVO_2_PINMODE;
    #endif
    #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3) 
      SERVO_3_PINMODE;
    #endif 
    #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4) 
      SERVO_4_PINMODE;
    #endif 
    #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5) 
      SERVO_5_PINMODE;
    #endif 
    #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6) 
      SERVO_6_PINMODE;
    #endif 
    #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7) 
      SERVO_7_PINMODE;
    #endif 
    #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8) 
      SERVO_8_PINMODE;
    #endif
  #endif

  #if defined(SERVO_1_HIGH)
    #if defined(PROMINI) || (defined(PROMICRO) && defined(HWPWM6)) // uses timer 0 Comperator A (8 bit)
      TCCR0A = 0; // normal counting mode
      TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
      #define SERVO_ISR TIMER0_COMPA_vect
      #define SERVO_CHANNEL OCR0A
      #define SERVO_1K_US 250
    #endif
    #if (defined(PROMICRO) && !defined(HWPWM6)) // uses timer 3 Comperator A (11 bit)
      TCCR3A &= ~(1<<WGM30) & ~(1<<WGM31); //normal counting & no prescaler
      TCCR3B &= ~(1<<WGM32) & ~(1<<CS31) & ~(1<<CS32) & ~(1<<WGM33);
      TCCR3B |= (1<<CS30);   
      TIMSK3 |= (1<<OCIE3A); // Enable CTC interrupt 
      #define SERVO_ISR TIMER3_COMPA_vect
      #define SERVO_CHANNEL OCR3A
      #define SERVO_1K_US 16000 
    #endif
    #if defined(MEGA) // uses timer 5 Comperator A (11 bit)
      TCCR5A &= ~(1<<WGM50) & ~(1<<WGM51); //normal counting & no prescaler
      TCCR5B &= ~(1<<WGM52) & ~(1<<CS51) & ~(1<<CS52) & ~(1<<WGM53);
      TCCR5B |= (1<<CS50);   
      TIMSK5 |= (1<<OCIE5A); // Enable CTC interrupt  
      #define SERVO_ISR TIMER5_COMPA_vect
      #define SERVO_CHANNEL OCR5A
      #define SERVO_1K_US 16000 
    #endif
  #endif

  #if defined(MEGA) && defined(MEGA_HW_PWM_SERVOS)
    #if defined(SERVO_RFR_RATE)
      #if (SERVO_RFR_RATE < 20)
        #define SERVO_RFR_RATE 20
      #endif
      #if (SERVO_RFR_RATE > 400)
        #define SERVO_RFR_RATE 400
      #endif
    #else
      #if defined(SERVO_RFR_50HZ)
        #define SERVO_RFR_RATE 50
      #elif defined(SERVO_RFR_160HZ)
        #define SERVO_RFR_RATE 160
      #elif defined(SERVO_RFR_300HZ)
        #define SERVO_RFR_RATE 300
      #endif
    #endif  
    #define SERVO_TOP_VAL (uint16_t)(1000000L / SERVO_RFR_RATE)
    // init Timer 5, 1 and 4 of the mega for hw PWM
    TIMSK5 &= ~(1<<OCIE5A); // Disable software PWM  
    #if (PRI_SERVO_TO >= 1) || (SEC_SERVO_TO >= 1)
      TCCR5A |= (1<<WGM51);   // phase correct mode & prescaler to 8 = 1us resolution
      TCCR5A &= ~(1<<WGM50);
      TCCR5B &= ~(1<<WGM52) &  ~(1<<CS50) & ~(1<<CS52);
      TCCR5B |= (1<<WGM53) | (1<<CS51);
      ICR5 = SERVO_TOP_VAL;
      #if (PRI_SERVO_FROM == 1 || SEC_SERVO_FROM == 1) 
        pinMode(44,OUTPUT);
        TCCR5A |= (1<<COM5C1); // pin 44
      #endif
      #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
        pinMode(45,OUTPUT);
        TCCR5A |= (1<<COM5B1); // pin 45
      #endif
      #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3) 
        pinMode(46,OUTPUT);
        TCCR5A |= (1<<COM5A1); // pin 46
      #endif
    #endif
    #if (PRI_SERVO_TO >= 4) || (SEC_SERVO_TO >= 4) 
      TCCR1A |= (1<<WGM11); // phase correct mode & prescaler to 8
      TCCR1A &= ~(1<<WGM10);
      TCCR1B &= ~(1<<WGM12) &  ~(1<<CS10) & ~(1<<CS12);
      TCCR1B |= (1<<WGM13) | (1<<CS11);
      ICR1 = SERVO_TOP_VAL;
      #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4) 
        pinMode(11, OUTPUT);
        TCCR1A |= (1<<COM1A1); // pin 11
      #endif
      #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5) 
        pinMode(12,OUTPUT);
        TCCR1A |= (1<<COM1B1); // pin 12
      #endif
    #endif
    #if (PRI_SERVO_TO >= 6) || (SEC_SERVO_TO >= 6) 
      // init 16bit timer 4
      TCCR4A |= (1<<WGM41); // phase correct mode
      TCCR4A &= ~(1<<WGM40);
      TCCR4B &= ~(1<<WGM42) &  ~(1<<CS40) & ~(1<<CS42);
      TCCR4B |= (1<<WGM43) | (1<<CS41);
      ICR4 = SERVO_TOP_VAL;
      #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6) 
        pinMode(6,OUTPUT);
        TCCR4A |= _BV(COM4A1); // connect pin 6 to timer 4 channel A
      #endif
      #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7) 
        pinMode(7,OUTPUT);
        TCCR4A |= _BV(COM4B1); // connect pin 7 to timer 4 channel B
      #endif
      #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8) 
        #if defined(AIRPLANE) || defined(HELICOPTER)
          servo[7] =  MINCOMMAND;    // Trhottle at minimum for airplane and heli
          OCR4C = MINCOMMAND;
        #endif  
        pinMode(8,OUTPUT);
        TCCR4A |= _BV(COM4C1); // connect pin 8 to timer 4 channel C
      #endif
    #endif 
  #endif // mega hw pwm

  #if defined(PROMICRO) && defined(A32U4_4_HW_PWM_SERVOS)
    // atm. always initialize 4 servos to pins 9, 10, 11, 5
    TIMSK1 &= ~(1<<OCIE1A) & ~(1<<OCIE1B) & ~(1<<OCIE1C);
    TCCR1A |= (1<<WGM11); // phase correct mode & prescaler to 8
    TCCR1A &= ~(1<<WGM10);
    TCCR1B &= ~(1<<WGM12) &  ~(1<<CS10) & ~(1<<CS12);
    TCCR1B |= (1<<WGM13) | (1<<CS11);
    pinMode(9,OUTPUT);
    TCCR1A |= (1<<COM1A1); // pin 9
    pinMode(10,OUTPUT);
    TCCR1A |= (1<<COM1B1); // pin 10
    pinMode(11,OUTPUT);
    TCCR1A |= (1<<COM1C1); // pin 11

    TCCR3A |= (1<<WGM31); // phase correct mode & prescaler to 8
    TCCR3A &= ~(1<<WGM30);
    TCCR3B &= ~(1<<WGM32) &  ~(1<<CS30) & ~(1<<CS32);
    TCCR3B |= (1<<WGM33) | (1<<CS31);
    pinMode(5,OUTPUT);
    TCCR3A |= (1<<COM3A1); // pin 5
    #if defined(SERVO_RFR_RATE)
      #if (SERVO_RFR_RATE < 50) || (SERVO_RFR_RATE > 400)
        #error "* invalid SERVO_RFR_RATE specified"
      #endif
      #define SERVO_TOP_VAL (uint16_t)(1000000L / SERVO_RFR_RATE)
    #elif defined(SERVO_RFR_50HZ)
      #define SERVO_TOP_VAL 16700
    #elif defined(SERVO_RFR_160HZ)
      #define SERVO_TOP_VAL 6200
    #elif defined(SERVO_RFR_300HZ)
      #define SERVO_TOP_VAL 3300
    #else
      #error "* must set SERVO_RFR_RATE or one of the fixed refresh rates of 50, 160 or 300 Hz"
    #endif
    #if defined(SERVO_PIN5_RFR_RATE)
      #if (SERVO_PIN5_RFR_RATE < 50) || (SERVO_PIN5_RFR_RATE > 400)
        #error "* invalid SERVO_PIN5_RFR_RATE specified"
      #endif
      #define SERVO_PIN5_TOP_VAL (uint16_t)(1000000L / SERVO_PIN5_RFR_RATE)
    #else
      #define SERVO_PIN5_TOP_VAL SERVO_TOP_VAL
    #endif
    ICR1   = SERVO_TOP_VAL;      // set TOP timer 1
    ICR3   = SERVO_PIN5_TOP_VAL; // set TOP timer 3
  #endif // promicro hw pwm
}

/**************************************************************************************/
/************              Servo software PWM generation             ******************/
/**************************************************************************************/

// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 us

// for servo 2-8
// its almost the same as for servo 1
#if defined(SERVO_1_HIGH) && !defined(A32U4_4_HW_PWM_SERVOS)
  #define SERVO_PULSE(PIN_HIGH,ACT_STATE,SERVO_NUM,LAST_PIN_LOW) \
    }else if(state == ACT_STATE){                                \
      LAST_PIN_LOW;                                              \
      PIN_HIGH;                                                  \
      SERVO_CHANNEL+=SERVO_1K_US;                                \
      state++;                                                   \
    }else if(state == ACT_STATE+1){                              \
      SERVO_CHANNEL+=atomicServo[SERVO_NUM];                     \
      state++;                                                   \
  
  ISR(SERVO_ISR) {
    static uint8_t state = 0; // indicates the current state of the chain
    if(state == 0){
      SERVO_1_HIGH; // set servo 1's pin high 
      SERVO_CHANNEL+=SERVO_1K_US; // wait 1000us
      state++; // count up the state
    }else if(state==1){
      SERVO_CHANNEL+=atomicServo[SERVO_1_ARR_POS]; // load the servo's value (0-1000us)
      state++; // count up the state
    #if defined(SERVO_2_HIGH)
      SERVO_PULSE(SERVO_2_HIGH,2,SERVO_2_ARR_POS,SERVO_1_LOW); // the same here
    #endif
    #if defined(SERVO_3_HIGH)
      SERVO_PULSE(SERVO_3_HIGH,4,SERVO_3_ARR_POS,SERVO_2_LOW);
    #endif
    #if defined(SERVO_4_HIGH)
      SERVO_PULSE(SERVO_4_HIGH,6,SERVO_4_ARR_POS,SERVO_3_LOW);
    #endif
    #if defined(SERVO_5_HIGH)
      SERVO_PULSE(SERVO_5_HIGH,8,SERVO_5_ARR_POS,SERVO_4_LOW);
    #endif
    #if defined(SERVO_6_HIGH)
      SERVO_PULSE(SERVO_6_HIGH,10,SERVO_6_ARR_POS,SERVO_5_LOW);
    #endif
    #if defined(SERVO_7_HIGH)
      SERVO_PULSE(SERVO_7_HIGH,12,SERVO_7_ARR_POS,SERVO_6_LOW);
    #endif
    #if defined(SERVO_8_HIGH)
      SERVO_PULSE(SERVO_8_HIGH,14,SERVO_8_ARR_POS,SERVO_7_LOW);
    #endif
    }else{
      LAST_LOW;
      #if defined(SERVO_RFR_300HZ)
        #if defined(SERVO_3_HIGH)  // if there are 3 or more servos we dont need to slow it down
          SERVO_CHANNEL+=(SERVO_1K_US>>3); // 0 would be better but it causes bad jitter
          state=0; 
        #else // if there are less then 3 servos we need to slow it to not go over 300Hz (the highest working refresh rate for the digital servos for what i know..)
          SERVO_CHANNEL+=SERVO_1K_US;
          if(state<4){
            state+=2;
          }else{
            state=0;
          }
        #endif
      #endif
      #if defined(SERVO_RFR_160HZ)
        #if defined(SERVO_4_HIGH)  // if there are 4 or more servos we dont need to slow it down
          SERVO_CHANNEL+=(SERVO_1K_US>>3); // 0 would be better but it causes bad jitter
          state=0; 
        #else // if there are less then 4 servos we need to slow it to not go over ~170Hz (the highest working refresh rate for analog servos)
          SERVO_CHANNEL+=SERVO_1K_US;
          if(state<8){
            state+=2;
          }else{
            state=0;
          }
        #endif
      #endif   
      #if defined(SERVO_RFR_50HZ) // to have ~ 50Hz for all servos
        SERVO_CHANNEL+=SERVO_1K_US;
        if(state<30){
          state+=2;
        }else{
          state=0;
        }     
      #endif
    }
  } 
  #endif
#endif

/**************************************************************************************/
/************             Motor software PWM generation              ******************/
/**************************************************************************************/
// SW PWM is only used if there are not enough HW PWM pins (for exampe hexa on a promini)

#if (NUMBER_MOTOR > 4) && (defined(PROMINI) || defined(PROMICRO))

  /****************    Pre define the used ISR's and Timerchannels     ******************/
  #if !defined(PROMICRO)
    #define SOFT_PWM_ISR1 TIMER0_COMPB_vect
    #define SOFT_PWM_ISR2 TIMER0_COMPA_vect
    #define SOFT_PWM_CHANNEL1 OCR0B
    #define SOFT_PWM_CHANNEL2 OCR0A 
  #elif !defined(HWPWM6)
    #define SOFT_PWM_ISR1 TIMER3_COMPB_vect
    #define SOFT_PWM_ISR2 TIMER3_COMPC_vect
    #define SOFT_PWM_CHANNEL1 OCR3B
    #define SOFT_PWM_CHANNEL2 OCR3C 
  #else
    #define SOFT_PWM_ISR2 TIMER0_COMPB_vect  
    #define SOFT_PWM_CHANNEL2 OCR0B 
  #endif
  
  /****************         Initialize Timers and PWM Channels         ******************/
  void initializeSoftPWM(void) {
    #if !defined(PROMICRO)
      TCCR0A = 0; // normal counting mode
      #if (NUMBER_MOTOR > 4) && !defined(HWPWM6) 
        TIMSK0 |= (1<<OCIE0B); // Enable CTC interrupt  
      #endif
      #if (NUMBER_MOTOR > 6) || ((NUMBER_MOTOR == 6) && !defined(SERVO))
        TIMSK0 |= (1<<OCIE0A);
      #endif
    #else
      #if !defined(HWPWM6)
        TCCR3A &= ~(1<<WGM30) & ~(1<<WGM31); //normal counting & no prescaler
        TCCR3B &= ~(1<<WGM32) & ~(1<<CS31) & ~(1<<CS32) & ~(1<<WGM33);
        TCCR3B |= (1<<CS30);   
        TIMSK3 |= (1<<OCIE3B); // Enable CTC interrupt  
        #if (NUMBER_MOTOR > 6) || ((NUMBER_MOTOR == 6) && !defined(SERVO))
          TIMSK3 |= (1<<OCIE3C);
        #endif   
      #else
        TCCR0A = 0; // normal counting mode
        TIMSK0 |= (1<<OCIE0B); // Enable CTC interrupt 
      #endif
    #endif
  }
  
  /****************               Motor SW PWM ISR's                 ******************/
  // hexa with old but sometimes better SW PWM method
  // for setups without servos
  #if (NUMBER_MOTOR == 6) && (!defined(SERVO) && !defined(HWPWM6))
    ISR(SOFT_PWM_ISR1) { 
      static uint8_t state = 0;
      if(state == 0){
        if (atomicPWM_PIN5_highState>0) SOFT_PWM_1_PIN_HIGH;
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
        state = 1;
      }else if(state == 1){
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
        state = 2;
      }else if(state == 2){
        SOFT_PWM_1_PIN_LOW;
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
        state = 3;  
      }else if(state == 3){
        SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
        state = 0;   
      }
    }
    ISR(SOFT_PWM_ISR2) { 
      static uint8_t state = 0;
      if(state == 0){
        if (atomicPWM_PIN6_highState>0) SOFT_PWM_2_PIN_HIGH;
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_highState;
        state = 1;
      }else if(state == 1){
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_highState;
        state = 2;
      }else if(state == 2){
        SOFT_PWM_2_PIN_LOW;
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_lowState;
        state = 3;  
      }else if(state == 3){
        SOFT_PWM_CHANNEL2 += atomicPWM_PIN6_lowState;
        state = 0;   
      }
    }
  #else
    #if (NUMBER_MOTOR > 4) && !defined(HWPWM6)
      // HEXA with just OCR0B 
      ISR(SOFT_PWM_ISR1) { 
        static uint8_t state = 0;
        if(state == 0){
          SOFT_PWM_1_PIN_HIGH;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_highState;
          state = 1;
        }else if(state == 1){
          SOFT_PWM_2_PIN_LOW;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN6_lowState;
          state = 2;
        }else if(state == 2){
          SOFT_PWM_2_PIN_HIGH;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN6_highState;
          state = 3;  
        }else if(state == 3){
          SOFT_PWM_1_PIN_LOW;
          SOFT_PWM_CHANNEL1 += atomicPWM_PIN5_lowState;
          state = 0;   
        }
      } 
    #endif
    //the same with digital PIN A2 & 12 OCR0A counter for OCTO
    #if (NUMBER_MOTOR > 6)
      ISR(SOFT_PWM_ISR2) {
        static uint8_t state = 0;
        if(state == 0){
          SOFT_PWM_3_PIN_HIGH;
          SOFT_PWM_CHANNEL2 += atomicPWM_PINA2_highState;
          state = 1;
        }else if(state == 1){
          SOFT_PWM_4_PIN_LOW;
          SOFT_PWM_CHANNEL2 += atomicPWM_PIN12_lowState;
          state = 2;
        }else if(state == 2){
          SOFT_PWM_4_PIN_HIGH;
          SOFT_PWM_CHANNEL2 += atomicPWM_PIN12_highState;
          state = 3;  
        }else if(state == 3){
          SOFT_PWM_3_PIN_LOW;
          SOFT_PWM_CHANNEL2 += atomicPWM_PINA2_lowState;
          state = 0;   
        }
      }
    #endif
  #endif
#endif

/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/

// get servo middle point from Config or from RC-Data
int16_t get_middle(uint8_t nr) {
  return (conf.servoConf[nr].middle < RC_CHANS) ? rcData[conf.servoConf[nr].middle] : conf.servoConf[nr].middle;
}

// int8_t servodir(uint8_t n, uint8_t b) { return ((conf.servoConf[n].rate & b) ? -1 : 1) ; }

void mixTable() {
  int16_t maxMotor;
  uint8_t i;
  #if defined(DYNBALANCE)
    return;
  #endif
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z
  #define SERVODIR(n,b) ((conf.servoConf[n].rate & b) ? -1 : 1)

  /****************                   main Mix Table                ******************/
  #if defined( MY_PRIVATE_MIXING )
    #include MY_PRIVATE_MIXING

  #elif defined( QUADX )
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L

  #else
    #error "missing coptertype mixtable entry. Either you forgot to define a copter type or the mixing table is lacking neccessary code"
  #endif // MY_PRIVATE_MIXING

  /************************************************************************************************************/
  /****************************                Cam stabilize Servos             *******************************/

  #if defined(SERVO_TILT)
    servo[0] = get_middle(0);
    servo[1] = get_middle(1);
    if (rcOptions[BOXCAMSTAB]) {
      servo[0] += ((int32_t)conf.servoConf[0].rate * att.angle[PITCH]) /50L;
      servo[1] += ((int32_t)conf.servoConf[1].rate * att.angle[ROLL])  /50L;
    }
  #endif

  #ifdef SERVO_MIX_TILT
    int16_t angleP = get_middle(0) - MIDRC;
    int16_t angleR = get_middle(1) - MIDRC;
    if (rcOptions[BOXCAMSTAB]) {
      angleP += ((int32_t)conf.servoConf[0].rate * att.angle[PITCH]) /50L;
      angleR += ((int32_t)conf.servoConf[1].rate * att.angle[ROLL])  /50L;
    }
    servo[0] = MIDRC+angleP-angleR;
    servo[1] = MIDRC-angleP-angleR;
  #endif

/****************                    Cam trigger Servo                ******************/
  #if defined(CAMTRIG)
    // setup MIDDLE for using as camtrig interval (in msec) or RC channel pointer for interval control
    #define CAM_TIME_LOW  conf.servoConf[2].middle
    static uint8_t camCycle = 0;
    static uint8_t camState = 0;
    static uint32_t camTime = 0;
    static uint32_t ctLow;
    if (camCycle==1) {
      if (camState == 0) {
        camState = 1;
        camTime = millis();
      } else if (camState == 1) {
        if ( (millis() - camTime) > CAM_TIME_HIGH ) {
          camState = 2;
          camTime = millis();
          if(CAM_TIME_LOW < RC_CHANS) {
            ctLow = constrain((rcData[CAM_TIME_LOW]-1000)/4, 30, 250);
            ctLow *= ctLow;
          } else ctLow = CAM_TIME_LOW;
        }
      } else { //camState ==2
        if (((millis() - camTime) > ctLow) || !rcOptions[BOXCAMTRIG] ) {
          camState = 0;
          camCycle = 0;
        }
      }
    }
    if (rcOptions[BOXCAMTRIG]) camCycle=1;
    servo[2] =(camState==1) ? conf.servoConf[2].max : conf.servoConf[2].min;
    servo[2] = (servo[2]-1500)*SERVODIR(2,1)+1500;
  #endif

/************************************************************************************************************/
  // add midpoint offset, then scale and limit servo outputs - except SERVO8 used commonly as Moror output
  // don't add offset for camtrig servo (SERVO3)
  #if defined(SERVO)
    for(i=SERVO_START-1; i<SERVO_END; i++) {
      if(i < 2) {
        servo[i] = map(servo[i], 1020,2000, conf.servoConf[i].min, conf.servoConf[i].max);   // servo travel scaling, only for gimbal servos
      }
    #if defined(HELICOPTER) && (YAWMOTOR)
      if(i != 5) // not limit YawMotor
    #endif
        servo[i] = constrain(servo[i], conf.servoConf[i].min, conf.servoConf[i].max); // limit the values
    }
    #if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR == 6) && defined(PROMINI)
      servo[3] = servo[0];    // copy CamPitch value to propper output servo for A0_A1_PIN_HEX
      servo[4] = servo[1];    // copy CamRoll  value to propper output servo for A0_A1_PIN_HEX
    #endif
    #if defined(TRI) && defined(MEGA_HW_PWM_SERVOS) && defined(MEGA)
      servo[5] = constrain(servo[5], conf.servoConf[5].min, conf.servoConf[5].max); // servo[5] is still use by gui for this config (more genereic)
      servo[3] = servo[5];    // copy TRI serwo value to propper output servo for MEGA_HW_PWM_SERVOS
    #endif
  #endif

  /****************                compensate the Motors values                ******************/
  #ifdef VOLTAGEDROP_COMPENSATION
    {
      #if (VBATNOMINAL == 84) // 2S
        #define GOV_R_NUM 24
        static int8_t g[] = { 0,4,8,12,17,21,25,30,34,39,44,49,54,59,65,70,76,81,87,93,99,106,112,119,126 };
      #elif (VBATNOMINAL == 126) // 3S
        #define GOV_R_NUM 36
        static int8_t g[] = { 0,3,5,8,11,14,17,19,22,25,28,31,34,38,41,44,47,51,54,58,61,65,68,72,76,79,83,87,91,95,99,104,108,112,117,121,126 };
      #elif (VBATNOMINAL == 252) // 6S
        #define GOV_R_NUM 72
        static int8_t g[] = { 0,1,3,4,5,7,8,9,11,12,14,15,17,18,19,21,22,24,25,27,28,30,31,33,34,36,38,39,41,
            42,44,46,47,49,51,52,54,56,58,59,61,63,65,66,68,70,72,74,76,78,79,81,83,85,87,89,91,93,95,97,99,
            101,104,106,108,110,112,114,117,119,121,123,126        };
      #elif (VBATNOMINAL == 255) // 6S HV
        #define GOV_R_NUM 73
        static int8_t g[] = { 0,1,3,4,5,7,8,9,11,12,14,15,16,18,19,21,22,24,25,26,28,29,31,33,34,36,37,39,40,
             42,44,45,47,48,50,52,53,55,57,59,60,62,64,66,67,69,71,73,75,76,78,80,82,84,86,88,90,92,94,96,98,
             100,102,104,106,108,111,113,115,117,119,122,124,126        };
      #elif (VBATNOMINAL == 129) // 3S HV
        #define GOV_R_NUM 37
        static int8_t g[] = { 0,3,5,8,11,13,16,19,22,25,28,31,34,37,40,43,46,49,53,56,59,63,66,70,74,77,81,85,
             89,93,96,101,105,109,113,117,122,126         };
      #elif (VBATNOMINAL == 168) // 4S
        #define GOV_R_NUM 48
        static int8_t g[] = { 0,2,4,6,8,10,12,14,17,19,21,23,25,28,30,32,34,37,39,42,44,47,49,52,54,57,59,62,
             65,67,70,73,76,78,81,84,87,90,93,96,99,103,106,109,112,116,119,122,126    };
      #else
        #error "VOLTAGEDROP_COMPENSATION requires correction values which fit VBATNOMINAL; not yet defined for your value of VBATNOMINAL"
      #endif
      uint8_t v = constrain( VBATNOMINAL - constrain(analog.vbat, conf.vbatlevel_crit, VBATNOMINAL), 0, GOV_R_NUM);
      for (i = 0; i < NUMBER_MOTOR; i++) {
        motor[i] += ( ( (int32_t)(motor[i]-1000) * (int32_t)g[v] ) )/ 500;
      }
    }
  #endif
  /****************                normalize the Motors values                ******************/
    maxMotor=motor[0];
    for(i=1; i< NUMBER_MOTOR; i++)
      if (motor[i]>maxMotor) maxMotor=motor[i];
    for(i=0; i< NUMBER_MOTOR; i++) {
      if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] -= maxMotor - MAXTHROTTLE;
      motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
      if ((rcData[THROTTLE] < MINCHECK) && !f.BARO_MODE)
      #ifndef MOTOR_STOP
        motor[i] = conf.minthrottle;
      #else
        motor[i] = MINCOMMAND;
      #endif
      if (!f.ARMED)
        motor[i] = MINCOMMAND;
    }

  /****************                      Powermeter Log                    ******************/
  #if (LOG_VALUES >= 3) || defined(POWERMETER_SOFT)
  {
    static uint32_t lastRead = currentTime;
    uint16_t amp;
    uint32_t ampsum, ampus; // pseudo ampere * microseconds
    /* true cubic function;
     * when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 500
     * when divided by no_vbat=60 (6V) for 3 cell battery this gives maximum value of ~ 1000
     * */

    static uint16_t amperes[64] =   {   0,  2,  6, 15, 30, 52, 82,123,
                                     175,240,320,415,528,659,811,984,
                                     1181,1402,1648,1923,2226,2559,2924,3322,
                                     3755,4224,4730,5276,5861,6489,7160,7875,
                                     8637 ,9446 ,10304,11213,12173,13187,14256,15381,
                                     16564,17805,19108,20472,21900,23392,24951,26578,
                                     28274,30041,31879,33792,35779,37843,39984,42205,
                                     44507,46890,49358,51910,54549,57276,60093,63000};
  
    if (analog.vbat > NO_VBAT) { // by all means - must avoid division by zero
      ampsum = 0;
      for (i =0;i<NUMBER_MOTOR;i++) {
        amp = amperes[ ((motor[i] - 1000)>>4) ] / analog.vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp
        ampus = ( (currentTime-lastRead) * (uint32_t)amp * (uint32_t)conf.pint2ma ) / PLEVELDIVSOFT;
        #if (LOG_VALUES >= 3)
          pMeter[i]+= ampus; // sum up over time the mapped ESC input
        #endif
        #if defined(POWERMETER_SOFT)
          ampsum += ampus; // total sum over all motors
        #endif
      }
      #if defined(POWERMETER_SOFT)
        pMeter[PMOTOR_SUM]+= ampsum / NUMBER_MOTOR; // total sum over all motors
      #endif
    }
    lastRead = currentTime;
  }
  #endif
}
