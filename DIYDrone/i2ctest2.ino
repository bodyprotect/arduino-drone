#include <Wire.h>
#include "Arduino.h"
#include "i2c.h"


void setup()
{
	Serial.begin(9600);
	initS();
	

	/* add setup code here */

}
  float currenttime = 0;
  float presenttime = 0;
  float dtime = 0;
void loop()
{

	
#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = -X; imu.accADC[PITCH]  = -Y; imu.accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] =  Y; imu.gyroADC[PITCH] = -X; imu.gyroADC[YAW] = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  =  X; imu.magADC[PITCH]  =  Y; imu.magADC[YAW]  = -Z;}
	ACC_getADC();
	Gyro_getADC();
	Device_Mag_getADC();
	getEstimatedAttitude();
	double rollangle = getrollangle(imu.accADC[PITCH], imu.accADC[YAW], imu.gyroADC[PITCH]);
	double pitchangle = getpitchangle(imu.accADC[ROLL], imu.accADC[YAW], imu.gyroADC[ROLL]);

	presenttime = millis();
	dtime = presenttime - currenttime;
	if (dtime > 700)
	{
		Serial.print("ROLL angle :");
		Serial.print(att.angle[ROLL]);
		Serial.print("\t PITCH angle :");
		Serial.print(att.angle[PITCH]);
		Serial.print("\t heading :");
		Serial.print(att.heading);
		
		//Serial.print("Mag:");                         //magnetometer values
		//Serial.print(imu.magADC[ROLL]);
		//Serial.print(',');
		//Serial.print(imu.magADC[PITCH]);
		//Serial.print(',');
		//Serial.print(imu.magADC[YAW]);
		//Serial.print(' ');

		//Serial.print("Acc:");                         //accelerometer values
		//Serial.print(imu.accADC[ROLL]);
		//Serial.print(',');
		//Serial.print(imu.accADC[PITCH]);
		//Serial.print(',');
		//Serial.print(imu.accADC[YAW]);
		//Serial.print(' ');

		//Serial.print("gyro:");                        //gyroscope values
		//Serial.print(imu.gyroADC[ROLL]);
		//Serial.print(',');
		//Serial.print(imu.gyroADC[PITCH]);
		//Serial.print(',');
		//Serial.print(imu.gyroADC[YAW]);

		//Serial.print(" rollangle: ");
		//Serial.print(rollangle);

		//Serial.print(" pitchangle: ");
		//Serial.print(pitchangle);

		Serial.print("\t alt : ");
		Serial.println(getalt());
		

		//Serial.print("yawangle: ");
		//Serial.println(getheadingangle(imu.magADC[ROLL], imu.magADC[PITCH]));
		
		currenttime = presenttime;
	}

	/*delay(1000);*/
}


//	/************************************** PID control**************************************/
//
//	uint8_t prop = 512;
//
//	// PITCH & ROLL
//	for (axis = 0; axis<2; axis++) {
//		rc = rcCommand[axis] << 1;
//		error = rc - imu.gyroData[axis];
//		errorGyroI[axis] = constrain(errorGyroI[axis] + error, -16000, +16000);       // WindUp   16 bits is ok here
//		if (abs(imu.gyroData[axis])>640) errorGyroI[axis] = 0;
//
//		ITerm = (errorGyroI[axis] >> 7)*conf.pid[axis].I8 >> 6;                        // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000
//
//		PTerm = mul(rc, conf.pid[axis].P8) >> 6;
//
//		if (f.ANGLE_MODE || f.HORIZON_MODE) { // axis relying on ACC
//			// 50 degrees max inclination
//			errorAngle = constrain(rc + GPS_angle[axis], -500, +500) - att.angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
//			errorAngleI[axis] = constrain(errorAngleI[axis] + errorAngle, -10000, +10000);                                                // WindUp     //16 bits is ok here
//
//			PTermACC = mul(errorAngle, conf.pid[PIDLEVEL].P8) >> 7; // 32 bits is needed for calculation: errorAngle*P8 could exceed 32768   16 bits is ok for result
//
//			int16_t limit = conf.pid[PIDLEVEL].D8 * 5;
//			PTermACC = constrain(PTermACC, -limit, +limit);
//
//			ITermACC = mul(errorAngleI[axis], conf.pid[PIDLEVEL].I8) >> 12;   // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result
//
//			ITerm = ITermACC + ((ITerm - ITermACC)*prop >> 9);
//			PTerm = PTermACC + ((PTerm - PTermACC)*prop >> 9);
//		}
//
//		PTerm -= mul(imu.gyroData[axis], dynP8[axis]) >> 6; // 32 bits is needed for calculation   
//
//		delta = imu.gyroData[axis] - lastGyro[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
//		lastGyro[axis] = imu.gyroData[axis];
//		DTerm = delta1[axis] + delta2[axis] + delta;
//		delta2[axis] = delta1[axis];
//		delta1[axis] = delta;
//
//		DTerm = mul(DTerm, dynD8[axis]) >> 5;        // 32 bits is needed for calculation
//
//		axisPID[axis] = PTerm + ITerm - DTerm;
//	}
//
//	//YAW
//#define GYRO_P_MAX 300
//#define GYRO_I_MAX 250
//
//	rc = mul(rcCommand[YAW], (2 * conf.yawRate + 30)) >> 5;
//
//	error = rc - imu.gyroData[YAW];
//	errorGyroI_YAW += mul(error, conf.pid[YAW].I8);
//	errorGyroI_YAW = constrain(errorGyroI_YAW, 2 - ((int32_t)1 << 28), -2 + ((int32_t)1 << 28));
//	if (abs(rc) > 50) errorGyroI_YAW = 0;
//
//	PTerm = mul(error, conf.pid[YAW].P8) >> 6;
//#ifndef COPTER_WITH_SERVO
//	int16_t limit = GYRO_P_MAX - conf.pid[YAW].D8;
//	PTerm = constrain(PTerm, -limit, +limit);
//#endif
//
//	ITerm = constrain((int16_t)(errorGyroI_YAW >> 13), -GYRO_I_MAX, +GYRO_I_MAX);
//
//	axisPID[YAW] = PTerm + ITerm;
//
//
//	mixTable();
//	// do not update servos during unarmed calibration of sensors which are sensitive to vibration
//
//	writeMotors();
//}
