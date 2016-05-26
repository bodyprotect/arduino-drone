#define Baro1
//#define Adafruit

#include <Wire.h>
#include "Arduino.h"
#include "i2c.h"

void setup()
{
	Serial.begin(9600);
	initS();
	LoadDefaults();

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
	
	currentTime = micros();
	cycleTime = currentTime - previousTime;
	previousTime = currentTime;
	ACC_getADC();
	Gyro_getADC();
	Device_Mag_getADC();
	getEstimatedAttitude();
	computeIMU();
#if defined (Baro1)
	Baro_update();
	getEstimatedAltitude();
#endif

	getPID();
	//double rollangle = getrollangle(imu.accADC[PITCH], imu.accADC[YAW], imu.gyroADC[PITCH]);
	//double pitchangle = getpitchangle(imu.accADC[ROLL], imu.accADC[YAW], imu.gyroADC[ROLL]);

	currenttime = millis();
	dtime = currenttime - presenttime ;
	if (dtime > 700)
	{
		Serial.print("ROLL angle :");
		Serial.print(att.angle[ROLL]);
		Serial.print("\t PITCH angle :");
		Serial.print(att.angle[PITCH]);
		Serial.print("\t heading :");
		Serial.print(att.heading);
#if defined(Baro1)
		Serial.print("\t alt :");
		Serial.print(alt.EstAlt);
		Serial.print("\t vel :");
		Serial.print(alt.vario);
		Serial.print("\t BaroPID :");
		Serial.print(BaroPID);
#endif

#if defined(Adafruit)
getalt();
		Serial.print("\t alt : ");
		Serial.print(totalAltitude);
#endif

		Serial.print("\t RPYPID :");
		Serial.print(axisPID[ROLL]);
		Serial.print("  ");
		Serial.print(axisPID[PITCH]);
		Serial.print("  ");
		Serial.print(axisPID[YAW]);

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

		//Serial.print("yawangle: ");
		//Serial.print(getheadingangle(imu.magADC[ROLL], imu.magADC[PITCH]));
		
		Serial.println("");
		presenttime = currenttime;
	}

	/*delay(1000);*/
}

