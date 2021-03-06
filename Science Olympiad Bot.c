#pragma config(Sensor, dgtl1,  twelve,         sensorDigitalIn)
#pragma config(Sensor, dgtl2,  button,         sensorDigitalIn)
#pragma config(Sensor, dgtl3,  rightEncoder,   sensorQuadEncoder)
#pragma config(Sensor, dgtl5,  nine,           sensorDigitalIn)
#pragma config(Sensor, dgtl6,  nineh,          sensorDigitalIn)
#pragma config(Sensor, dgtl7,  ten,            sensorDigitalIn)
#pragma config(Sensor, dgtl8,  tenh,           sensorDigitalIn)
#pragma config(Sensor, dgtl9,  eleven,         sensorDigitalIn)
#pragma config(Sensor, dgtl10, elevenh,        sensorDigitalIn)
#pragma config(Sensor, dgtl11, leftEncoder,    sensorQuadEncoder)
#pragma config(Motor,  port5,           leftFront,     tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           rightBack,     tmotorVex393TurboSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           rightFront,    tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port9,           leftBack,      tmotorVex393TurboSpeed_MC29, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)//Platform Type
#include <CKGeneral.h>

//Calculates the number of ticks in one inch of travel based on wheel size
int ticksForMeters(float meters)
{
	return (int)round(meters*360.0/(0.0635*PI));
}

//Sets Left Drive Train Motors to a power
void setLeftDriveTrainPower(int power)
{
	power = bound(power, -127, 127);
	motor[leftFront] = power;
	motor[leftBack] = power;
}

//Sets Right Drive Train Motors to a power
void setRightDriveTrainPower(int power)
{
	power = bound(power, -127, 127);
	motor[rightFront] = power;
	motor[rightBack] = power;
}

int readEncoder(int port){
	int v = SensorValue[port];
	//SensorValue[port] = 0;
	return v;
}

//Takes distanceToDrive (in inches), drives that distance
void driveADistance(float distanceToDrive)
{
	string dispStr;
	SensorValue[leftEncoder] = 0;
	SensorValue[rightEncoder] = 0;
	int error = 0;
	int leftDriven = 0;
	int rightDriven = 0;
	int goalDistance = ticksForMeters(distanceToDrive);
	float Kp = -0.01;
	bool isLeftDone = leftDriven > goalDistance;
	bool isRightDone = rightDriven > goalDistance;
	while (!isLeftDone || !isRightDone)
	{
		leftDriven = readEncoder(leftEncoder);
		rightDriven = -readEncoder(rightEncoder);
		sprintf(dispStr, "%d  %d", leftDriven, rightDriven);
writeDebugStream(dispStr);
		clearLCDLine(0);
		displayLCDString(0,0,dispStr);
		error = leftDriven - rightDriven; //+ if left has gone further, - if right has gone further
		isLeftDone = leftDriven > goalDistance;
		isRightDone = rightDriven > goalDistance;
		int offset = (int)round(Kp * error);
		offset = bound(offset, -20, 20);
	setLeftDriveTrainPower( isLeftDone ? 0 : (100 - offset) );
	setRightDriveTrainPower( isRightDone ? 0 : (100 + offset) );
		delay(100);
	}
	setLeftDriveTrainPower(0);
	setRightDriveTrainPower(0);
}

task main()
{
	if(SensorValue[nine] == 0)
		driveADistance(9);
	if(SensorValue[nineh] == 0)
		driveADistance(9.5);
	if(SensorValue[ten] == 0)
		driveADistance(10);
	if(SensorValue[tenh] == 0)
		driveADistance(10.5);
	if(SensorValue[eleven] == 0)
		driveADistance(11);
	if(SensorValue[elevenh] == 0)
		driveADistance(11.5);
	if(SensorValue[twelve] == 0)
		driveADistance(12);
}
