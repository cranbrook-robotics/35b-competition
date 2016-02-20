#pragma config(UserModel, "C:/Users/rstudent/code/robot-configs/35B.c") //Cortex Configs
#pragma platform(VEX)//Platform Type
#pragma competitionControl(Competition) //This is a Competition Template
#pragma autonomousDuration(15) //15 second autonomous mode
#pragma userControlDuration(105) //1:45 driver control mode
#include "Vex_Competition_Includes.c" //Uses Vex stuff
#include <CKFlywheelSpeedController.h>


//Global Variable Declarations
float speed = 0;
tMotor motorPorts[] = { mFlyRT, mFlyLT, mFlyLB, mFlyRB };//35B
//IMEMotorSet imems;
//MovingAverage maVelocity;
//float targetV, measV, vError, cruisePower, power;
FlywheelSpeedController controller;




//Pre-Autonomous Functions - Initializes most variables
void pre_auton()
{
	bStopTasksBetweenModes = true;
	bLCDBacklight = true;
	clearLCDLine(0);
	clearLCDLine(1);
	//IMEMotorSetInit( imems, motorPorts, 4 );
	//MovingAverageInit( maVelocity, 6 );

	FlywheelSpeedControllerInit( controller, 0.065, 0, 0, 0.1026, 0.1888, motorPorts, 4 );

}

//Define Functions Needed for Operator Control and Auton
//float powerForSpeed( float speed ){
//	if(speed < 1){return 0;}
//	return 0.1026 * exp( 0.1888 * speed );
//}
float ticksForInches(float inches, float wheelDiameter)//Calculates the number of ticks in one inch of travel based on wheel size
{
	return PI*wheelDiameter*inches/90.0;
}
void setLeftDriveTrainPower(int power)//Sets Left Drive Train Motors to a power
{
	motor[leftFront] = power;
	motor[leftBack] = power;
}
void setRightDriveTrainPower(int power)//Sets Right Drive Train Motors to a power
{
	motor[rightDrive] = power;
}
void driveADistance(float distanceToDrive) //Takes distanceToDrive (in inches), drives that distance
{
	SensorValue[leftDrive] = 0;
	SensorValue[rightDrive] = 0;
	float leftDriven = SensorValue[leftDrive];
	float rightDriven = SensorValue[rightDrive];
	float goalDistance = ticksForInches(distanceToDrive, 4);
	float Kp = 0.05;
	int powerToLeftMotors = 100;
	int powerToRightMotors = 100;
	setLeftDriveTrainPower(powerToLeftMotors);
	setRightDriveTrainPower(powerToRightMotors);
	while (leftDriven < goalDistance || rightDriven < goalDistance)
	{
		float error = leftDriven - rightDriven; //+ if left has gone further, - if right has gone further
		powerToLeftMotors += -Kp * error;
		powerToRightMotors += Kp * error;
		setLeftDriveTrainPower(powerToLeftMotors);
		setRightDriveTrainPower(powerToRightMotors);
		delay(100);
	}
}

task flywheelSpeedUpdate()
{
	while(true)
	{
		//targetV = speed;
		//measure(imems);
		//nextSample(maVelocity, imems.ime.velocity);
		//measV = getAverage(maVelocity);
		//vError = measV - targetV;
		//cruisePower = powerForSpeed(targetV);
		//power = cruisePower + -0.05*sgn(vError)*vError*vError;
		//setPower(imems,bound(power,0,1));
		setTargetSpeed(controller,speed);
		update(controller);
		delay(50);
	}
}
task slowDelayStuff()
{
	string lcdBatteryVoltages, flywheelSpeedDisplay;
	while(true)
	{
		sprintf(lcdBatteryVoltages, "M: %.2f P: %.2f", MainBatteryVoltage(), SensorValue[pPowerExp]/280.0);
		sprintf(flywheelSpeedDisplay, "T: %.2f A: %.2f", controller.targetSpeed, controller.measuredSpeed); //flywheelMotors.ime.velocity
		clearLCDLine(0);
		clearLCDLine(1);
		displayLCDString(0,0,lcdBatteryVoltages);
		displayLCDString(1,0,flywheelSpeedDisplay);
		delay(300);
	}
}
task flywheelSpeedFromJoystick()
{
	int lastSpeedFromJoystickTime = nPgmTime;
	while(true)
	{
		bool up = (bool)vexRT[Btn8U];
		bool down = (bool)vexRT[Btn8D];
		if(nPgmTime - lastSpeedFromJoystickTime > 300)
		{
			if(up){speed += 0.5;}
			if(down){speed -= 0.5;}
			if(up || down)
			{
				lastSpeedFromJoystickTime = nPgmTime;
			}
		}
	}
}
void intakeIfRightSpeed()
{
	if(controller.targetSpeed == controller.measuredSpeed)
	{
		motor[chainIntake] = 127;
		motor[bandIntake] = 127;
	}
	else
	{
		motor[chainIntake] = 0;
		motor[bandIntake] = 0;
	}
}
//Autonomous
task autonomous()
{
	startTask(flywheelSpeedUpdate);
	speed = 3.5;
	driveADistance(75);
	while(true)
		intakeIfRightSpeed();
}

//User Control
task usercontrol()
{
	startTask(flywheelSpeedUpdate);
	startTask(flywheelSpeedFromJoystick);
	startTask(slowDelayStuff);

	while(true){

		speed = bound(speed,0,10); //Ensure speed is legal

		//LEDs
		if(speed <= 3.3){SensorValue[redLED] = 1;SensorValue[yellowLED] = 0;SensorValue[greenLED] = 0;}
		else if(speed <= 6.6){SensorValue[redLED] = 0;SensorValue[yellowLED] = 1;SensorValue[greenLED] = 0;}
		else{SensorValue[redLED] = 0;SensorValue[yellowLED] = 0;SensorValue[greenLED] = 1;}

		//Band Intake
		if(vexRT[Btn5U]){motor[bandIntake] = 127;}
		else if(vexRT[Btn5D]){motor[bandIntake] = -127;}
		else {motor[bandIntake] = 0;}

		//Chain Intake
		if(vexRT[Btn6U]){motor[chainIntake] = 127;}
		else if(vexRT[Btn6D]){motor[chainIntake] = -127;}
		else {motor[chainIntake] = 0;}

		//Drive Train
		motor[leftBack] = vexRT[Ch3];
		motor[leftFront] = vexRT[Ch3];
		motor[rightDrive] = vexRT[Ch2];

		//Set Flywheel Speed to Presets
		if(vexRT[Btn7D]){speed = 0;}
		if(vexRT[Btn7L]){speed = 3.3;}
		if(vexRT[Btn7U]){speed = 6.6;}
		if(vexRT[Btn7R]){speed = 10;}
	}
}
