#include "main.h"
#include "selection.h"
#include "selection.ccp"
#include "lemlib/api.hpp"
#include "motors.cpp"


struct Movement
{
	public:
	Movement(int l, int r, int d)
	{
		left = l;
		right = r;
		delay = d;
	}

	int left;
	int right;
    int delay;
};

void SetDriveRelative(int ticks, int Lspeed, int Rspeed)
	{

		left_front.move_relative(-(ticks), Lspeed);
		left_back.move_relative(-(ticks), Lspeed);
		left_mid.move_relative(-(ticks), Lspeed);
		right_front.move_relative(ticks, Rspeed);
		right_back.move_relative(ticks, Rspeed);
		right_mid.move_relative(ticks, Rspeed);
	}

void SetDrive(int Lspeed, int Rspeed)
	{
		left_mid.move(-(Lspeed));
		left_front.move(-(Lspeed));
		left_back.move(-(Lspeed));
		right_front.move(Rspeed);
		right_back.move(Rspeed);
		right_mid.move(Rspeed);
	}

double getLeftPos()
{
	return (left_front.get_position() + left_back.get_position() + left_mid.get_position()) / 3;
}

double getRightPos()
{
	return (right_front.get_position() + right_back.get_position() + right_mid.get_position()) / 3;
}

double getPos()
{
	return (getLeftPos() + getRightPos()) / 2;
}

void driveTrain(int distance, int timeout)
{

	driveL_train.set_reversed(true);
	int startPos = getPos();
	double kp = 9.00;
	double ki = 1.0;
	double kd = -8.50;   /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output PS 
						*/
	double P;
	double I = 0;
	double D;
	int lastError = 0;
	int errorTerm = 100000;
	int errorTotal = 0;
	int sign;
	int count = 0;

	sign = (distance < 0) ? -1 : 1;
	
	errorTerm = distance + startPos - getPos();

	while (errorTerm > 1 or errorTerm < -1 and count <= timeout)
	{
		if(count > 2000)
		{
			break;
			printf("TIMEOUT \n");
		}

		errorTerm = distance + startPos - getPos();

		int Pos = getPos();

		errorTotal = errorTotal + errorTerm;

		sign = (errorTerm < 0) ? -1 : 1;


		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;

		P = errorTerm * kp;
		//I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + I + D) + (1000 * sign)));

		printf("O=%D, P=%0.2f, D=%0.2f, Position=%d, startPos=%d Err=%d\n",output, P, D, Pos, startPos, errorTerm);

		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(20);
		count += 20;
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	printf("End\nErr=%d", errorTerm);
	driveL_train.set_reversed(false);

	return;
}

void turn(int angle)
{
	driveL_train.set_reversed(false);
	double CircleTicks = 2750;
	int turnTicks = (CircleTicks/360) * angle;
	int count = 0;



	int startPos = getPos();
	double kp = 11.0;
	double ki = 0.1;
	double kd = -5.50; /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output
						*/
	double P;
	double I;
	double D;
	int lastError = 0;
	int errorTerm;
	int errorTotal = 0;
	int sign = 1;

	sign = (angle > 0) ? 1 : -1;

	errorTerm = (turnTicks + startPos) - floor(getPos());


	printf("start\n");
	while (errorTerm > 1 or errorTerm < -1 and count <= 2000)
	{

		if(count > 2000)
		{
			break;
			printf("TIMEOUT \n");
		}

		errorTerm = (turnTicks + startPos) - floor(getPos());

		sign = (errorTerm < 0) ? -1 : 1;

		int pos = getPos();

		errorTotal = errorTotal + errorTerm;

		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;


		P = errorTerm * kp;
		I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + D)) + (1500 * sign));


		printf("step err=%d, P=%.02f, D=%.02f, StartPos=%d, Pos=%d, O=%d turn=%d count=%d \n", errorTerm, P, D, startPos, pos, output, turnTicks, count);


		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(10);
		count += 10;
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	driveL_train.set_reversed(true);

	pros::delay(10);
	printf("\nDone err=%d\n, O=%d", errorTerm, turnTicks);

	return;
}

void Centralturn(int angle, bool side)
{	
	driveL_train.set_reversed(true);
	double CircleTicks = 5400;
	int turnTicks = (CircleTicks/360) * angle;
	int count = 0;
	int startPos;
	double Pos;

	double kp = 2.0;
	double ki = 0.1;
	double kd = -5.50; /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output
						*/
	double P;
	double I;
	double D;
	int lastError = 0;
	int errorTerm;
	int errorTotal = 0;
	int sign = 1;

	sign = (angle > 0) ? 1 : -1;


	if(side == 1)
	{
		startPos = getRightPos();
	}
	
	else if(side == 0)
	{
		startPos = getLeftPos();
	}
	Pos = (side == 1) ? getRightPos() : getLeftPos();

	errorTerm = (turnTicks + startPos) - Pos;

	while(errorTerm > 1 or errorTerm < 1 and count <= 3000)
	{
		if(count > 3000)
		{
			break;
			printf("TIMEOUT \n");
		}

		Pos = (side == 1) ? getRightPos() : getLeftPos();
		errorTerm = (turnTicks + startPos) - floor(Pos);

		sign = (errorTerm < 0) ? -1 : 1;

		int pos = getPos();

		errorTotal = errorTotal + errorTerm;

		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;


		P = errorTerm * kp;
		I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + D)) + (1250 * sign));


		printf("step err=%d, P=%.02f, D=%.02f, StartPos=%d, Pos=%d, O=%d turn=%d count=%d \n", errorTerm, P, D, startPos, pos, output, turnTicks, count);

		if(side == 1)
		{
			driveR_train.move_voltage(output);
		}
		else if(side == 0)
		{
			driveL_train.move_voltage(output);
		}
		
		lastError = errorTerm;
		pros::delay(10);
		count += 10;
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);

	pros::delay(10);
	printf("\nDone err=%d\n, O=%d", errorTerm, turnTicks);

	return;
}

void RadTurn(double radius, int angle)
{

	driveL_train.set_reversed(true);
	int wheelRadius = 1.5;
	double CenterOffset = 7.5;
	
	double innerRad = abs(radius) - CenterOffset;
	double outerRad = abs(radius) + CenterOffset;

	double innerCirc = (2 * 3.14) * innerRad;
	double outerCirc = (2 * 3.14) * outerRad;
	double wheelCirc = (2 * 3.14) * 1.5;

	double ticksPerOuterCirc = (outerCirc / wheelCirc) * 1800;
	double ticksPerInnerCirc = (innerCirc / wheelCirc) * 1800; //1350tick per wheel revolution

	int distanceO = (ticksPerOuterCirc / 360) * angle;
	int distanceI = (ticksPerInnerCirc / 360) * angle;

	int IstartPos;
	int OstartPos;

	IstartPos = getLeftPos();
	OstartPos = getRightPos();

	int errorOTerm;
	int errorITerm;

	errorOTerm = distanceO + OstartPos - getRightPos();
	errorITerm = distanceI + IstartPos - getLeftPos();

	//driveL_train.set_reversed(true);

	if(angle > 0)
	{
		IstartPos = getRightPos();
		OstartPos = getLeftPos();
	}
	else if(angle < 0)
	{
		IstartPos = getLeftPos();
		OstartPos = getRightPos();
	}
	double kp = 9.00;
	double ki = 1.0;
	double kd = -7.50;  /* derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output PS */
						
	double InnerP;
	double InnerI;
	double InnerD;
	int lastErrorI = 0;
	int errorTermI = 100000;
	int errorTotalI = 0;
	int signI;

	double OuterP;
	double OuterI;
	double OuterD;
	int lastErrorO = 0;
	int errorTermO = 100000;
	int errorTotalO = 0;
	int signO;

	int Pos = getPos();
	int count = 0;

	signI = (distanceI < 0) ? -1 : 1;
	signO = (distanceO < 0) ? -1 : 1;
	
	if(angle > 0)
	{
		errorTermI = distanceI + IstartPos - getRightPos();
		errorTermO = distanceO + OstartPos - getLeftPos();

	}
	else if(angle < 0)
	{
		errorTermI = distanceI + IstartPos - getLeftPos();
		errorTermO = distanceO + OstartPos - getRightPos();

	}

	while (errorTermI > 1 or errorTermI < -1 or errorTermO > 1 or errorTermO < -1)//and count <= 2000
	{
		/*if(count > 2000)
		{
			break;
			printf("TIMEOUT \n");
		}*/
		
		if(angle > 0)
		{
			errorTermI = distanceI + IstartPos - getRightPos();
			errorTermO = distanceO + OstartPos - getLeftPos();

		}
		else if(angle < 0)
		{
			errorTermI = distanceI + IstartPos - getLeftPos();
			errorTermO = distanceO + OstartPos - getRightPos();
		}

		Pos = getPos();

		errorTotalI = errorTotalI + errorTermI;
		errorTotalO = errorTotalO + errorTermO;

		signI = (errorTermI < 0) ? -1 : 1;
		signO = (errorTermO < 0) ? -1 : 1;

		errorTotalI = (errorTotalI > 500 / ki) ? 500 / ki : errorTotalI;
		errorTotalO = (errorTotalO > 500 / ki) ? 500 / ki : errorTotalO;

		InnerP = errorTermI * kp;
		//InnerI = errorTotal * ki;
		InnerD = (lastErrorI - errorTermI) * kd;
		int outputI = (((InnerP + InnerI + InnerD) + (1000 * signI)));

		OuterP = errorTermO * kp;
		//OuterI = errorTotal * ki;
		OuterD = (lastErrorO - errorTermO) * kd;
		int outputO = (((OuterP + OuterI + OuterD) + (1000 * signO)));

		printf("I_O=%D, I_P=%0.2f, I_D=%0.2f, Position=%d, startPos=%d I_Err=%d\n",outputI, InnerP, InnerD, Pos, IstartPos, errorTermI);
		printf("O_O=%D, O_P=%0.2f, O_D=%0.2f, Position=%d, startPos=%d O_Err=%d\n",outputO, OuterP, OuterD, Pos, OstartPos, errorTermO);

		if(angle > 0)
		{
			driveL_train.move_voltage(outputO);
			driveR_train.move_voltage(outputI);
		}
		else if(angle < 0)
		{
			driveL_train.move_voltage(outputI);
			driveR_train.move_voltage(outputO);
		}

		lastErrorI = errorTermI;
		lastErrorO = errorTermO;
		pros::delay(20);
		count += 20;
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	printf("End Err_I=%d, Err_O=%d \n", errorTermI, errorTermO);
	driveL_train.set_reversed(false);

	return;
	
}

void gyroTurn(int angle)
{
	printf("start \n");
	driveL_train.set_reversed(true);
	driveR_train.set_reversed(true);

	double heading = Inertial_Sensor.get_heading();

	double kp = 80.0;
	double ki = 0.1;
	double kd = -6.50; /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output
						*/
	double P;
	double I;
	double D;
	int lastError = 0;
	double errorTerm;
	int errorTotal = 0;
	int sign = 1; 
	int count = 0;
	double ActualAngle;

	ActualAngle = (heading + angle) > 360 ? heading + angle - 360 : heading + angle;
	double diff = heading - ActualAngle;
	if(diff < -180)
	{
		errorTerm = -(360 + diff);
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	else if(diff > 180)
	{
		errorTerm = 360 - diff;
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	else
	{
		errorTerm = -diff;
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	errorTerm = fabs(errorTerm);


	printf("start\n");
	while (errorTerm > 0.5 or errorTerm < -0.5 and count <= 2000) 
	{

		if(count > 2000)
		{
			break;
			printf("TIMEOUT \n");
		}

		heading = Inertial_Sensor.get_heading();
		diff = heading - angle;
		if(diff < -180)
		{
			errorTerm = -(360 + diff);
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		else if(diff > 180)
		{
			errorTerm = 360 - diff;
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		else
		{
			errorTerm = -diff;
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		errorTerm = fabs(errorTerm);

		errorTotal = errorTotal + errorTerm;

		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;


		P = errorTerm * kp;
		I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + D) + 1500) * sign);


		printf("err=%0.2f, P=%.02f, D=%.02f, O=%d, heading=%0.2f count=%d \n", errorTerm, P, D, output, heading, count);


		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(10);
		count += 10;
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	driveL_train.set_reversed(true);
	driveR_train.set_reversed(false);

	pros::delay(100);
	printf("Heading=%0.2f \n", Inertial_Sensor.get_heading());

	return;
}

void AbsGyroTurn(int angle)
{
	printf("start \n");
	driveL_train.set_reversed(true);
	driveR_train.set_reversed(true);

	double heading = Inertial_Sensor.get_heading();

	double kp = 80.0;
	double ki = 0.1;
	double kd = -6.50; /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output
						*/
	double P;
	double I;
	double D;
	int lastError = 0;
	double errorTerm;
	int errorTotal = 0;
	int sign = 1; 
	int count = 0;

	double diff = heading - angle;
	if(diff < -180)
	{
		errorTerm = -(360 + diff);
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	else if(diff > 180)
	{
		errorTerm = 360 - diff;
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	else
	{
		errorTerm = -diff;
		sign = (errorTerm < 0) ? sign = -1 : sign = 1;
	}
	errorTerm = fabs(errorTerm);


	printf("start\n");
	while (errorTerm > 0.5 or errorTerm < -0.5 and count <= 2000) 
	{

		if(count > 2000)
		{
			break;
			printf("TIMEOUT \n");
		}

		heading = Inertial_Sensor.get_heading();
		diff = heading - angle;
		if(diff < -180)
		{
			errorTerm = -(360 + diff);
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		else if(diff > 180)
		{
			errorTerm = 360 - diff;
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		else
		{
			errorTerm = -diff;
			sign = (errorTerm < 0) ? sign = -1 : sign = 1;
		}
		errorTerm = fabs(errorTerm);

		errorTotal = errorTotal + errorTerm;

		errorTotal = (errorTotal > 500 / ki) ? 500 / ki : errorTotal;


		P = errorTerm * kp;
		I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + D) + 1500) * sign);


		printf("err=%0.2f, P=%.02f, D=%.02f, O=%d, heading=%0.2f count=%d \n", errorTerm, P, D, output, heading, count);


		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(10);
		count += 10;
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	driveL_train.set_reversed(true);
	driveR_train.set_reversed(false);

	pros::delay(100);
	printf("Heading=%0.2f \n", Inertial_Sensor.get_heading());

	return;
}

void gyroSturn(int distance, int angle, int time)
{
	
}
