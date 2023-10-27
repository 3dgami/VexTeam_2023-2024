#include "main.h"

pros::ADIAnalogOut piston (1);
pros::Controller master{CONTROLLER_MASTER};	
pros::Motor intake(13);
pros::Motor launch(12);	//update all motor ports
pros::Motor right_front(11);
pros::Motor left_front(1);
pros::Motor left_back(10);
pros::Motor right_back(20);
pros::Motor_Group driveL_train({left_front, left_back});
pros::Motor_Group driveR_train({right_front, right_back});
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void driveU_train( int Rspeed, int Lspeed)
{
	driveL_train.move(Lspeed);
	driveR_train.move(Rspeed);

}

double getLeftPos()
{
	return (left_front.get_position() + left_back.get_position()) / 2;
}

double getRightPos()
{
	return (right_front.get_position() + right_back.get_position()) / 2;
}

double getPos()
{
	return (getLeftPos() + getRightPos()) / 2;
}

void driveTrain(int distance)
{
	int startPos = getPos();
	double kp = 15.0;
	double ki = 0.2;
	double kd = -0.15;   /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output PS 
						*/
	double P;
	double I;
	double D;
	int lastError = 0;
	int errorTerm;
	int errorTotal = 0;
	int sign;

	if (distance < 0){
		sign = -1;
		}
	else{
			sign = 1;
		}

	

	while (errorTerm > 1)
	{
		errorTerm = abs((distance) + startPos) - getPos();

		errorTotal = errorTotal + errorTerm;

		if (errorTotal > 50 / ki)
		{
			errorTotal = 50 / ki;
		}


		P = errorTerm * kp;
		I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + I + D) + 200)* sign);

		printf("O=%D, P=%0.2f, D=%0.2f, Err=%d\n",output, P, D, errorTerm);

		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(20);
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	printf("End\nErr=%d", errorTerm);

	return;
}

void turn(int angle)
{
	driveL_train.set_reversed(false);
	double CircleTicks = 1930;
	int turnTicks = floor((CircleTicks/360) * angle);

	int startPos = getPos();
	double kp = 9.0;
	//double ki = 0.2;
	double kd = -0.05; /*derivitive should control and stop overshooting this can be done
						  by having kd be negative or having a (P + I - D) for the output
						*/
	double P;
	//double I;
	double D;
	int lastError = 0;
	int errorTerm;
	int errorTotal = 0;
	int sign = 1;

	printf("start\n");
	while (errorTerm > 0)
	{
		errorTerm = (turnTicks + startPos) - getPos();

		errorTotal = errorTotal + errorTerm;
		/*
		if (errorTotal > 50 / ki)
		{
			errorTotal = 50 / ki;
		}
		*/	

		P = errorTerm * kp;
		//I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (P + D) + 900;

		printf("step err=%d, P=%.02f, D=%.02f, O=%d\n, turn=%d", errorTerm, P, D, output, turnTicks);

		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(10);
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	driveL_train.set_reversed(true);

	pros::delay(1000);
	errorTerm = abs((turnTicks) + startPos) - getPos();
	printf("\nDone err=%d\n, O=%d", errorTerm, turnTicks);

	return;
}


void opcontrol()
{

	pros::ADIAnalogOut piston (1);//get port
	pros::Controller master(CONTROLLER_MASTER);

	bool intakeState = false;
	bool extend = false;
	int dead_Zone = 10;

	int leftSpeed = 0;
	int rightSpeed = 0;
	int analogY = master.get_analog(ANALOG_LEFT_Y);
	int analogX = master.get_analog(ANALOG_LEFT_X);

	while(true)
	{
		
		if(analogY == 0 && abs(analogX) > dead_Zone)
		{
			leftSpeed = analogY;	
			rightSpeed = analogX;
		}
		else if(analogX >= dead_Zone && analogY > dead_Zone)
		{
			leftSpeed = analogY;
			rightSpeed = analogY - analogX;
		}
		else if(analogX < -dead_Zone && analogY > dead_Zone)
		{
			leftSpeed = analogY + analogX;
			rightSpeed = analogY;
		}
		else if(analogX >= dead_Zone && analogY < -dead_Zone)
		{
			leftSpeed = analogY;
			rightSpeed = analogY + analogX;
		}
		else if(analogX < -dead_Zone && analogY < - dead_Zone)
		{
			leftSpeed = analogY - analogX;
			rightSpeed = analogY;
		}
		else if(analogX == 0 && abs(analogY) > dead_Zone)
		{
			leftSpeed = analogY;
			rightSpeed = analogY;
		}

		if (master.get_digital(DIGITAL_L1))
		{
			if (extend != true)
			{
				piston.set_value(false);
				extend = false;
			}
			else
			{
				piston.set_value(true);
				extend = true;
			}
		
			printf("Digital_A Pnuematic, Extend=%d", extend);
		}

		if (master.get_digital(DIGITAL_R1))
		{
			launch.move_relative(1800 * 3, 100);
			printf("Digital_B launch");
		}

		if (master.get_digital(DIGITAL_Y))
		{	
			
			if (intakeState == false)
			{
				intake.move_velocity(200);
				intakeState = true;
			}
			else
			{
				intake.move_velocity(0);
				intakeState = false;
			}
		
			printf("Digital_A intake intakeState=%d", intakeState);
		}

		if (master.get_digital(DIGITAL_X))
		{
			printf("Digital_B");
		}

		if(abs(leftSpeed) < 40 && abs(rightSpeed) < 40)
		{
			driveU_train(rightSpeed,leftSpeed);
		}
		else
		{
			driveU_train(rightSpeed * 1.5, leftSpeed * 1.5);
		}
	}
}

	/*driveL_train.set_reversed(true);
	int distance = 1800;// TILE
	turn(180);
	pros::delay(500);
	turn(180);
	pros::delay(500);
	driveTrain(distance);
	pros::delay(500);
	turn(90);
	pros::delay(500);
	turn(90);
	pros::delay(500);
	driveTrain(distance);
	pros::delay(500);
	turn(180);*/
