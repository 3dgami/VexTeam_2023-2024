#include "main.h"


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

pros::Controller master{CONTROLLER_MASTER};	
pros::Motor intake(14);
pros::Motor launchN(12);	//update all motor ports
pros::Motor launchP(13);
pros::Motor right_front(11);
pros::Motor left_front(1);
pros::Motor left_back(10);
pros::Motor right_back(20);
pros::Motor_Group driveL_train({left_front, left_back});
pros::Motor_Group driveR_train({right_front, right_back});


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
	launchN.set_reversed(true);

	bool intakeState = false;
	bool extend = false;
	int dead_Zone = 10;

	while(true)
	{
		
		driveL_train.move(master.get_analog(ANALOG_LEFT_Y));
		driveR_train.move(master.get_analog(ANALOG_RIGHT_Y));
		
		/*
		ARCADE CONTROLL

		int power = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		int left = power + turn;
		int right = power - turn;
		driveL_train.move(left);
		driveR_train.move(right);
		*/
	
		if (master.get_digital_new_press(DIGITAL_L1))
		{
			if (extend == true)
			{
				piston.set_value(false);
				extend = false;
			}
			else
			{
				piston.set_value(true);
				extend = true;
			}
		
			printf("Digital_L1 Pnuematic, Extend=%d \n", extend);
		}

		if (master.get_digital_new_press(DIGITAL_R1))
		{
			launchN.move_relative(900 * 3, 100);
			launchP.move_relative(900 * 3, 100);
			printf("Digital_R1 launch \n");
		}

		if (master.get_digital_new_press(DIGITAL_Y))
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
		
			printf("Digital_Y intake intakeState=%d \n", intakeState);
		}

		if (master.get_digital_new_press(DIGITAL_X))
		{
			printf("Digital_X \n");
		}

		pros::delay(10);
	}
}