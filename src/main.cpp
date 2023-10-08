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
pros::Motor right_front(11);
pros::Motor left_front(1);
pros::Motor left_back(10);
pros::Motor right_back(20);
pros::Motor_Group driveL_train({left_front, left_back});
pros::Motor_Group driveR_train({right_front, right_back});

int getLeftPos()
{
	return (left_front.get_position() + left_back.get_position()) / 2;
}

int getRightPos()
{
	return (right_front.get_position() + right_back.get_position()) / 2;
}

int getPos()
{
	return (getLeftPos() + getRightPos()) / 2;
}

void driveU_train(int distance, int velocity)
{
	driveL_train.move_relative(distance, velocity);
	driveR_train.move_relative(distance, velocity);

}


void driveTrain(int distance)
{
	int startPos = getPos();
	double kp = 17.0;
	double ki = 0.2;
	double kd = 5.0;
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

	

	while (errorTerm > 1 )
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
		int output = ((P + I + D) * sign);

		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(100);
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);

	return;
}

void turn(int angle)
{
	driveL_train.set_reversed(false);
	double CircleTicks = 1980.00;
	double turnTicks = (CircleTicks/360) * angle;
	

	int startPos = getPos();
	double kp = 10.0;
	//double ki = 0.2;
	double kd = 1.00;
	double P;
	//double I;
	double D;
	int lastError = 0;
	int errorTerm;
	int errorTotal = 0;
	int sign = 1;
	//printf("step err=%d, errT=%d, P=%.02f, D=%.02f", errorTerm, errorTotal, P, D);

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
		int output = (P + D) + 975;

		printf("step err=%d, errT=%d, P=%.02f, D=%.02f, O=%d\n", errorTerm, errorTotal, P, D, output);

		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(20);
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	driveL_train.set_reversed(true);

	pros::delay(1000);
	errorTerm = abs((turnTicks) + startPos) - getPos();
	printf("\nDone err=%d\n", errorTerm);

	return;
}


void opcontrol()
{

	driveL_train.set_reversed(true);
	int distance = 1450;// TILE
	turn(180);
	pros::delay(500);
	turn(180);
	/*pros::delay(500);
	driveTrain(1450);
	pros::delay(500);
	turn(90);
	pros::delay(500);
	turn(90);
	pros::delay(500);
	turn(180);
	pros::delay(500);
	driveTrain(1450);
	*/



}
 