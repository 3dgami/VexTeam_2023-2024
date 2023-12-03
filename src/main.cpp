#include "main.h"

//update all motor ports if needed
pros::Controller master{CONTROLLER_MASTER};	
pros::Motor climb(13); 
pros::Motor intake(19);
pros::Motor launchN(14, true);
pros::Motor launchP(15);
pros::Motor right_front(20);
pros::Motor left_front(10);
pros::Motor left_back(1);//
pros::Motor right_back(11);
pros::Motor_Group driveL_train({left_front, left_back});
pros::Motor_Group driveR_train({right_front, right_back});
int rotationPort = 12;
int maxAngle = -10;
int minAngle = 1000000000;
int ShootPos = 8800;
int UpPos = 3500;


void SetDriveRelative(int ticks, int Lspeed, int Rspeed)
	{

		left_front.move_relative(-(ticks), Lspeed);
		left_back.move_relative(-(ticks), Lspeed);
		right_front.move_relative(ticks, Rspeed);
		right_back.move_relative(ticks, Rspeed);
	}

void SetDrive(int Lspeed, int Rspeed)
	{

		left_front.move(-(Lspeed));
		left_back.move(-(Lspeed));
		right_front.move(Rspeed);
		right_back.move(Rspeed);
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

	driveL_train.set_reversed(true);
	int startPos = getPos();
	double kp = 10.0;
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
	
	errorTerm = distance + startPos - getPos();

	while (errorTerm > 1 or errorTerm < -1)
	{
		errorTerm = distance + startPos - getPos();

		int Pos = getPos();

		errorTotal = errorTotal + errorTerm;

		if (errorTotal > 50 / ki)
		{
			errorTotal = 50 / ki;
		}


		P = errorTerm * kp;
		//I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + D) + (600*sign)));

		printf("O=%D, P=%0.2f, D=%0.2f, Position=%d, startPos=%d Err=%d\n",output, P, D, Pos, startPos, errorTerm);

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
	double CircleTicks = 2450;
	int turnTicks = (CircleTicks/360) * angle;



	int startPos = getPos();
	double kp = 7.0;
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

	if (angle < 0)
	{
		sign = -1;
	}
	else
	{
		sign = 1;
	}

	printf("start\n");
	while (errorTerm > 1 or errorTerm < -1)
	{
		errorTerm = (turnTicks + startPos) - floor(getPos());

		


		int pos = getPos();

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
		int output = ((P + D) + (1100*sign));

		printf("step err=%d, P=%.02f, D=%.02f, StartPos=%d, Pos=%d, O=%d\n, turn=%d", errorTerm, P, D, startPos, pos, output, turnTicks);


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
void competition_initialize()
{}

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
void autonomous() 
{
	auto ExpansionPort = 'H';
	auto ExpansionIntakePort = 'G';
	pros::c::adi_pin_mode(ExpansionPort, OUTPUT);
	pros::c::adi_digital_write(ExpansionPort, HIGH);
	pros::c::adi_pin_mode(ExpansionIntakePort, OUTPUT);
	pros::c::adi_digital_write(ExpansionIntakePort, HIGH);
	
	//1250 ticks = one block

	//score alliance ball
	driveTrain(1500);
	pros::delay(100);
	turn(-45);
	pros::delay(100);
	driveTrain(750);
	pros::delay(100);
	driveTrain(-800);

	//move to center to block opponent
	pros::delay(100);
	turn(-90);
	pros::delay(100);
	driveTrain(1100);
	pros::delay(100);
	turn(90);
	pros::delay(100);
	driveTrain(1250*1.5);


	//shut down all motors
	driveR_train.move_voltage(0);
	driveL_train.move_voltage(0);
	pros::c::adi_digital_write(ExpansionPort, LOW);



}

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

void opcontrol()
{
	auto ExpansionPort = 'H';
	auto ExpansionIntakePort = 'G';
	auto ExpansionClimbPort = 'F';

	pros::c::adi_pin_mode(ExpansionPort, OUTPUT);
	pros::c::adi_digital_write(ExpansionPort, HIGH);
	pros::c::adi_pin_mode(ExpansionIntakePort, OUTPUT);
	pros::c::adi_digital_write(ExpansionIntakePort, HIGH);
	pros::c::adi_pin_mode(ExpansionClimbPort, OUTPUT);
	pros::c::adi_digital_write(ExpansionClimbPort, LOW);

	pros::Rotation rotation_sensor(rotationPort);

	pros::Controller master(CONTROLLER_MASTER);


	bool intakeState = false;
	bool extend = false;
	bool extendIntake = false;
	bool extendClimb = false;

	bool climbPos = false;

	int dead_Zone = 10;
	int count = 0;

	int angle = rotation_sensor.get_angle();


	while(true)
	{


		/*USE TO CALIBRATE TURN SENSOR FOR LAUNCHER*/
		/*
		rotation_sensor.get_angle();
		int angle = rotation_sensor.get_angle();
		if(angle > maxAngle)
		{
			maxAngle = angle;		
		}

		if (angle < minAngle)
		{
			minAngle = angle;
		}
		printf("MaxAngle=%d; MinAngle=%d; currentAngle=%d \r\n", maxAngle, minAngle, angle);
		*/

		/*TANK CONTROL*/
		/*
		driveR_train.set_reversed(true);
		driveL_train.move(master.get_analog(ANALOG_LEFT_Y));
		driveR_train.move(master.get_analog(ANALOG_RIGHT_Y));
		*/
		
		/*ARCADE CONTROLL*/

		int power = -(master.get_analog(ANALOG_RIGHT_X));
		int turn = master.get_analog(ANALOG_LEFT_Y);

		int left = power - turn;
		int right = power + turn;

		driveL_train.move(left);
		driveR_train.move(right);
		
		
		//wing pneumatics (OPEN/CLOSE)
		if (master.get_digital_new_press(DIGITAL_L1))
		{
			if (extend == true)
			{
				pros::c::adi_digital_write(ExpansionPort, LOW);
				extend = false;
			}
			else
			{
				pros::c::adi_digital_write(ExpansionPort, HIGH);
				extend = true;
			}
		
			printf("Digital_L1 Pnuematic, Extend=%d \n", extend);
		}

		//launcher
		if (master.get_digital_new_press(DIGITAL_R1))
		{	

			launchN.move_relative(250, 300);
			launchP.move_relative(250, 300);
			pros::delay(200);// Ill try to lower this delay but the get_angle sometime doesnt get the end angle if no delay, but ill have to test
			angle = rotation_sensor.get_angle();

			while(angle < (ShootPos))
			{	
				if(angle >= (ShootPos))
				{
					break;
				}
				angle = rotation_sensor.get_angle();
				launchN.move_velocity(300);
				launchP.move_velocity(300);
				
				printf("angle=%d \n", angle);
				pros::delay(5);

			}
			launchN.move_velocity(0);
			launchP.move_velocity(0);

			printf("Digital_R1 launch \n");

		}

		//intake, expell ball (NEGATIVE)
		if (master.get_digital_new_press(DIGITAL_A))
		{	
			if (intakeState == false)
			{
				intake.move_velocity(-200);

				intakeState = true;

				printf("intakeState = true");
			}
			else
			{
				intake.move_velocity(0);

				intakeState = false;

				printf("intakeState = false");
			}
		}

		//intake, collect ball (POSITIVE)
		if (master.get_digital_new_press(DIGITAL_Y))
		{	
			if (intakeState == false)
			{
				intake.move_velocity(200);

				intakeState = true;

				printf("intakeState = true");
			}
			else
			{
				intake.move_velocity(0);

				intakeState = false;

				printf("intakeState = false");
			}
		
			printf("Digital_Y intake intakeState=%d \n", intakeState);
		}

		//intake pneumatics (UP/DOWN)
		if (master.get_digital_new_press(DIGITAL_X))
		{

			if (extendIntake == true)
			{
				pros::c::adi_digital_write(ExpansionIntakePort, LOW);

				intake.move_velocity(0);

				extendIntake = false;
			}
			else
			{
				pros::c::adi_digital_write(ExpansionIntakePort, HIGH);

				extendIntake = true;
			}
		
			printf("Digital_X Pnuematic Intake, Extend=%d \n", extend);

		}

		//launcher, move 100 ticks
		if (master.get_digital_new_press(DIGITAL_RIGHT))
		{
			launchN.move_relative(100, 50);
			launchP.move_relative(100, 50);
		}

		//launcher, move 500 ticks
		if (master.get_digital_new_press(DIGITAL_LEFT))
		{
			launchN.move_relative(500, 100);
			launchP.move_relative(500, 100);
		}

		//climb mech pneumatics (OPEN)
		while (master.get_digital(DIGITAL_R2) && master.get_digital(DIGITAL_L2))
		{
			
			if (!(count % 500))
			{
				printf("count = 500 \n");
				
				pros::c::adi_digital_write(ExpansionClimbPort, HIGH);
				climbPos = true;
    		}
    		count++;
    		pros::delay(2);
		}

		//climbing mech pulley, pull string (POSITIVE) 
		if(master.get_digital(DIGITAL_UP) and climbPos)
		{
			if(climb.get_target_velocity() != 0)
			{
				climb.move_velocity(0);
				printf("climbmotor 0 \n");
			}
			else
			{
				climb.move_velocity(150);
				printf("climbmotor positive \n");
			}
		}

		//climbing mech pulley, loosen string (NEGATIVE)
		if(master.get_digital(DIGITAL_DOWN) and climbPos)
		{	
			if(climb.get_target_velocity() != 0)
			{
				climb.move_velocity(0);
				printf("climbmotor 0 \n");
			}
			else
			{
				climb.move_velocity(-150);
				printf("climbmotor negative \n");
			}

		}
		
		count = 0;
		pros::delay(10);
		
	
	}
}

