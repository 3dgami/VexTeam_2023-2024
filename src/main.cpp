#include "main.h"
#include "selection.h"
#include "selection.ccp"

//update all motor ports if needed
pros::Controller master{CONTROLLER_MASTER};	
pros::Motor climb(13); 
pros::Motor intake(6);
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
int ShootPos = 8500;
int UpPos = 2153;


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
	double kp = 13.00;
	double ki = 0.2;
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

	sign = (distance < 0) ? -1 : 1;
	
	errorTerm = distance + startPos - getPos();

	while (errorTerm > 1 or errorTerm < -1)
	{
		errorTerm = distance + startPos - getPos();

		int Pos = getPos();

		errorTotal = errorTotal + errorTerm;

		sign = (errorTerm < 0) ? -1 : 1;


		errorTotal = (errorTotal > 50 / ki) ? 50 / ki : errorTotal;

		P = errorTerm * kp;
		//I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + I + D) + (1000 * sign)));

		printf("O=%D, P=%0.2f, D=%0.2f, Position=%d, startPos=%d Err=%d\n",output, P, D, Pos, startPos, errorTerm);

		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(20);
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



	int startPos = getPos();
	double kp = 11.0;
	double ki = 0.2;
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

	sign = (sign < 0) ? -1 : 1;

	printf("start\n");
	while (errorTerm > 1 or errorTerm < -1)
	{
		errorTerm = (turnTicks + startPos) - floor(getPos());

		sign = (errorTerm < 0) ? -1 : 1;

		int pos = getPos();

		errorTotal = errorTotal + errorTerm;

		errorTotal = (errorTotal > 50 / ki) ? 50 / ki : errorTotal;


		P = errorTerm * kp;
		I = errorTotal * ki;
		D = (lastError - errorTerm) * kd;
		int output = (((P + I + D)) + (1250 * sign));


		printf("step err=%d, P=%.02f, D=%.02f, StartPos=%d, Pos=%d, O=%d turn=%d \n", errorTerm, P, D, startPos, pos, output, turnTicks);


		driveL_train.move_voltage(output);
		driveR_train.move_voltage(output);

		lastError = errorTerm;
		pros::delay(10);
	}
	driveL_train.move_voltage(0);
	driveR_train.move_voltage(0);
	driveL_train.set_reversed(true);

	pros::delay(10);
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
void initialize()
{
	selector::init();
}

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
	
	auto ExpansionPort1 = 'D';
	auto ExpansionPort2 = 'E';
	auto ExpansionIntakePort1 = 'G';
	auto ExpansionIntakePort2 = 'H';
	auto ExpansionHook = 'A';
	double POS = 0;
	int angle;

	pros::Rotation rotation_sensor(rotationPort);


	pros::c::adi_pin_mode(ExpansionPort1, OUTPUT);
	pros::c::adi_digital_write(ExpansionPort1, LOW);
	pros::c::adi_pin_mode(ExpansionPort2, OUTPUT);
	pros::c::adi_digital_write(ExpansionPort2, LOW);

	pros::c::adi_pin_mode(ExpansionIntakePort1, OUTPUT);
	pros::c::adi_digital_write(ExpansionIntakePort1, LOW);
	pros::c::adi_pin_mode(ExpansionIntakePort2, OUTPUT);
	pros::c::adi_digital_write(ExpansionIntakePort2, LOW);

	pros::c::adi_pin_mode(ExpansionHook, OUTPUT);
	pros::c::adi_digital_write(ExpansionHook, LOW);

	if(selector::auton == 1)
	{
		 //run auton for Near Red 
		pros::c::adi_digital_write(ExpansionHook, HIGH);
		pros::delay(100);
		driveTrain(-250);
		turn(45);
		turn(-90);
		driveTrain(1200);
		driveTrain(-500);
		driveTrain(500);
		driveTrain(-500);
		driveTrain(500);
		

	}

	if(selector::auton == 2)
	{
		 //run auton for Far Red 
		driveTrain(750);
		pros::c::adi_digital_write(ExpansionHook, HIGH);
		pros::delay(100);
		driveTrain(-500);
		pros::delay(100);
		pros::c::adi_digital_write(ExpansionHook, LOW);
		driveTrain(1000);
		turn(45);
		driveTrain(1425);
		driveTrain(-500);
		driveTrain(500);
		driveTrain(-500);
	}

	if(selector::auton == 3)
	{
		 //do nothing 
	}

	if(selector::auton == -1)
	{
		 //run auton for Near Blue
		pros::c::adi_digital_write(ExpansionHook, HIGH);
		pros::delay(100);
		driveTrain(-250);
		turn(45);
		turn(-90);
		driveTrain(1200);
		driveTrain(-500);
		driveTrain(500);
		driveTrain(-500);
		driveTrain(500);
		
	}

	if(selector::auton == -2)
	{
		 //run auton for Far Blue
		driveTrain(750);
		pros::c::adi_digital_write(ExpansionHook, HIGH);
		pros::delay(100);
		driveTrain(-750);
		pros::delay(100);
		pros::c::adi_digital_write(ExpansionHook, LOW);
		driveTrain(1000);
		turn(45);
		driveTrain(1425);
		driveTrain(-500);
		driveTrain(500);
		driveTrain(-500);
	}

	if(selector::auton == -3)
	{
		 //do nothing
	}

	if(selector::auton == 0)
	{
		 //skills
		while(true)
		{
			angle = rotation_sensor.get_angle();
			if(angle > (ShootPos))
			{
				pros::delay(250);
			}
			launchN.move_velocity(150);
			launchP.move_velocity(150);
			pros::delay(5);
		}
	}
	
	//driveTrain(1425);//oneblock



	/*pros::c::adi_digital_write(ExpansionHook, HIGH);
	turn(-90);
	pros::c::adi_digital_write(ExpansionHook, LOW);
	turn(90);
	driveTrain(1500);
	turn(-45);
	driveTrain(1300);
	driveTrain(300);
	turn(90);
	driveTrain(4500);
	pros::c::adi_digital_write(ExpansionHook, HIGH);*/


	/*
	
	//1500? ticks = one block
	pros::c::adi_digital_write(ExpansionHook, HIGH);
	driveTrain(500);
	driveTrain(-500);
	pros::c::adi_digital_write(ExpansionHook, LOW);
	turn(225); // tweak
	pros::c::adi_digital_write(ExpansionPort, HIGH);
	driveTrain(1600);
	turn(90);
	driveTrain(1300);
	*/

	//shut down all motors
	driveR_train.move_voltage(0);
	driveL_train.move_voltage(0);
	
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

	driveL_train.set_reversed(false);
	
	auto ExpansionPort1 = 'D';
	auto ExpansionPort2 = 'E';
	auto ExpansionIntakePort1 = 'G';
	auto ExpansionIntakePort2 = 'H';
	auto ExpansionHook = 'A';
	double POS = 0;

	pros::c::adi_pin_mode(ExpansionPort1, OUTPUT);
	pros::c::adi_digital_write(ExpansionPort1, LOW);
	pros::c::adi_pin_mode(ExpansionPort2, OUTPUT);
	pros::c::adi_digital_write(ExpansionPort2, LOW);

	pros::c::adi_pin_mode(ExpansionIntakePort1, OUTPUT);
	pros::c::adi_digital_write(ExpansionIntakePort1, LOW);
	pros::c::adi_pin_mode(ExpansionIntakePort2, OUTPUT);
	pros::c::adi_digital_write(ExpansionIntakePort2, LOW);

	pros::c::adi_pin_mode(ExpansionHook, OUTPUT);
	pros::c::adi_digital_write(ExpansionHook, LOW);

	pros::Rotation rotation_sensor(rotationPort);

	pros::Controller master(CONTROLLER_MASTER);


	bool intakeState = false;
	bool extend = false;
	bool extendIntake = false;
	bool extendClimb = false;
	bool extendHook = false;

	bool climbPos = false;

	int dead_Zone = 10;
	int count = 0;
	int left;
	int right;

	int angle = rotation_sensor.get_angle();
	


	while(true)
	{

		//printf("POS=%.2F \n", getPos());
		/*USE TO CALIBRATE TURN SENSOR FOR LAUNCHER*/
		
		/*rotation_sensor.get_angle();
		int angle = rotation_sensor.get_angle();
		if(angle > maxAngle)
		{
			maxAngle = angle;		
		}

		if (angle < minAngle)
		{
			minAngle = angle;
		}
		printf("MaxAngle=%d; MinAngle=%d; currentAngle=%d \r\n", maxAngle, minAngle, angle);*/
		

		/*TANK CONTROL*/
		/*
		driveR_train.set_reversed(true);
		driveL_train.move(master.get_analog(ANALOG_LEFT_Y));
		driveR_train.move(master.get_analog(ANALOG_RIGHT_Y));
		*/
		
		/*ARCADE CONTROLL*/

		int power = -(master.get_analog(ANALOG_RIGHT_X));
		int turn = master.get_analog(ANALOG_LEFT_Y);
		left = power - turn;
		right = power + turn;

		driveL_train.move(left);
		driveR_train.move(right);
		
		//upwing
		/*if (master.get_digital_new_press(DIGITAL_R2))
		{
			if(extendHook == true)
			{
				pros::c::adi_digital_write(ExpansionHook, LOW);
				extendHook = false;

			}
			else
			{
				pros::c::adi_digital_write(ExpansionHook, HIGH);
				extendHook = true;
		
			}
			printf("Digital_R2 Pnuematic, ExtendHook=%d \n", extendHook);
		}*/
			
		
		//wing pneumatics (OPEN/CLOSE)
		if (master.get_digital_new_press(DIGITAL_L1))
		{
			if (extend == true)
			{
				pros::c::adi_digital_write(ExpansionPort1, LOW);
				pros::c::adi_digital_write(ExpansionPort2, LOW);
				extend = false;
			}
			else
			{
				pros::c::adi_digital_write(ExpansionPort1, HIGH);
				pros::c::adi_digital_write(ExpansionPort2, HIGH);
				extend = true;
			}
		
			printf("Digital_L1 Pnuematic, Extend=%d \n", extend);
		}

		//launcher
		if (master.get_digital_new_press(DIGITAL_R1))
		{	

			launchN.move_relative(500, 150);
			launchP.move_relative(500, 150);
			pros::delay(200);// Ill try to lower this delay but the get_angle sometime doesnt get the end angle if no delay, but ill have to test
			angle = rotation_sensor.get_angle();

			while(angle < (ShootPos))
			{	

				if(angle > (ShootPos))
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
				pros::c::adi_digital_write(ExpansionIntakePort1, LOW);
				pros::c::adi_digital_write(ExpansionIntakePort2, LOW);

				intake.move_velocity(0);

				extendIntake = false;
			}
			else
			{
				pros::c::adi_digital_write(ExpansionIntakePort1, HIGH);
				pros::c::adi_digital_write(ExpansionIntakePort2, HIGH);

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
		/*if(master.get_digital(DIGITAL_R2) && master.get_digital(DIGITAL_L2))
		{	
			printf("count = 500 \n");

			if(climbPos == false)
			{
				pros::c::adi_digital_write(ExpansionClimbPort, HIGH);
				climbPos = true;
			}
			else
			{
				pros::c::adi_digital_write(ExpansionClimbPort, LOW);
				climbPos = false;
			}
		}

		//climbing mech pulley, pull string (POSITIVE) 
		if(master.get_digital_new_press(DIGITAL_UP) and climbPos)
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
		if(master.get_digital_new_press(DIGITAL_DOWN) and climbPos)
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

		}*/
			
		
		count = 0;
		pros::delay(10);
		
	
	}
}

