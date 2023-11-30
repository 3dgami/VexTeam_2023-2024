#include "main.h"


pros::Controller master{CONTROLLER_MASTER};	
pros::Motor intake1(19);
//pros::Motor intake2(10, true);
pros::Motor launchN(14, true);	//update all motor ports
pros::Motor launchP(15);
pros::Motor right_front(20);
pros::Motor left_front(10);
pros::Motor left_back(1);//
pros::Motor right_back(11);
pros::Motor_Group driveL_train({left_front, left_back});
pros::Motor_Group driveR_train({right_front, right_back});
int rotationPort = 11;
int maxAngle = -10;
int minAngle = 1000000000;
int ShootPos = 11800;
int UpPos = 4702;


void SetDriveRelative(int ticks, int Lspeed, int Rspeed)
	{

		left_front.move_relative(-(ticks), Lspeed);
		//left_middle.move_relative(ticks, Lspeed);
		left_back.move_relative(-(ticks), Lspeed);

		right_front.move_relative(ticks, Rspeed);
		//right_middle.move_relative(ticks, Rspeed);
		right_back.move_relative(ticks, Rspeed);
	}

void SetDrive(int Lspeed, int Rspeed)
	{

		left_front.move(-(Lspeed));
		//left_middle.move(Lspeed);
		left_back.move(-(Lspeed));
		right_front.move(Rspeed);
		//right_middle.move(Rspeed);
		right_back.move(Rspeed);
	}

/*void driveU_train( int ticks)
{
	driveL_train.move_relative(-(ticks), 100);
	driveR_train.move_relative(ticks, 100);

}*/

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

void driveTrain(int distance, int RP, int LP)
{

	driveL_train.set_reversed(true);
	int startPos = getPos();
	double kp = 12.0;
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

		driveL_train.move_voltage(output + LP);
		driveR_train.move_voltage(output + RP);

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

void Move(int ticks, int Lspeed, int Rspeed, int timeOut)
	{
		int counter = 0;
		int startPos = getPos();

		SetDriveRelative(ticks, Lspeed * 127 / 200, Rspeed * 127 / 200);

		while (abs(getPos() - startPos) < abs(ticks) && counter <= timeOut)
		{
			pros::c::delay(10);
			counter = counter + 10;
		}

		SetDrive(0, 0);
		pros::c::delay(100);
	}

/**
 *
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
	auto ExpansionPort = 'A';
	auto ExpansionIntakePort = 'B';
	pros::c::adi_pin_mode(ExpansionPort, OUTPUT);
	pros::c::adi_digital_write(ExpansionPort, LOW);
	pros::c::adi_pin_mode(ExpansionIntakePort, OUTPUT);
	pros::c::adi_digital_write(ExpansionIntakePort, LOW);
	
	//driveTrain(-1250);//one block
	//driveTrain(1000, 0, 0);
	//pros::delay(500);
	//turn(-45);
	//pros::delay(500);
	driveTrain(500, 0, 2000);
	pros::delay(500);
	//driveTrain(-200);
	//pros::delay(500);
	//driveTrain(250);
	//pros::delay(500);
	//driveTrain(-400, 0, 0);

	/*pros::delay(500);
	turn(-90);
	pros::delay(500);
	driveTrain(1250, 0, 0);
	pros::delay(500);
	turn(90);
	pros::delay(500);
	driveTrain(1250, 0, 0);*/


	//shut down all motors
	//intake1.move_velocity(0);
	//intake2.move_velocity(0);
	//driveR_train.move_voltage(0);
	//driveL_train.move_voltage(0);

	//pros::c::adi_digital_write(ExpansionPort, LOW);



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
	auto ExpansionPort = 'A';
	auto ExpansionIntakePort = 'B';

	pros::c::adi_pin_mode(ExpansionPort, OUTPUT);
	pros::c::adi_digital_write(ExpansionPort, LOW);
	pros::c::adi_pin_mode(ExpansionIntakePort, OUTPUT);
	pros::c::adi_digital_write(ExpansionIntakePort, LOW);

	pros::Rotation rotation_sensor(rotationPort);

	pros::Controller master(CONTROLLER_MASTER);

	//rotation_sensor.set_position(0);

	bool intakeState = false;
	bool extend = true;//change to false
	bool extendIntake = false;

	int dead_Zone = 10;
	int count = 0;

	//pros::delay(500);
	//driveTrain(1500);
	//turn(90);
	//pros::delay(500);

	while(true)
	{

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
		

		/*TANK CONTROL*/

		//driveR_train.set_reversed(true);
		//driveL_train.move(master.get_analog(ANALOG_LEFT_Y));
		//driveR_train.move(master.get_analog(ANALOG_RIGHT_Y));
		
		/*ARCADE CONTROLL*/

		int power = -(master.get_analog(ANALOG_RIGHT_X));
		int turn = master.get_analog(ANALOG_LEFT_Y);

		int left = power - turn;
		int right = power + turn;

		driveL_train.move(left);
		driveR_train.move(right);
		
		
	
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

		if (master.get_digital_new_press(DIGITAL_R1) && extend == true)
		{	

			launchN.move_relative(150, 100);
			launchP.move_relative(150, 100);
			//angle = rotation_sensor.get_angle();
			pros::delay(250);// Ill try to lower this delay but the get_angle sometime doesnt get the end angle if no delay, but ill have to test
			angle = rotation_sensor.get_angle();

			while(angle < (ShootPos))
			{	
				if(angle >= (ShootPos))
				{
					break;
				}
				angle = rotation_sensor.get_angle();
				launchN.move_velocity(150);
				launchP.move_velocity(150);
				
				printf("angle=%d \n", angle);
				pros::delay(5);

			}
			launchN.move_velocity(0);
			launchP.move_velocity(0);

			printf("Digital_R1 launch \n");

		}

		if (master.get_digital_new_press(DIGITAL_A))
		{	
			if (intakeState == false)
			{
				intake1.move_velocity(-200);
				//intake2.move_velocity(-200);

				intakeState = true;

				printf("intakeState = true");
			}
			else
			{
				intake1.move_velocity(0);
				//intake2.move_velocity(0);

				intakeState = false;

				printf("intakeState = false");
			}
		}

		if (master.get_digital_new_press(DIGITAL_Y))
		{	
			if (intakeState == false)
			{
				intake1.move_velocity(200);
				//intake2.move_velocity(200);

				intakeState = true;

				printf("intakeState = true");
			}
			else
			{
				intake1.move_velocity(0);
				//intake2.move_velocity(0);

				intakeState = false;

				printf("intakeState = false");
			}
		
			printf("Digital_Y intake intakeState=%d \n", intakeState);
		}

		

		if (master.get_digital_new_press(DIGITAL_X))
		{

			if (extendIntake == true)
			{
				pros::c::adi_digital_write(ExpansionIntakePort, LOW);

				intake1.move_velocity(0);
				//intake2.move_velocity(0);

				extendIntake = false;
			}
			else
			{
				pros::c::adi_digital_write(ExpansionIntakePort, HIGH);
				extendIntake = true;
			}
		
			printf("Digital_X Pnuematic Intake, Extend=%d \n", extend);

		}
		if (master.get_digital_new_press(DIGITAL_RIGHT) && extend)
		{
			
		}

		if (master.get_digital_new_press(DIGITAL_LEFT) && extend)
		{
			launchN.move_relative(100, 50);
			launchP.move_relative(100, 50);
		}

		while (master.get_digital(DIGITAL_R2) && master.get_digital(DIGITAL_L2))
		{
			if (!(count % 500)){
      		
      		//master.print(0, 0, "Counter: %d", count);
    		}
    		count++;
    		pros::delay(2);
		}
		
		count = 0;
		pros::delay(10);
		
	
	}
}
//11460
//3884 = all the way up
//13034 = down 
//47.02 deg
//135.61
