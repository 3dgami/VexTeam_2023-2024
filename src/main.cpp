#include "main.h"
#include "selection.h"
#include "selection.ccp"
#include "lemlib/api.hpp"
#include "functions.cpp"
#include "lemlibinit.cpp"
#include "motors.cpp"

int rotationPort = 19;
int maxAngle = -10;
int minAngle = 1000000000;
int ShootPos = 8100;
int UpPos = 2153;

void on_center_button() {}


void screen() {

    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	selector::init();
	//Inertial_Sensor.reset(true /*true*/);

	//pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate the chassis
    //pros::Task screenTask(screen); // create a task to print the position to the screen

    /*
	chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
    chassis.setPose(5.2, 10.333, 87); // X: 5.2, Y: 10.333, Heading: 87
	*/
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
	int count;
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
		pros::delay(20);
		driveTrain(-1000, 1000);
		driveL_train.move_voltage(-5000);
		driveR_train.move_voltage(-7000);
		pros::delay(500);
		driveTrain(300, 750);
		pros::c::adi_digital_write(ExpansionHook, LOW);
		driveTrain(500, 1000);
		pros::delay(20);
		AbsGyroTurn(40);
		driveTrain(-500, 1000);
		driveTrain(300, 500);
		AbsGyroTurn(225);
		//pros::c::adi_digital_write(ExpansionPort1, HIGH);
		//pros::c::adi_digital_write(ExpansionPort2, HIGH);
		driveTrain(1300, 1000);
	

		

	}

	if(selector::auton == 2)
	{
		 //run auton for Far Red 
		pros::c::adi_digital_write(ExpansionHook, HIGH);
		pros::delay(20);
		driveTrain(1000, 1000);
		driveL_train.move_voltage(5000);
		driveR_train.move_voltage(7000);
		driveTrain(-300, 750);
		pros::c::adi_digital_write(ExpansionHook, LOW);
		driveTrain(500, 1000);
		pros::delay(20);
		AbsGyroTurn(320);
		driveTrain(500, 1000);
		pros::c::adi_digital_write(ExpansionPort1, HIGH);
		pros::c::adi_digital_write(ExpansionPort2, HIGH);
		driveTrain(1500, 1000);
		
	}

	if(selector::auton == 3)
	{
		 //do nothing 
	}

	if(selector::auton == -1)
	{
		 //run auton for Near Blue
		
	}

	if(selector::auton == -2)
	{
		 //run auton for Far Blue
	}

	if(selector::auton == -3)
	{
		 //do nothing
	}

	if(selector::auton == 0)
	{
		 //skills
		driveTrain(900, 1000);
		turn(80); //used to be 'turn(70)'
		driveTrain(-1000, 1000);
		pros::delay(25);

		driveL_train.move_voltage(100);
		driveR_train.move_voltage(-1000);
		launchN.move_velocity(300);
		launchP.move_velocity(300);
	
		pros::delay(28000);
	
		launchN.move_velocity(0);
		launchP.move_velocity(0);
		driveL_train.move_voltage(0);
		driveR_train.move_voltage(0);

		driveTrain(900, 1000);
		AbsGyroTurn(215);
		driveTrain(1400, 1000);
		AbsGyroTurn(270);
		driveTrain(3900, 2000);

		Movement moves[] = 
		{		
			Movement(8000, 4000, 1325), 			
			//Movement(100000, 100000, 1000),
			//Movement(-7500, -7500, 500),
			//Movement(100000, 100000, 750),
			//Movement(-7500, -7500, 750),

		};
		//SideTurn(1, 70);
		int stepCount = sizeof(moves)/sizeof(Movement);
		for(int i = 0; i<stepCount; i++)
		{
			printf("i=%d, sizeof%d \n", i,stepCount);
			driveL_train.move_voltage(-1*moves[i].left);
			driveR_train.move_voltage(moves[i].right);
			if(i == 1)
			{
				pros::c::adi_digital_write(ExpansionPort1, HIGH);
				//pros::c::adi_digital_write(ExpansionPort2, HIGH);
				printf("expand \n");
			}
			pros::delay(moves[i].delay);




		}
		
		driveTrain(1100, 1000);
		driveTrain(-1400, 1000);
		pros::c::adi_digital_write(ExpansionPort1, LOW);
		AbsGyroTurn(45); //used to be turn(-90)
		driveTrain(3000, 2000);
		
		AbsGyroTurn(270); // used to be turn(45)

		pros::c::adi_digital_write(ExpansionPort1, HIGH);
		pros::c::adi_digital_write(ExpansionPort2, HIGH);
		pros::delay(10);

		driveTrain(1600, 1000);

		pros::c::adi_digital_write(ExpansionPort1, LOW);
		pros::c::adi_digital_write(ExpansionPort2, LOW);
		pros::delay(10);

		driveTrain(-1300, 1000);
		AbsGyroTurn(0);
		driveTrain(1600, 1000);
		AbsGyroTurn(260);
		
		pros::c::adi_digital_write(ExpansionPort1, HIGH);
		pros::c::adi_digital_write(ExpansionPort2, HIGH);
		pros::delay(10);

		driveTrain(1900, 1000);

		pros::c::adi_digital_write(ExpansionPort1, LOW);
		pros::c::adi_digital_write(ExpansionPort2, LOW);
		
		driveTrain(-1500, 1000);

		AbsGyroTurn(0);

		driveTrain(1500, 1000);

		AbsGyroTurn(245);

		driveTrain(1800, 1000);

		
	}
	
	//driveTrain(1425);//oneblock

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

	bool extend = false;
	bool extendHook = false;

	int dead_Zone = 10;
	int count = 0;
	int left;
	int right;
	bool shootState = false;

	int angle = rotation_sensor.get_angle();
	


	while(true)
	{
		
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
		

		if(master.get_digital_new_press(DIGITAL_R2))
		{
			if(shootState == false)
			{
				launchN.move_velocity(150);
				launchP.move_velocity(150);
				shootState = true;
			}
			else
			{
				launchN.move_velocity(0);
				launchP.move_velocity(0);
				shootState = false;
			}

		}


		//Hook
		if (master.get_digital_new_press(DIGITAL_L2))
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
		}
			
		
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
			launchN.move_velocity(200);
			launchP.move_velocity(200);
			pros::delay(50);
			printf("first \n");
			//pros::delay(50);// Ill try to lower this delay but the get_angle sometime doesnt get the end angle if no delay, but ill have to test
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
				pros::delay(1);
			}
			launchN.move_velocity(0);
			launchP.move_velocity(0);

			printf("Digital_R1 launch \n");

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

		if (master.get_digital_new_press(DIGITAL_A))
		{
			if (extend == true)
			{
				pros::c::adi_digital_write(ExpansionHook, LOW);
				extend = false;
			}
			else
			{
				pros::c::adi_digital_write(ExpansionHook, HIGH);
				extend = true;
			}
		
			printf("Digital_L1 Pnuematic, Extend=%d \n", extend);
		}
		
		count = 0;
		pros::delay(5);
		
	
	}
}

