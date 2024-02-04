
/*void DanielTurn(int distance1, int Turnangle)
{
	bool howmuch, distance1, Rpower, acdistr, Turnangle, Lpower, turn, extradistcuzturn, dbw, actdistl, distancer, distancel;
	dbw = 10.0;
  	extradistcuzturn = dbw * Turnangle;

	 while (!(acdistr == distance1)) {
    distancel = distance1 - extradistcuzturn;
    distancer = distance1 + extradistcuzturn;
    distance1 = (distancel + distancer) / 2.0;
    acdistr = getRightPos();
    actdistl = getLeftPos();
    // functions below basically make the robot only go distance1.
    Rpower = -1.0 * ((acdistr - (0.5 * (distancer + extradistcuzturn)) * (0.5 * (distancer + extradistcuzturn))) / (0.5 * (distancer + extradistcuzturn))) + 100.0;
    Lpower = -1.0 * ((actdistl - (0.5 * (distancel + extradistcuzturn)) * (0.5 * (distancel + extradistcuzturn))) / (0.5 * (distancel + extradistcuzturn))) + 100.0;
    driveR_train.move_velocity(Rpower);
    driveL_train.move_velocity(Lpower);
    
 	pros::delay(5000);

  }
  return;
}*/


void gyroTurn(int angle)
{
	printf("start \n");
	driveL_train.set_reversed(false);

	double heading = Inertial_Sensor.get_heading();
	//double target = heading + angle;

	while(heading > angle + 0.5 or heading < angle - 0.5)
	{
		driveL_train.move_velocity(-25);
		driveR_train.move_velocity(-25);
		heading = Inertial_Sensor.get_heading();
		pros::delay(2);
		printf("heading=%0.2f \n", heading);

	}
	driveL_train.move_velocity(0);
	driveR_train.move_velocity(0);
	pros::delay(150);
	printf("done, heading=%0.2f \n", heading);
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
			