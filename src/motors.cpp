#include "main.h"
#include "selection.h"
#include "selection.ccp"
#include "lemlib/api.hpp"



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
pros::Motor right_mid(6);
pros::Motor left_mid(5);
pros::Motor_Group driveL_train({left_front, left_back, left_mid});
pros::Motor_Group driveR_train({right_front, right_back, right_mid});
pros::IMU Inertial_Sensor(2);

auto ExpansionPort1 = 'D';
auto ExpansionPort2 = 'E';
auto ExpansionIntakePort1 = 'G';
auto ExpansionIntakePort2 = 'H';
auto ExpansionHook = 'A';
int count;
int angle;
