#include "main.h"
#include "selection.h"
#include "selection.ccp"
#include "lemlib/api.hpp"

/*pros::Rotation dead_left_x(7);
pros::Rotation dead_right_x(8);
pros::Rotation dead_y(9);*/
pros::Controller master{CONTROLLER_MASTER};	
pros::Motor right_front(20);
pros::Motor left_front(10);
pros::Motor left_back(1);//
pros::Motor right_back(11);
pros::Motor right_mid(6);
pros::Motor left_mid(5);
pros::Motor_Group driveL_train({left_front, left_back, left_mid});
pros::Motor_Group driveR_train({right_front, right_back, right_mid});
pros::IMU Inertial_Sensor(2);

lemlib::Drivetrain_t drivetrain {
    &driveL_train, // left drivetrain motors
    &driveR_train, // right drivetrain motors
    10, // track width
    3.25, // wheel diameter
    360 // wheel rpm
};

// Y tracking wheel
/*lemlib::TrackingWheel y_tracking_wheel(&dead_y, 2.75, -4.6); // 2.75" wheel diameter, -4.6" offset from tracking center UN-MEASURED
// X tracking wheels
lemlib::TrackingWheel x_left_tracking_wheel(&dead_right_x, 2.75, 1.7); // 2.75" wheel diameter, 1.7" offset from tracking center
lemlib::TrackingWheel x_right_tracking_wheel(&dead_left_x, 2.75, 4.5); // 2.75" wheel diameter, 4.5" offset from tracking center UN-MEASURED*/
 
// odometry struct
lemlib::OdomSensors_t sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // horizontal tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &Inertial_Sensor // inertial sensor
};
/*
    &y_tracking_wheel, // vertical tracking wheel 1
    &x_left_tracking_wheel, // horizontal tracking wheel 2
    &x_right_tracking_wheel*/
 
// forward/backward PID
lemlib::ChassisController_t lateralController {
    10, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
 
// turning PID
lemlib::ChassisController_t angularController {
    2, // kP
    10, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    40 // slew rate
};

// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);