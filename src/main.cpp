#include "main.h"
#include "lemlib/chassis/chassis.hpp"

// Motor/chassis objects

MotorGroup right_motors({13, 14, 15}, MotorGearset::blue);
MotorGroup left_motors({-16, -17,-7}, MotorGearset::blue);

// drivetrain settings
Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.5, // 11.5 inch track width
                              Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              4 // horizontal drift is 2 (for now)
);

Imu imu(11);

Rotation horizontal_rotation_sensor(-8); 

TrackingWheel horizontal_tracking_wheel(
        &horizontal_rotation_sensor,
        Omniwheel::NEW_325, 
        1.5, // located on the tracking center
        450
);

OdomSensors sensors(nullptr, 
                            nullptr, 
                            nullptr,
                            nullptr,
                            &imu // inertial sensor(none right now)
);

// lateral PID controller
ControllerSettings lateral_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              2, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              5, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
ControllerSettings angular_controller(1.8, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              13, // derivative gain (kD)
                                              6, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              5, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// // input curve for throttle input during driver control
// ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
//                                      10, // minimum output where drivetrain will move out of 127
//                                      1.019 // expo curve gain
// );

// input curve for throttle input during driver control
ExpoDriveCurve throttle_curve(8, // joystick deadband out of 127
                                     13, // minimum output where drivetrain will move out of 127
                                     1.09 // expo curve gain
);

// // input curve for steer input during driver control
// ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
//                                   10, // minimum output where drivetrain will move out of 127
//                                   1.019 // expo curve gain
// );

ExpoDriveCurve steer_curve(8, // joystick deadband out of 127
                                  18, // minimum output where drivetrain will move out of 127
                                  1.09 // expo curve gain
);

// create the chassis
Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve,
                        &steer_curve
                        // &throttle_curve, 
                        // &steer_curve
);

Controller controller(E_CONTROLLER_MASTER);

Motor intake_stg_1_motor(19, MotorGearset::rpm_200);
Motor intake_stg_2_motor(9, MotorGearset::blue);
Motor intake_stg_3_motor(20, MotorGearset::rpm_200);

adi::DigitalOut trapdoor('C', false);
adi::DigitalOut match_load('E', false);
adi::DigitalOut wing_descore('A', false);

// Helpers for moving mechanisms

void trapdoor_move(bool open_close){
        trapdoor.set_value(open_close);
}
void match_load_move(bool down_up){
        match_load.set_value(down_up);
}
void wing_descore_move(bool up_down){
        wing_descore.set_value(up_down);
}
void intake_stg1_move(bool intake){
        if (!intake){
                intake_stg_1_motor.move_velocity(200);
        }
        else{
                intake_stg_1_motor.move_velocity(-200);
        }
}
void intake_stg2_move(bool cycle){
        if (!cycle){
                intake_stg_2_motor.move_velocity(600);
        }
        else{
                intake_stg_2_motor.move_velocity(-600);
        }
}
void intake_stg3_move(bool score){
        if (!score){
                intake_stg_3_motor.move_velocity(60);
        }
        else{
                intake_stg_3_motor.move_velocity(-200);
        }
}
void intake_stg1_stop(){
        intake_stg_1_motor.move_velocity(0);
}
void intake_stg2_stop(){
        intake_stg_2_motor.move_velocity(0);
}
void intake_stg3_stop(){
        intake_stg_3_motor.move_velocity(0);
}
void intake_stg1_move_velocity_percent(int velocity){
        intake_stg_1_motor.move_velocity(velocity*2);
}
void intake_stg2_move_velocity_percent(int velocity){
        intake_stg_2_motor.move_velocity(velocity*6);
}

void intake_stg3_move_velocity_percent(int velocity){
        intake_stg_3_motor.move_velocity(velocity*2);
}


int loop_delay_ms = 20;
bool intake_stg3_O_F = false;
bool trapdoor_O_F = false;
bool loader_O_F = false;
bool intake_stg2_O_F = false;
bool wing_descore_O_F = false;


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	lcd::initialize();
	// screen::print(1, "Hello PROS User!");
        chassis.calibrate(true);
        cout << "=== Program started ===" << endl;
        // chassis.setPose(0, 0, 0);
	Task screenTask([&]() {
                while (true) {
                        // print robot location to the brain screen
                        lcd::print(0, "X: %f", chassis.getPose().x); // x
                        lcd::print(1, "Y: %f", chassis.getPose().y); // y
                        lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
                        cout << "(" << chassis.getPose().x << ", " << chassis.getPose().y << ")" << ",";
                        // log position telemetry
                        lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
                        // delay to save resources
                        delay(50);
                }
        });
        chassis.setBrakeMode(motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
        // lcd::register_btn1_cb(on_center_button);
        
        // Uncomment to choose starting pose in initialize
        // chassis.setPose(63, -16, 270); 
        // chassis.setPose(0, 0, 0);
        // autonomous(); // Uncomment this line to run autonomous at the start of the program
        wing_descore_move(false);

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
        intake_stg1_stop();
        intake_stg2_stop();
        intake_stg3_stop();
        chassis.arcade(0, 0);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
        // chassis.setPose(63, -16, 270);
}

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
void autonomous() {
        lcd::print(3, "Autonomous");
        
        /*
        match_load_move(false);
        start_angular_pid_logging_task(
            &chassis,
            &imu,
            angular_controller,
            180, // target in degrees
            5000, // timeout in ms
            loop_delay_ms
        );
        start_lateral_pid_logging_task(
            &chassis,
            &imu,
            lateral_controller,
            48, // target in inches
            8000, // timeout in ms
            loop_delay_ms
        );
        */

        
        chassis.setPose(63, 17, 270);
        // chassis.setPose(0, 0, 0);
        wing_descore_move(true);
        intake_stg2_move(true);
        intake_stg1_move(true);
        intake_stg3_move(false);
        match_load_move(false);
        
        // Take center balls
        chassis.moveToPoint(34, 16, 2000, {}, false);
        chassis.swingToPoint(22, 22, DriveSide::LEFT, 1000, {}, false);
        chassis.moveToPoint(22, 22, 1000, {}, false);
        // chassis.moveToPose(27, -21, 270, 100, {}, false);
        // Delay to allow balls to intake
        delay(500);
        // chassis.moveToPose(22, -21, 270, 500, {}, false);
        // delay(1000);

        
        // Turn and drive twoards the target point
        chassis.turnToPoint(50, 47, 1000, {}, false);
        chassis.moveToPoint(50, 47, 2000, {}, false);
        
        // Turn twoards loading zone and drive. Also lower the match load
        chassis.turnToPoint(66, 47, 1000, {}, false);
        match_load_move(true);
        delay(1000);
        chassis.moveToPose(66, 47, 90, 1000, {.minSpeed=70}, false);
        chassis.arcade(127, 0, true);
        // chassis.arcade(110,0);
        
        // Delay to allow for loading
        delay(800);
        
        // Move to goal and score
        chassis.moveToPose(27, 47, 90, 2000, {.forwards=false}, false);
        match_load_move(false);
        chassis.arcade(-127,0, true);
        intake_stg3_move(true);
        delay(5000);
        chassis.arcade(30 ,0, true);
        delay(500);
        chassis.arcade(-60, 0, true);
        
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
void opcontrol() {
        chassis.setBrakeMode(motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
        // wing_descore_move(true);
        while (true) {
                // get left y and right y positions
                int leftY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
                int leftX = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
                int rightY = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
                int rightX = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
                
                if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){
                        if (!intake_stg3_O_F){
                                intake_stg3_move(true);
                                intake_stg3_O_F = true;     
                        }
                        else{
                                intake_stg3_stop();
                                intake_stg3_O_F = false;
                        }
                }
                else if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_X)){
                        if(!intake_stg3_O_F){
                                intake_stg3_move(false);
                                intake_stg3_O_F = true;
                        }
                        else{
                                intake_stg3_stop();
                                intake_stg3_O_F = false;
                        }
                }
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
                        intake_stg2_move(true);
                }
                else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
                        intake_stg2_move(false);
                }
                else{
                        intake_stg2_stop();
                }
                // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
                //         if (!intake_stg2_O_F){
                //                 intake_stg2_move(true);
                //                 intake_stg2_O_F = true;
                //         }
                //         else{
                //                 intake_stg2_stop();
                //                 intake_stg2_O_F = false;
                //         }
                // }
                // else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
                //         if(!intake_stg2_O_F){
                //                 intake_stg2_move(false);
                //                 intake_stg2_O_F = true;
                //         }
                //         else{
                //                 intake_stg2_stop();
                //                 intake_stg2_O_F = false;
                //         }
                // }
                
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
                        intake_stg1_move(true);
                }
                else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
                        intake_stg1_move(false);
                }
                else{
                        intake_stg1_stop();
                }


                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
                        if (!trapdoor_O_F){
                                trapdoor_move(true);
                                trapdoor_O_F = true;
                        }
                        else{
                                trapdoor_move(false);
                                trapdoor_O_F = false;
                        }
                }
                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
                        if (!loader_O_F){
                                match_load_move(true);
                                loader_O_F = true;
                        }
                        else{
                                match_load_move(false);
                                loader_O_F = false;
                        }
                }
                if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
                        if(!wing_descore_O_F){
                                wing_descore_move(true);
                                wing_descore_O_F = true;
                        }
                        else{
                                wing_descore_move(false);
                                wing_descore_O_F = false;
                        }
                }
                
                // move the robot
                chassis.arcade(leftY, rightX);

                delay(10);
        }
}
