/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package team3543;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

class RobotInfo3543
{
    static final String ROBOT_NAME                      = "Robot3543";
    //
    // DriveBase subsystem.
    //
    static final DcMotor.RunMode DRIVE_MOTOR_MODE       = DcMotor.RunMode.RUN_USING_ENCODER;
    static final double TURN_POWER_LIMIT                = 0.5;
    //
    // Velocity controlled constants.
    //
    static final double MOTOR_MAX_VELOCITY              = (2240 * 2.5); //encoder counts per second for HD HEX 40
    static final double MOTOR_KP                        = 0.75;
    static final double MOTOR_KI                        = 0.0;
    static final double MOTOR_KD                        = 0.0;
    //
    // 2019-10-28: Scale=0.0164402015697676, Kp=0.09, Ki=0.0, Kd=0.003
    // 2019-11-13: Scale=0.0144873520175266, Kp=0.09, Ki=0.0, Kd=0.001
    //
    static final double ENCODER_X_KP                    = 0.09;
    static final double ENCODER_X_KI                    = 0.0;
    static final double ENCODER_X_KD                    = 0.001;
    static final double ENCODER_X_TOLERANCE             = 2.0;
    static final double ENCODER_X_INCHES_PER_COUNT      = 0.0144873520175266;
    //
    // 2019-10-28: Scale=0.0176214934808376, Kp=0.05, Ki=0.0, Kd=0.003
    // 2019-11-13: Scale=0.0166031473564429, Kp=0.06, Ki=0.0, Kd=0.002
    //
    static final double ENCODER_Y_KP                    = 0.06;
    static final double ENCODER_Y_KI                    = 0.0;
    static final double ENCODER_Y_KD                    = 0.002;
    static final double ENCODER_Y_TOLERANCE             = 1.0;
    static final double ENCODER_Y_INCHES_PER_COUNT      = 0.0166031473564429;
    //
    // 2019-10-28: Kp=0.02, Ki=0.0, Kd=0.0015
    // 2019-11-13: Kp=0.02, Ki=0.0, Kd=0.001
    //
    static final double GYRO_KP                         = 0.02;
    static final double GYRO_KI                         = 0.0;
    static final double GYRO_KD                         = 0.001;
    static final double GYRO_TOLERANCE                  = 2.0;

    static final double PIDDRIVE_STALL_TIMEOUT          = 0.5;      //in seconds.

    // TODO: need to tune all PID coefficients
    static final double PURE_PURSUIT_POS_KP             = (.1 + .05) / 2.0; // average of encoder KP for x and y PID drive, as suggested by Abhay.
    static final double PURE_PURSUIT_POS_KI             = 0.0;
    static final double PURE_PURSUIT_POS_KD             = 0.0;
    static final double PURE_PURSUIT_TURN_KP            = 0.0125; // last tuned on 3543 RoverRuckus base, PDT 10-20-2019 1423 hrs
    static final double PURE_PURSUIT_TURN_KI            = 0.0;
    static final double PURE_PURSUIT_TURN_KD            = 0.00125; // last tuned on 3543 RoverRuckus base, PDT 10-20-2019 1429 hrs
    static final double PURE_PURSUIT_TURN_TOLERANCE     = 0.0;
    static final double PURE_PURSUIT_VEL_KP             = 0.0; // TODO: tune this, purepursuit is jerky and accelerates suddenly
    static final double PURE_PURSUIT_VEL_KI             = 0.0; // TODO: tune this, purepursuit is jerky and accelerates suddenly
    static final double PURE_PURSUIT_VEL_KD             = 0.9; // TODO: tune this. was placeholder FRC value. with this, purepursuit is jerky and accelerates suddenly

    // Neverest 40 motor, max shaft speed = 160 RPM
    // motor-to-wheel tooth ratio = 24:16 = 3:2
    // wheel max angular speed = (3 / 2) * 160 RPM
    // max tangential speed of wheel (in/s) = wheel max angular speed * 2 * pi * radius / 60.0
    // = (3 / 2) * (160 RPM) * 2 * 3.1415926 * (2 in.) / 60.0
    // = 50.2654816 in./sec.
    // KF should be set to the reciprocal of max tangential velocity (time to travel unit distance), units: sec./in.
    static final double PURE_PURSUIT_VEL_KF             = 1.0 / 50.2654816;

    //
    // Vision subsystem.
    //
    static final VuforiaLocalizer.CameraDirection CAMERA_DIR = BACK;
    static final VuforiaLocalizer.Parameters.CameraMonitorFeedback CAMERA_MONITOR_FEEDBACK =
            VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
    static final boolean PHONE_IS_PORTRAIT              = false;
    static final double ROBOT_LENGTH                    = 17.5; //Robot length in inches
    static final double ROBOT_WIDTH                     = 17.5; //Robot width in inches
    static final double PHONE_FRONT_OFFSET              = 6.0;  //Phone offset from front of robot in inches
    static final double PHONE_HEIGHT_OFFSET             = 5.375;//Phone offset from the floor in inches
    static final double PHONE_LEFT_OFFSET               = 11.75;//Phone offset from the left side of the robot in inches

    // TODO: Need to retune.
    static final double HOMOGRAPHY_CAMERA_TOPLEFT_X     = 0.0;
    static final double HOMOGRAPHY_CAMERA_TOPLEFT_Y     = 360.0;
    static final double HOMOGRAPHY_CAMERA_TOPRIGHT_X    = 1280.0;
    static final double HOMOGRAPHY_CAMERA_TOPRIGHT_Y    = 360.0;
    static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_X  = 0.0;
    static final double HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y  = 720.0;
    static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X = 1280.0;
    static final double HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y = 720.0;

    // These should be in real-world robot coordinates. Needs calibration after camera is actually mounted in position.
    // Measurement unit: inches
    // TODO: Tune all of this
    static final double HOMOGRAPHY_WORLD_TOPLEFT_X      = -61.0;
    static final double HOMOGRAPHY_WORLD_TOPLEFT_Y      = 83.0;
    static final double HOMOGRAPHY_WORLD_TOPRIGHT_X     = 33.0;
    static final double HOMOGRAPHY_WORLD_TOPRIGHT_Y     = 83.0;
    static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_X   = -39.5;
    static final double HOMOGRAPHY_WORLD_BOTTOMLEFT_Y   = 19.0;
    static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_X  = 12.0;
    static final double HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y  = 19.0;

    //
    // Elevator subsystem.
    //
    // 2019-10-28: Elevator Scale = 18.622/17897.0 = 0.0010405095826116, Kp = 1.0, Ki = 0.0, Kd = 0.0
    static final double ELEVATOR_KP                     = 1.0;
    static final double ELEVATOR_KI                     = 0.0;
    static final double ELEVATOR_KD                     = 0.0;
    static final double ELEVATOR_TOLERANCE              = 0.5;
    static final double ELEVATOR_CAL_POWER              = 0.5;
    static final double ELEVATOR_MIN_HEIGHT             = 0.0;
    static final double ELEVATOR_MAX_HEIGHT             = 18.5;
    static final double ELEVATOR_SCALE                  = (18.622 / 17897.0);
    static final double ELEVATOR_OFFSET                 = 0.0;

    // ExtenderArm subsystem.
    static final double EXTENDER_ARM_MAX_STEPRATE       = (1.0/3.0);
    static final double EXTENDER_ARM_LOGICAL_MIN_POS    = 0.2;
    static final double EXTENDER_ARM_LOGICAL_MAX_POS    = 0.8;
    static final double EXTENDER_ARM_DOWN_POS           = 0.65;
    static final double EXTENDER_ARM_RETRACTED_POS      = EXTENDER_ARM_LOGICAL_MIN_POS;
    static final double EXTENDER_ARM_PLACEMENT_POS      = 0.6;
    static final double EXTENDER_ARM_MOVE_TIME          = 3.0;

    // Wrist subsystem.
    static final double WRIST_MAX_STEPRATE              = (1.0/0.75);
    static final double WRIST_MIN_POS                   = 0.2;
    static final double WRIST_MAX_POS                   = 0.8;
    static final double WRIST_RETRACT_POS               = 0.3;
    static final double WRIST_EXTEND_POS                = 0.7;
    static final boolean WRIST_INVERTED                 = true;

    // Grabber subsystem.
    static final double GRABBER_OPEN_POS                = 0.8;
    static final double GRABBER_CLOSE_POS               = 0.3;
    static final double GRABBER_GRAB_TIME               = 3.0;
    static final double GRABBER_RELEASE_TIME            = 3.0;

    // FoundationLatch subsystem.
    static final double FOUNDATION_LATCH_CLOSE_POS      = (100.0/255.0);
    static final double FOUNDATION_LATCH_CLOSE_TIME     = 0.5;
    static final double FOUNDATION_LATCH_OPEN_POS       = 0.0;
    static final double FOUNDATION_LATCH_OPEN_TIME      = 0.5;

}   //class RobotInfo3543
