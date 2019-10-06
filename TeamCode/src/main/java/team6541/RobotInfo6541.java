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

package team6541;

import com.qualcomm.robotcore.hardware.DcMotor;

class RobotInfo6541
{
    //
    // DriveBase subsystem.
    //
    static final DcMotor.RunMode DRIVE_MOTOR_MODE       = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    static final double TURN_POWER_LIMIT                = 0.5;
    //
    // Velocity controlled constants.
    //
    static final double MOTOR_MAX_VELOCITY              = 3080.0;   //encoder counts per second
    static final double MOTOR_KP                        = 0.75;
    static final double MOTOR_KI                        = 0.0;
    static final double MOTOR_KD                        = 0.0;
    //
    // 2018-10-27: Kp=0.1, Ki=0.0, Kd=0.0, Scale=0.0177558441951763
    //
    // 31 inches
    static final double ENCODER_X_KP                    = 0.1;
    static final double ENCODER_X_KI                    = 0.0;
    static final double ENCODER_X_KD                    = 0.0;
    static final double ENCODER_X_TOLERANCE             = 2.0;
    static final double ENCODER_X_INCHES_PER_COUNT      = 0.0177558441951763 * (26.5 / 36.8) * (24.25 / 25.1) * (37.0 / 36.2);  // (16.0 / 1352.5) * (26.0 / 24.6) * (37.4 / 40.0) * (37.1 / 42.0); //0.0177558441951763; // 1352.5
    //
    // 2018-10-27: Kp=0.035, Ki=0.0, Kd=0.0025, Scale=0.0172934
    // 2018-11-29: Kp=0.05, Ki=0.0, Kd=0.0, Scale=0.0158423538151923
    //
    static final double ENCODER_Y_KP                    = 0.05;
    static final double ENCODER_Y_KI                    = 0.0;
    static final double ENCODER_Y_KD                    = 0.0;
    static final double ENCODER_Y_TOLERANCE             = 1.0;
    static final double ENCODER_Y_INCHES_PER_COUNT      = 0.0158423538151923;
    //
    // 2018-10-27: Kp=0.025, Ki=0.0, Kd=0.0
    // 2018-11-29: Kp=0.025, Ki=0.0, Kd=0.0025
    //
    static final double GYRO_KP                         = 0.025;
    static final double GYRO_KI                         = 0.0;
    static final double GYRO_KD                         = 0.0025;
    static final double GYRO_TOLERANCE                  = 2.0;

    static final double PIDDRIVE_STALL_TIMEOUT          = 0.5;      //in seconds.

    // TODO: need to tune all PID coefficients
    static final double PURE_PURSUIT_POS_KP             = 0.011;
    static final double PURE_PURSUIT_POS_KI             = 0.0;
    static final double PURE_PURSUIT_POS_KD             = 0.001;
    static final double PURE_PURSUIT_TURN_KP            = 0.025;
    static final double PURE_PURSUIT_VEL_KP             = 0.0;
    static final double PURE_PURSUIT_VEL_KI             = 0.0;
    static final double PURE_PURSUIT_VEL_KD             = 0.9;
    static final double PURE_PURSUIT_VEL_KF             = 1.0/223;

    static final double CAMERA_HEIGHT                   = 720.0;    // in pixels.
    static final double CAMERA_WIDTH                    = 1280.0;   // in pixels.

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

}   //class RobotInfo6541
