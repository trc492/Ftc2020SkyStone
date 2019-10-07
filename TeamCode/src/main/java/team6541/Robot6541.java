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

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import common.Robot;
import ftclib.FtcDcMotor;
import trclib.TrcHomographyMapper;
import trclib.TrcMecanumDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Robot6541 extends Robot
{
    //
    // Feature switches.
    //
    private static final boolean USE_SPEECH = true;
    private static final boolean USE_BATTERY_MONITOR = false;
    private static final boolean USE_VUFORIA = false;
    private static final boolean USE_TENSORFLOW = true;
    private static final boolean SHOW_VUFORIA_VIEW = false;
    private static final boolean SHOW_TENSORFLOW_VIEW = true;
    private static final boolean USE_VELOCITY_CONTROL = false;
    private static final boolean HAS_ROBOT = false;

    private static final VuforiaLocalizer.CameraDirection CAMERA_DIR = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final double ROBOT_LENGTH = 17.5;        //Robot length in inches
    private static final double ROBOT_WIDTH = 17.5;         //Robot width in inches
    private static final double PHONE_FRONT_OFFSET = 0.75;  //Phone offset from front of robot in inches
    private static final double PHONE_HEIGHT_OFFSET = 6.25; //Phone offset from the floor in inches
    private static final double PHONE_LEFT_OFFSET = 8.75;   //Phone offset from the left side of the robot in inches
    //
    // Global objects.
    //
    public static final String robotName = "Robot6541";

    public Robot6541(TrcRobot.RunMode runMode)
    {
        super(runMode, robotName, CAMERA_DIR,
              SHOW_VUFORIA_VIEW, USE_VUFORIA || USE_TENSORFLOW, USE_SPEECH, USE_BATTERY_MONITOR, HAS_ROBOT);
        //
        // Initialize vision subsystems.
        //
        if (USE_VUFORIA && vuforia != null)
        {
            initVuforia(
                    CAMERA_DIR, PHONE_IS_PORTRAIT, ROBOT_LENGTH, ROBOT_WIDTH, PHONE_FRONT_OFFSET, PHONE_LEFT_OFFSET,
                    PHONE_HEIGHT_OFFSET);
        }
        //
        // TensorFlow slows down our threads really badly, so don't enable it if we don't need it.
        //
        if (USE_TENSORFLOW && vuforia != null &&
            (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
                    RobotInfo6541.HOMOGRAPHY_CAMERA_TOPLEFT_X, RobotInfo6541.HOMOGRAPHY_CAMERA_TOPLEFT_Y,
                    RobotInfo6541.HOMOGRAPHY_CAMERA_TOPRIGHT_X, RobotInfo6541.HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
                    RobotInfo6541.HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, RobotInfo6541.HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
                    RobotInfo6541.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, RobotInfo6541.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);

            TrcHomographyMapper.Rectangle worldRect = new TrcHomographyMapper.Rectangle(
                    RobotInfo6541.HOMOGRAPHY_WORLD_TOPLEFT_X, RobotInfo6541.HOMOGRAPHY_WORLD_TOPLEFT_Y,
                    RobotInfo6541.HOMOGRAPHY_WORLD_TOPRIGHT_X, RobotInfo6541.HOMOGRAPHY_WORLD_TOPRIGHT_Y,
                    RobotInfo6541.HOMOGRAPHY_WORLD_BOTTOMLEFT_X, RobotInfo6541.HOMOGRAPHY_WORLD_BOTTOMLEFT_Y,
                    RobotInfo6541.HOMOGRAPHY_WORLD_BOTTOMRIGHT_X, RobotInfo6541.HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y);

            initTensorFlow(SHOW_TENSORFLOW_VIEW, cameraRect, worldRect);
        }

        if (HAS_ROBOT)
        {
            //
            // Initialize DriveBase.
            //
            initDriveBase();
            //
            // Initialize other subsystems.
            //
        }
        //
        // Tell the driver initialization is complete.
        //
        speak("Init complete!");
    }   //Robot6541

    private void initDriveBase()
    {
        leftFrontWheel = new FtcDcMotor("lfWheel");
        rightFrontWheel = new FtcDcMotor("rfWheel");
        leftRearWheel = new FtcDcMotor("lrWheel");
        rightRearWheel = new FtcDcMotor("rrWheel");

        leftFrontWheel.motor.setMode(RobotInfo6541.DRIVE_MOTOR_MODE);
        rightFrontWheel.motor.setMode(RobotInfo6541.DRIVE_MOTOR_MODE);
        leftRearWheel.motor.setMode(RobotInfo6541.DRIVE_MOTOR_MODE);
        rightRearWheel.motor.setMode(RobotInfo6541.DRIVE_MOTOR_MODE);

        leftFrontWheel.setOdometryEnabled(true);
        rightFrontWheel.setOdometryEnabled(true);
        leftRearWheel.setOdometryEnabled(true);
        rightRearWheel.setOdometryEnabled(true);

        if (USE_VELOCITY_CONTROL)
        {
            TrcPidController.PidCoefficients motorPidCoef = new TrcPidController.PidCoefficients(
                    RobotInfo6541.MOTOR_KP, RobotInfo6541.MOTOR_KI, RobotInfo6541.MOTOR_KD);
            leftFrontWheel.enableVelocityMode(RobotInfo6541.MOTOR_MAX_VELOCITY, motorPidCoef);
            rightFrontWheel.enableVelocityMode(RobotInfo6541.MOTOR_MAX_VELOCITY, motorPidCoef);
            leftRearWheel.enableVelocityMode(RobotInfo6541.MOTOR_MAX_VELOCITY, motorPidCoef);
            rightRearWheel.enableVelocityMode(RobotInfo6541.MOTOR_MAX_VELOCITY, motorPidCoef);
        }

        leftFrontWheel.setInverted(false);
        leftRearWheel.setInverted(false);
        rightFrontWheel.setInverted(true);
        rightRearWheel.setInverted(true);

        leftFrontWheel.setBrakeModeEnabled(true);
        leftRearWheel.setBrakeModeEnabled(true);
        rightFrontWheel.setBrakeModeEnabled(true);
        rightRearWheel.setBrakeModeEnabled(true);

        driveBase = new TrcMecanumDriveBase(leftFrontWheel, leftRearWheel, rightFrontWheel, rightRearWheel, gyro);
        driveBase.setPositionScales(RobotInfo6541.ENCODER_X_INCHES_PER_COUNT, RobotInfo6541.ENCODER_Y_INCHES_PER_COUNT);
        driveMode = DriveMode.HOLONOMIC_MODE;
        //
        // Initialize PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
                "encoderXPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo6541.ENCODER_X_KP, RobotInfo6541.ENCODER_X_KI, RobotInfo6541.ENCODER_X_KD),
                RobotInfo6541.ENCODER_X_TOLERANCE, driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo6541.ENCODER_Y_KP, RobotInfo6541.ENCODER_Y_KI, RobotInfo6541.ENCODER_Y_KD),
                RobotInfo6541.ENCODER_Y_TOLERANCE, driveBase::getYPosition);
        gyroPidCtrl = new TrcPidController(
                "gyroPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo6541.GYRO_KP, RobotInfo6541.GYRO_KI, RobotInfo6541.GYRO_KD),
                RobotInfo6541.GYRO_TOLERANCE, driveBase::getHeading);
        gyroPidCtrl.setAbsoluteSetPoint(true);
        gyroPidCtrl.setOutputRange(-RobotInfo6541.TURN_POWER_LIMIT, RobotInfo6541.TURN_POWER_LIMIT);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setStallTimeout(RobotInfo6541.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);
        pidDrive.setMsgTracer(globalTracer, true);
    }   //initDriveBase

}   //class Robot6541
