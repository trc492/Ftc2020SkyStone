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

import common.Grabber;
import common.Robot;
import ftclib.FtcServoActuator;
import ftclib.FtcDcMotor;
import ftclib.FtcMotorActuator;
import trclib.TrcHomographyMapper;
import trclib.TrcMecanumDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;

class Robot6541 extends Robot
{
    private static final Preferences preferences6541 = new Preferences()
            .setTeam3543(false)
            .setHasRobot(true)
            .setInitSubsystems(true)
            .setHasElevator(true)
            .setHasBlinkin(true)
            .setUseVuforia(true)
            .setUseTensorFlow(false)
            .setShowVuforiaView(false)
            .setShowTensorFlowView(false)
            .setUseFlashLight(true)
            .setUseVisionTrigger(false)
            .setUseTraceLog(true)
            .setUseSpeech(true)
            .setUseBatteryMonitor(false)
            .setUseLoopPerformanceMonitor(false)
            .setUseVelocityControl(false);
    private static final PhoneParameters phoneParams6541 = new PhoneParameters()
            .setCameraDir(RobotInfo6541.CAMERA_DIR)
            .setCameraMonitorFeedback(RobotInfo6541.CAMERA_MONITOR_FEEDBACK)
            .setPhoneIsPortrait(RobotInfo6541.PHONE_IS_PORTRAIT)
            .setPhoneOffsets(
                    RobotInfo6541.PHONE_FRONT_OFFSET, RobotInfo6541.PHONE_LEFT_OFFSET,
                    RobotInfo6541.PHONE_HEIGHT_OFFSET);
    private static final FtcMotorActuator.Parameters elevatorParams6541 = new FtcMotorActuator.Parameters()
            .setPosRange(RobotInfo6541.ELEVATOR_MIN_HEIGHT, RobotInfo6541.ELEVATOR_MAX_HEIGHT)
            .setScaleOffset(RobotInfo6541.ELEVATOR_SCALE, RobotInfo6541.ELEVATOR_OFFSET)
            .setPidParams(
                    RobotInfo6541.ELEVATOR_KP, RobotInfo6541.ELEVATOR_KI, RobotInfo6541.ELEVATOR_KD,
                    RobotInfo6541.ELEVATOR_TOLERANCE)
            .setMotorParams(
                    RobotInfo6541.ELEVATOR_INVERTED, RobotInfo6541.ELEVATOR_HAS_UPPER_LIMIT_SWITCH,
                    RobotInfo6541.ELEVATOR_CAL_POWER)
            .setPosPresets(RobotInfo6541.ELEVATOR_HEIGHT_PRESETS);
    private static final FtcServoActuator.Parameters elbowParams6541 = new FtcServoActuator.Parameters()
            .setStepParams(RobotInfo6541.ELBOW_MAX_STEPRATE, RobotInfo6541.ELBOW_MIN_POS, RobotInfo6541.ELBOW_MAX_POS)
            .setRetractParams(RobotInfo6541.ELBOW_RETRACT_POS, RobotInfo6541.ELBOW_RETRACT_TIME)
            .setExtendParams(RobotInfo6541.ELBOW_EXTEND_POS, RobotInfo6541.ELBOW_EXTEND_TIME);
    private static final FtcServoActuator.Parameters grabberParams6541 = new FtcServoActuator.Parameters()
            .setStepParams(
                    RobotInfo6541.GRABBER_MAX_STEPRATE, RobotInfo6541.GRABBER_MIN_POS, RobotInfo6541.GRABBER_MAX_POS)
            .setExtendParams(RobotInfo6541.GRABBER_RELEASE_POS, RobotInfo6541.GRABBER_RELEASE_TIME)
            .setRetractParams(RobotInfo6541.GRABBER_GRAB_POS, RobotInfo6541.GRABBER_GRAB_TIME);
    private static final FtcServoActuator.Parameters backFoundationLatchParams6541 = new FtcServoActuator.Parameters()
            .setStepParams(
                    RobotInfo6541.BACK_FOUNDATION_LATCH_MAX_STEPRATE, RobotInfo6541.BACK_FOUNDATION_LATCH_MIN_POS,
                    RobotInfo6541.BACK_FOUNDATION_LATCH_MAX_POS)
            .setRetractParams(
                    RobotInfo6541.BACK_FOUNDATION_LATCH_RELEASE_POS, RobotInfo6541.BACK_FOUNDATION_LATCH_RELEASE_TIME)
            .setExtendParams(
                    RobotInfo6541.BACK_FOUNDATION_LATCH_GRAB_POS, RobotInfo6541.BACK_FOUNDATION_LATCH_GRAB_TIME);
    //
    // Team specific subsystems.
    //
    FtcServoActuator elbow = null;

    Robot6541(TrcRobot.RunMode runMode)
    {
        super(runMode, RobotInfo6541.ROBOT_NAME, preferences6541, phoneParams6541);
        //
        // Initialize vision subsystems.
        //
        if (preferences.useVuforia && vuforia != null &&
            (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            initVuforia(phoneParams6541, RobotInfo6541.ROBOT_LENGTH, RobotInfo6541.ROBOT_WIDTH);
        }

        if (preferences.useTensorFlow && vuforia != null &&
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

            initTensorFlow(preferences.showTensorFlowView, cameraRect, worldRect);
        }

        if (preferences.hasRobot)
        {
            //
            // Initialize DriveBase.
            //
            initDriveBase();
            //
            // Initialize other subsystems.
            //
            if (preferences.initSubsystems)
            {
                if (preferences.hasElevator)
                {
                    elevator = new FtcMotorActuator("elevator", elevatorParams6541);
                    elevator.zeroCalibrate();
                }
                // Elbow is 6541 only.
                elbow = new FtcServoActuator("elbowServo", elbowParams6541);
                elbow.retract();

                grabber = new Grabber("grabberServo", grabberParams6541);
                grabber.grab();

                backFoundationLatch = new Grabber("backFoundationLatchServo", backFoundationLatchParams6541);
                backFoundationLatch.release();
            }
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

        if (preferences.useVelocityControl)
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

        pidDrive = new TrcPidDrive(
                "pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
        //CodeReview: you should really turn on Absolute Target Mode and adjust your autonomous to work in this mode.
        pidDrive.setAbsoluteTargetModeEnabled(false);
        pidDrive.setStallTimeout(RobotInfo6541.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);
    }   //initDriveBase

}   //class Robot6541
