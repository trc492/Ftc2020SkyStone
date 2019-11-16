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

import common.Elevator;
import common.FoundationLatch;
import common.Grabber;
import common.Robot;
import common.ServoEndEffector;
import common.Wrist;
import ftclib.FtcDcMotor;
import trclib.TrcHashMap;
import trclib.TrcHomographyMapper;
import trclib.TrcMecanumDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;

class Robot6541 extends Robot
{
    private static TrcHashMap<String, Boolean> preferences6541 = new TrcHashMap<String, Boolean>()
            .add("hasRobot", true)
            .add("hasElevator", true)
            .add("useTraceLog", true)
            .add("useSpeech", true)
            .add("useBatteryMonitor", false)
            .add("useLoopPerformanceMonitor", true)
            .add("useBlinkin", true)
            .add("useVelocityControl", false)
            .add("useVuforia", false)
            .add("useTensorFlow", false)
            .add("useFlashLight", false)
            .add("showVuforiaView", false)
            .add("showTensorFlowView", false)
            .add("initSubsystems", true)
            .add("useVisionTrigger", false)
            .add("team3543", false);
    private static TrcHashMap<String, Object> phoneParams6541 = new TrcHashMap<String, Object>()
            .add("cameraDir", RobotInfo6541.CAMERA_DIR)
            .add("cameraMonitorFeedback", RobotInfo6541.CAMERA_MONITOR_FEEDBACK)
            .add("phoneIsPortrait", RobotInfo6541.PHONE_IS_PORTRAIT)
            .add("phoneFrontOffset", RobotInfo6541.PHONE_FRONT_OFFSET)
            .add("phoneLeftOffset", RobotInfo6541.PHONE_LEFT_OFFSET)
            .add("phoneHeightOffset", RobotInfo6541.PHONE_HEIGHT_OFFSET);
    private static TrcHashMap<String, Double> elevatorParams6541 = new TrcHashMap<String, Double>()
            .add("minHeight", RobotInfo6541.ELEVATOR_MIN_HEIGHT)
            .add("maxHeight", RobotInfo6541.ELEVATOR_MAX_HEIGHT)
            .add("scale", RobotInfo6541.ELEVATOR_SCALE)
            .add("offset", RobotInfo6541.ELEVATOR_OFFSET)
            .add("Kp", RobotInfo6541.ELEVATOR_KP)
            .add("Ki", RobotInfo6541.ELEVATOR_KI)
            .add("Kd", RobotInfo6541.ELEVATOR_KD)
            .add("tolerance", RobotInfo6541.ELEVATOR_TOLERANCE)
            .add("calPower", RobotInfo6541.ELEVATOR_CAL_POWER);
    private static TrcHashMap<String, Object> wristParams6541 = new TrcHashMap<String, Object>()
            .add("maxStepRate", RobotInfo6541.WRIST_MAX_STEPRATE)
            .add("minPos", RobotInfo6541.WRIST_MIN_POS)
            .add("maxPos", RobotInfo6541.WRIST_MAX_POS)
            .add("retractPos", RobotInfo6541.WRIST_RETRACT_POS)
            .add("extendPos", RobotInfo6541.WRIST_EXTEND_POS)
            .add("inverted", RobotInfo6541.WRIST_INVERTED);
    private static TrcHashMap<String, Double> foundationLatchParams6541 = new TrcHashMap<String, Double>()
            .add("closePos", RobotInfo6541.FOUNDATION_LATCH_CLOSE_POS)
            .add("closeTime", RobotInfo6541.FOUNDATION_LATCH_CLOSE_TIME)
            .add("openPos", RobotInfo6541.FOUNDATION_LATCH_OPEN_POS)
            .add("openTime", RobotInfo6541.FOUNDATION_LATCH_OPEN_TIME);
    private static ServoEndEffector.Parameters grabberParams6541 = new ServoEndEffector.Parameters()
            .setExtendPos(RobotInfo6541.GRABBER_RELEASE_POS)
            .setExtendTime(RobotInfo6541.GRABBER_RELEASE_TIME)
            .setRetractPos(RobotInfo6541.GRABBER_GRAB_POS)
            .setRetractTime(RobotInfo6541.GRABBER_GRAB_TIME);

    Robot6541(TrcRobot.RunMode runMode)
    {
        super(runMode, RobotInfo6541.ROBOT_NAME, preferences6541, phoneParams6541);
        //
        // Initialize vision subsystems.
        //
        if (preferences.getBoolean("useVuforia") && vuforia != null &&
            (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            initVuforia(phoneParams6541, RobotInfo6541.ROBOT_LENGTH, RobotInfo6541.ROBOT_WIDTH);
        }

        if (preferences.getBoolean("useTensorFlow") && vuforia != null &&
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

            initTensorFlow(preferences.getBoolean("showTensorFlowView"), cameraRect, worldRect);
        }

        if (preferences.getBoolean("hasRobot"))
        {
            //
            // Initialize DriveBase.
            //
            initDriveBase();
            //
            // Initialize other subsystems.
            //
            if (preferences.getBoolean("initSubsystems"))
            {
                if (preferences.getBoolean("hasElevator"))
                {
                    elevator = new Elevator(preferences6541, elevatorParams6541, RobotInfo6541.ELEVATOR_HEIGHT_PRESETS);
                    elevator.zeroCalibrate();
                }

                wrist = new Wrist(wristParams6541);
                wrist.setPosition(RobotInfo6541.WRIST_MIN_POS);

                grabber = new Grabber("grabberServo", grabberParams6541);

                foundationLatch = new FoundationLatch(foundationLatchParams6541);
                foundationLatch.release();
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

        if (preferences.getBoolean("useVelocityControl"))
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
        pidDrive.setAbsTargetModeEnabled(false);
        pidDrive.setStallTimeout(RobotInfo6541.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);
    }   //initDriveBase

}   //class Robot6541
