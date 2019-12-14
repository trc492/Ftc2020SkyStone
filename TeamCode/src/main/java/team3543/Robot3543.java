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

class Robot3543 extends Robot
{
    private static final Preferences preferences3543 = new Preferences()
            .setTeam3543(true)
            .setHasRobot(true)
            .setInitSubsystems(true)
            .setHasElevator(true)
            .setHasBlinkin(true)
            .setPlaySongs(false)
            .setSongsInResources(true)
            .setUseVuforia(true)
            .setUseTensorFlow(false)
            .setShowVuforiaView(true)
            .setShowTensorFlowView(false)
            .setUseFlashLight(true, true)
            .setUseVisionTrigger(false)
            .setUseTraceLog(true)
            .setUseSpeech(true)
            .setUseBatteryMonitor(false)
            .setUseLoopPerformanceMonitor(false)
            .setUseVelocityControl(true);
    private static final PhoneParameters phoneParams3543 = new PhoneParameters()
            .setCameraDir(RobotInfo3543.CAMERA_DIR)
            .setCameraMonitorFeedback(RobotInfo3543.CAMERA_MONITOR_FEEDBACK)
            .setPhoneIsPortrait(RobotInfo3543.PHONE_IS_PORTRAIT)
            .setPhoneOffsets(
                    RobotInfo3543.PHONE_FRONT_OFFSET, RobotInfo3543.PHONE_LEFT_OFFSET,
                    RobotInfo3543.PHONE_HEIGHT_OFFSET);
    private static final FtcMotorActuator.Parameters elevatorParams3543 = new FtcMotorActuator.Parameters()
            .setPosRange(RobotInfo3543.ELEVATOR_MIN_HEIGHT, RobotInfo3543.ELEVATOR_MAX_HEIGHT)
            .setScaleOffset(RobotInfo3543.ELEVATOR_SCALE, RobotInfo3543.ELEVATOR_OFFSET)
            .setPidParams(
                    RobotInfo3543.ELEVATOR_KP, RobotInfo3543.ELEVATOR_KI, RobotInfo3543.ELEVATOR_KD,
                    RobotInfo3543.ELEVATOR_TOLERANCE)
            .setMotorParams(
                    RobotInfo3543.ELEVATOR_INVERTED, RobotInfo3543.ELEVATOR_HAS_LOWER_LIMIT_SWITCH,
                    RobotInfo3543.ELEVATOR_HAS_UPPER_LIMIT_SWITCH, RobotInfo3543.ELEVATOR_CAL_POWER)
            .setStallProtectionParams(
                    RobotInfo3543.ELEVATOR_STALL_MIN_POWER, RobotInfo3543.ELEVATOR_STALL_TIMEOUT,
                    RobotInfo3543.ELEVATOR_RESET_TIMEOUT);
    private static final FtcMotorActuator.Parameters extenderArmParams3543 = new FtcMotorActuator.Parameters()
            .setPosRange(RobotInfo3543.EXTENDER_ARM_MIN_POS, RobotInfo3543.EXTENDER_ARM_MAX_POS)
            .setScaleOffset(RobotInfo3543.EXTENDER_ARM_SCALE, RobotInfo3543.EXTENDER_ARM_OFFSET)
            .setPidParams(
                    RobotInfo3543.EXTENDER_ARM_KP, RobotInfo3543.EXTENDER_ARM_KI, RobotInfo3543.EXTENDER_ARM_KD,
                    RobotInfo3543.EXTENDER_ARM_TOLERANCE)
            .setMotorParams(
                    RobotInfo3543.EXTENDER_ARM_INVERTED, RobotInfo3543.EXTENDER_ARM_HAS_LOWER_LIMIT_SWITCH,
                    RobotInfo3543.EXTENDER_ARM_HAS_UPPER_LIMIT_SWITCH, RobotInfo3543.EXTENDER_ARM_CAL_POWER)
            .setStallProtectionParams(
                RobotInfo3543.EXTENDER_ARM_STALL_MIN_POWER, RobotInfo3543.EXTENDER_ARM_STALL_TIMEOUT,
                RobotInfo3543.EXTENDER_ARM_RESET_TIMEOUT);
    private static final FtcServoActuator.Parameters wristParams3543 = new FtcServoActuator.Parameters()
            .setStepParams(RobotInfo3543.WRIST_MAX_STEPRATE, RobotInfo3543.WRIST_MIN_POS, RobotInfo3543.WRIST_MAX_POS)
            .setInverted(false, false)
            .setRetractParams(RobotInfo3543.WRIST_RETRACT_POS, RobotInfo3543.WRIST_ROTATE_TIME)
            .setExtendParams(RobotInfo3543.WRIST_EXTEND_POS, RobotInfo3543.WRIST_ROTATE_TIME);
    private static final FtcServoActuator.Parameters grabberParams3543 = new FtcServoActuator.Parameters()
            .setStepParams(
                    RobotInfo3543.GRABBER_MAX_STEPRATE, RobotInfo3543.GRABBER_MIN_POS, RobotInfo3543.GRABBER_MAX_POS)
            .setInverted(false, false)
            .setRetractParams(RobotInfo3543.GRABBER_CLOSE_POS, RobotInfo3543.GRABBER_GRAB_TIME)
            .setExtendParams(RobotInfo3543.GRABBER_OPEN_POS, RobotInfo3543.GRABBER_RELEASE_TIME);
    private static final FtcServoActuator.Parameters backFoundationLatchParams3543 = new FtcServoActuator.Parameters()
            .setStepParams(
                    RobotInfo3543.BACK_FOUNDATION_LATCH_MAX_STEPRATE, RobotInfo3543.BACK_FOUNDATION_LATCH_MIN_POS,
                    RobotInfo3543.BACK_FOUNDATION_LATCH_MAX_POS)
            .setInverted(false, false)
            .setRetractParams(
                    RobotInfo3543.BACK_FOUNDATION_LATCH_GRAB_POS, RobotInfo3543.BACK_FOUNDATION_LATCH_GRAB_TIME)
            .setExtendParams(
                    RobotInfo3543.BACK_FOUNDATION_LATCH_RELEASE_POS, RobotInfo3543.BACK_FOUNDATION_LATCH_RELEASE_TIME);
    private static final FtcServoActuator.Parameters frontFoundationLatchParams3543 = new FtcServoActuator.Parameters()
            .setStepParams(
                    RobotInfo3543.FRONT_FOUNDATION_LATCH_MAX_STEPRATE, RobotInfo3543.FRONT_FOUNDATION_LATCH_MIN_POS,
                    RobotInfo3543.FRONT_FOUNDATION_LATCH_MAX_POS)
            .setInverted(true, false)
            .setRetractParams(
                    RobotInfo3543.FRONT_FOUNDATION_LATCH_GRAB_POS, RobotInfo3543.FRONT_FOUNDATION_LATCH_GRAB_TIME)
            .setExtendParams(
                    RobotInfo3543.FRONT_FOUNDATION_LATCH_RELEASE_POS,
                    RobotInfo3543.FRONT_FOUNDATION_LATCH_RELEASE_TIME);
    //
    // Team specific subsystems.
    //
    FtcMotorActuator extenderArm = null;
    FtcServoActuator wrist = null;
    Grabber frontFoundationLatch = null;

    Robot3543(TrcRobot.RunMode runMode)
    {
        super(runMode, RobotInfo3543.ROBOT_NAME, preferences3543, phoneParams3543);
        //
        // Initialize vision subsystems.
        //
        if (preferences.useVuforia && vuforia != null &&
            (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            initVuforia(phoneParams3543, RobotInfo3543.ROBOT_LENGTH, RobotInfo3543.ROBOT_WIDTH);
        }

        if (preferences.useTensorFlow && vuforia != null &&
            (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            TrcHomographyMapper.Rectangle cameraRect = new TrcHomographyMapper.Rectangle(
                    RobotInfo3543.HOMOGRAPHY_CAMERA_TOPLEFT_X, RobotInfo3543.HOMOGRAPHY_CAMERA_TOPLEFT_Y,
                    RobotInfo3543.HOMOGRAPHY_CAMERA_TOPRIGHT_X, RobotInfo3543.HOMOGRAPHY_CAMERA_TOPRIGHT_Y,
                    RobotInfo3543.HOMOGRAPHY_CAMERA_BOTTOMLEFT_X, RobotInfo3543.HOMOGRAPHY_CAMERA_BOTTOMLEFT_Y,
                    RobotInfo3543.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_X, RobotInfo3543.HOMOGRAPHY_CAMERA_BOTTOMRIGHT_Y);

            TrcHomographyMapper.Rectangle worldRect = new TrcHomographyMapper.Rectangle(
                    RobotInfo3543.HOMOGRAPHY_WORLD_TOPLEFT_X, RobotInfo3543.HOMOGRAPHY_WORLD_TOPLEFT_Y,
                    RobotInfo3543.HOMOGRAPHY_WORLD_TOPRIGHT_X, RobotInfo3543.HOMOGRAPHY_WORLD_TOPRIGHT_Y,
                    RobotInfo3543.HOMOGRAPHY_WORLD_BOTTOMLEFT_X, RobotInfo3543.HOMOGRAPHY_WORLD_BOTTOMLEFT_Y,
                    RobotInfo3543.HOMOGRAPHY_WORLD_BOTTOMRIGHT_X, RobotInfo3543.HOMOGRAPHY_WORLD_BOTTOMRIGHT_Y);

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
                    elevator = new FtcMotorActuator("elevator", elevatorParams3543);
                    elevator.setBeep(androidTone);
                    elevator.zeroCalibrate();
                }
                // ExtenderArm is 3543 only.
                extenderArm = new FtcMotorActuator("extenderArm", extenderArmParams3543);
                extenderArm.setBeep(androidTone);
                extenderArm.zeroCalibrate();
                // Wrist is 3543 only.
                wrist = new FtcServoActuator("wristServo", wristParams3543);
                wrist.retract();

                grabber = new Grabber("grabberServo", grabberParams3543);
                grabber.release();

                backFoundationLatch = new Grabber("backFoundationLatchServo", backFoundationLatchParams3543);
                backFoundationLatch.release();
                // FrontFoundationLatch is 3543 only
                frontFoundationLatch = new Grabber(
                        "leftFoundationLatchServo", "rightFoundationLatchServo",
                        frontFoundationLatchParams3543);
                frontFoundationLatch.release();
            }
        }
        //
        // Tell the driver initialization is complete.
        //
        speak("Init complete!");
    }   //Robot3543

    private void initDriveBase()
    {
        leftFrontWheel = new FtcDcMotor("lfWheel");
        rightFrontWheel = new FtcDcMotor("rfWheel");
        leftRearWheel = new FtcDcMotor("lrWheel");
        rightRearWheel = new FtcDcMotor("rrWheel");

        leftFrontWheel.motor.setMode(RobotInfo3543.DRIVE_MOTOR_MODE);
        rightFrontWheel.motor.setMode(RobotInfo3543.DRIVE_MOTOR_MODE);
        leftRearWheel.motor.setMode(RobotInfo3543.DRIVE_MOTOR_MODE);
        rightRearWheel.motor.setMode(RobotInfo3543.DRIVE_MOTOR_MODE);

        leftFrontWheel.setOdometryEnabled(true);
        rightFrontWheel.setOdometryEnabled(true);
        leftRearWheel.setOdometryEnabled(true);
        rightRearWheel.setOdometryEnabled(true);

        if (preferences.useVelocityControl)
        {
//            TrcPidController.PidCoefficients motorPidCoef = new TrcPidController.PidCoefficients(
//                    RobotInfo3543.MOTOR_KP, RobotInfo3543.MOTOR_KI, RobotInfo3543.MOTOR_KD);
//            leftFrontWheel.enableVelocityMode(RobotInfo3543.MOTOR_MAX_VELOCITY, motorPidCoef);
//            rightFrontWheel.enableVelocityMode(RobotInfo3543.MOTOR_MAX_VELOCITY, motorPidCoef);
//            leftRearWheel.enableVelocityMode(RobotInfo3543.MOTOR_MAX_VELOCITY, motorPidCoef);
//            rightRearWheel.enableVelocityMode(RobotInfo3543.MOTOR_MAX_VELOCITY, motorPidCoef);
            leftFrontWheel.enableVelocityMode(RobotInfo3543.MOTOR_MAX_VELOCITY, null);
            rightFrontWheel.enableVelocityMode(RobotInfo3543.MOTOR_MAX_VELOCITY, null);
            leftRearWheel.enableVelocityMode(RobotInfo3543.MOTOR_MAX_VELOCITY, null);
            rightRearWheel.enableVelocityMode(RobotInfo3543.MOTOR_MAX_VELOCITY, null);
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
        driveBase.setPositionScales(RobotInfo3543.ENCODER_X_INCHES_PER_COUNT, RobotInfo3543.ENCODER_Y_INCHES_PER_COUNT);
        driveMode = DriveMode.HOLONOMIC_MODE;
        //
        // Initialize PID drive.
        //
        encoderXPidCtrl = new TrcPidController(
                "encoderXPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo3543.ENCODER_X_KP, RobotInfo3543.ENCODER_X_KI, RobotInfo3543.ENCODER_X_KD),
                RobotInfo3543.ENCODER_X_TOLERANCE, driveBase::getXPosition);
        encoderYPidCtrl = new TrcPidController(
                "encoderYPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo3543.ENCODER_Y_KP, RobotInfo3543.ENCODER_Y_KI, RobotInfo3543.ENCODER_Y_KD),
                RobotInfo3543.ENCODER_Y_TOLERANCE, driveBase::getYPosition);
        gyroPidCtrl = new TrcPidController(
                "gyroPidCtrl",
                new TrcPidController.PidCoefficients(
                        RobotInfo3543.GYRO_KP, RobotInfo3543.GYRO_KI, RobotInfo3543.GYRO_KD),
                RobotInfo3543.GYRO_TOLERANCE, driveBase::getHeading);
        gyroPidCtrl.setAbsoluteSetPoint(true);
//        gyroPidCtrl.setOutputRange(-RobotInfo3543.TURN_POWER_LIMIT, RobotInfo3543.TURN_POWER_LIMIT);

        pidDrive = new TrcPidDrive(
                "pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setAbsoluteTargetModeEnabled(true);
        pidDrive.setStallTimeout(RobotInfo3543.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);
        pidDrive.setMsgTracer(globalTracer);
    }   //initDriveBase

}   //class Robot3543
