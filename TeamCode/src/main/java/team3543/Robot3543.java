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

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import common.Robot;
import common.TensorFlowVision;
import common.VuforiaVision;
import ftclib.FtcDcMotor;
import trclib.TrcMecanumDriveBase;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;
import trclib.TrcUtil;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Robot3543 extends Robot
{
    //
    // Feature switches.
    //
    public static final boolean USE_SPEECH = true;
    public static final boolean USE_BATTERY_MONITOR = false;
    public static final boolean USE_VUFORIA = false;
    public static final boolean USE_TENSORFLOW = true;
    public static final boolean USE_VELOCITY_CONTROL = false;
    //
    // Global objects.
    //
    public static final String robotName = "Robot3543";
    //
    // Other subsystems.
    //

    public Robot3543(TrcRobot.RunMode runMode)
    {
        super(runMode, robotName, USE_SPEECH, USE_BATTERY_MONITOR);
        //
        // Initialize vision subsystems.
        //
        if (USE_VUFORIA)
        {
            final int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            final VuforiaLocalizer.CameraDirection CAMERA_DIR = BACK;
            final boolean PHONE_IS_PORTRAIT = false;
            float phoneXRotate;
            float phoneYRotate;
            float phoneZRotate = 0.0f;
            //
            // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation,
            // based on how your phone is mounted:
            // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
            // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or
            //                                    PHONE_IS_PORTRAIT = false (landscape)
            //
            // NOTE: If you are running on a CONTROL HUB, with only one USB WebCam, you must select CAMERA_DIR = BACK;
            // and PHONE_IS_PORTRAIT = false;
            //
            /*
             * Create a transformation matrix describing where the phone is on the robot.
             *
             * The coordinate frame for the robot looks the same as the field.
             * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the
             * Y axis. Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
             *
             * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
             * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top
             * of the camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip
             * the phone.
             *
             * If using the rear (High Res) camera:
             * We need to rotate the camera around it's long axis to bring the rear camera forward.
             * This requires a negative 90 degree rotation on the Y axis
             *
             * If using the Front (Low Res) camera
             * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
             * This requires a Positive 90 degree rotation on the Y axis
             *
             * Next, translate the camera lens to where it is on the robot.
             * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and
             * 200 mm above ground level.
             */
            final double ROBOT_LENGTH = 17.5;           //Robot length in inches
            final double ROBOT_WIDTH = 17.5;            //Robot width in inches
            final double PHONE_FRONT_OFFSET = 0.75;     //Phone offset from front of robot in inches
            final double PHONE_HEIGHT_OFFSET = 6.25;    //Phone offset from the floor in inches
            final double PHONE_LEFT_OFFSET = 8.75;      //Phone offset from the left side of the robot in inches
            final int CAMERA_FORWARD_DISPLACEMENT = (int)((ROBOT_LENGTH/2.0 - PHONE_FRONT_OFFSET)*TrcUtil.MM_PER_INCH);
            final int CAMERA_VERTICAL_DISPLACEMENT = (int)(PHONE_HEIGHT_OFFSET*TrcUtil.MM_PER_INCH);
            final int CAMERA_LEFT_DISPLACEMENT = (int)((ROBOT_WIDTH/2.0 - PHONE_LEFT_OFFSET)*TrcUtil.MM_PER_INCH);
            //
            // Create a transformation matrix describing where the phone is on the robot.
            //
            // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
            // Lock it into Portrait for these numbers to work.
            //
            // Info:  The coordinate frame for the robot looks the same as the field.
            // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
            // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
            //
            // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
            // pointing to the LEFT side of the Robot.
            // The two examples below assume that the camera is facing forward out the front of the robot.

            // We need to rotate the camera around it's long axis to bring the correct camera forward.
            phoneYRotate = CAMERA_DIR == BACK ? -90.0f : 90.0f;

            // Rotate the phone vertical about the X axis if it's in portrait mode
            phoneXRotate = PHONE_IS_PORTRAIT ? 90.0f : 0.0f;

            // Next, translate the camera lens to where it is on the robot.
            // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.

            OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                            phoneYRotate, phoneZRotate, phoneXRotate));

            vuforiaVision = new VuforiaVision(this, cameraViewId, CAMERA_DIR, robotFromCamera);
        }
        //
        // TensorFlow slows down our threads really badly, so don't enable it if we don't need it.
        //
        if (USE_TENSORFLOW && (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            final VuforiaLocalizer.CameraDirection CAMERA_DIR = BACK;
            tensorFlowVision = new TensorFlowVision(tfodMonitorViewId, CAMERA_DIR, globalTracer);
            tensorFlowVision.setEnabled(true);
            globalTracer.traceInfo(moduleName, "Enabling TensorFlow.");
        }
        //
        // Initialize DriveBase.
        //
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

        if (USE_VELOCITY_CONTROL)
        {
            TrcPidController.PidCoefficients motorPidCoef = new TrcPidController.PidCoefficients(
                    RobotInfo3543.MOTOR_KP, RobotInfo3543.MOTOR_KI, RobotInfo3543.MOTOR_KD);
            leftFrontWheel.enableVelocityMode(RobotInfo3543.MOTOR_MAX_VELOCITY, motorPidCoef);
            rightFrontWheel.enableVelocityMode(RobotInfo3543.MOTOR_MAX_VELOCITY, motorPidCoef);
            leftRearWheel.enableVelocityMode(RobotInfo3543.MOTOR_MAX_VELOCITY, motorPidCoef);
            rightRearWheel.enableVelocityMode(RobotInfo3543.MOTOR_MAX_VELOCITY, motorPidCoef);
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
        gyroPidCtrl.setOutputRange(-RobotInfo3543.TURN_POWER_LIMIT, RobotInfo3543.TURN_POWER_LIMIT);

        pidDrive = new TrcPidDrive("pidDrive", driveBase, encoderXPidCtrl, encoderYPidCtrl, gyroPidCtrl);
        pidDrive.setStallTimeout(RobotInfo3543.PIDDRIVE_STALL_TIMEOUT);
        pidDrive.setBeep(androidTone);
        pidDrive.setMsgTracer(globalTracer, true);
        //
        // Initialize other subsystems.
        //

        //
        // Tell the driver initialization is complete.
        //
        speak("Init complete!");
    }   //Robot3543

}   //class Robot3543
