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

package common;

import android.speech.tts.TextToSpeech;

import com.qualcomm.ftcrobotcontroller.R;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import javax.annotation.Nonnull;

import ftclib.FtcAndroidTone;
import ftclib.FtcBNO055Imu;
import ftclib.FtcDcMotor;
import ftclib.FtcOpMode;
import ftclib.FtcRobotBattery;
import ftclib.FtcVuforia;
import hallib.HalDashboard;
import trclib.TrcDbgTrace;
import trclib.TrcDriveBase;
import trclib.TrcGyro;
import trclib.TrcHashMap;
import trclib.TrcHomographyMapper;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcPose2D;
import trclib.TrcRobot;
import trclib.TrcUtil;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Robot
{
    protected enum DriveMode
    {
        TANK_MODE,
        HOLONOMIC_MODE,
    }   //enum DriveMode

    private static final String OPENCV_NATIVE_LIBRARY_NAME = "opencv_java3";
    //
    // Global objects.
    //
    public String robotName;
    public TrcHashMap<String, Boolean> preferences;
    public FtcOpMode opMode;
    public HalDashboard dashboard;
    public TrcDbgTrace globalTracer;
    public FtcAndroidTone androidTone;
    public TextToSpeech textToSpeech = null;
    public FtcRobotBattery battery = null;
    public LEDIndicator ledIndicator = null;
    //
    // Sensors.
    //
    public FtcBNO055Imu imu;
    public TrcGyro gyro;
    //
    // Vision subsystems.
    //
    public FtcVuforia vuforia = null;
    public VuforiaVision vuforiaVision = null;
    public TensorFlowVision tensorFlowVision = null;
    public TensorFlowVision.TargetInfo[] targetsInfo = null;
    public long detectionIntervalTotalTime = 0;
    public long detectionIntervalStartTime = 0;
    public int detectionSuccessCount = 0;
    public int detectionFailedCount = 0;
    public String targetFinder = null;
    //
    // DriveBase subsystem.
    //
    public FtcDcMotor leftFrontWheel = null;
    public FtcDcMotor rightFrontWheel = null;
    public FtcDcMotor leftRearWheel = null;
    public FtcDcMotor rightRearWheel = null;

    public TrcDriveBase driveBase = null;
    public DriveMode driveMode = DriveMode.HOLONOMIC_MODE;
    public TrcPidController encoderXPidCtrl = null;
    public TrcPidController encoderYPidCtrl = null;
    public TrcPidController gyroPidCtrl = null;
    public TrcPidDrive pidDrive = null;
    public TrcPose2D fieldOrigin = null;

    public TrcPidController.PidCoefficients tunePidCoeff = new TrcPidController.PidCoefficients();
    //
    // Other common subsystems.
    //
    @Nonnull public Elevator elevator = null;
    @Nonnull public Wrist wrist = null;
    @Nonnull public GenericGrabber grabber = null;
    @Nonnull public FoundationLatch foundationLatch = null;

    public Robot(
            TrcRobot.RunMode runMode, String robotName,
            TrcHashMap<String, Boolean> preferences, TrcHashMap<String, Object> phoneParams)
    {
        //
        // Initialize global objects.
        //
        this.robotName = robotName;
        this.preferences = preferences;
        opMode = FtcOpMode.getInstance();
        opMode.hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();
        globalTracer = FtcOpMode.getGlobalTracer();
        dashboard.setTextView(
                ((FtcRobotControllerActivity)opMode.hardwareMap.appContext).findViewById(R.id.textOpMode));
        androidTone = new FtcAndroidTone("AndroidTone");

        if (preferences.getBoolean("useSpeech"))
        {
            textToSpeech = FtcOpMode.getInstance().getTextToSpeech();
            speak("Init starting");
        }

        if (preferences.getBoolean("useVuforia") || preferences.getBoolean("useTensorFlow"))
        {
            final String VUFORIA_LICENSE_KEY =
                    "ATu19Kj/////AAAAGcw4SDCVwEBSiKcUtdmQd2aOugrxo/OgeBJUt7XwMSi3e0KSZaylbsTnWp8EBxyA5o/00JFJVDY1OxJ" +
                    "XLxQOpz1tbM4ex1sl1EbF25olEZ3w9xXZ1QaqMP+5T63VqTwvkgKbtM+dS+tLi8EHMvJ2viYf6WwOE776e0s3QNfl/XvONM" +
                    "XS4ZtEWLNeiSEMTCdO9bdeaxnSb2RfErcmjadAThDWf6PC9HrMRHLmgfcFaZlj5JN+figOjgKhyQZeYYrcDEm0lICN5kAr2" +
                    "pdfNKNOii3A80eXyTVDfPGfzTwVa4eNBY/SgmoIdBbMPb3hfZBOz7GVoVHHQWbCNbzm31p1OY+zqPPWMfzzpyiJ4mA9bLTQ";

            int cameraViewId = !preferences.getBoolean("showVuforiaView") ? -1 :
                    opMode.hardwareMap.appContext.getResources().getIdentifier(
                            "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            vuforia = new FtcVuforia(
                    VUFORIA_LICENSE_KEY, cameraViewId,
                    (VuforiaLocalizer.CameraDirection) phoneParams.get("cameraDir"),
                    (VuforiaLocalizer.Parameters.CameraMonitorFeedback) phoneParams.get("cameraMonitorFeedback"));
        }

        if (preferences.getBoolean("hasRobot"))
        {
            if (preferences.getBoolean("useBatteryMonitor"))
            {
                battery = new FtcRobotBattery();
            }

            if (preferences.getBoolean("useBlinkin"))
            {
                ledIndicator = new LEDIndicator();
            }

            //
            // Initialize sensors.
            //
            imu = new FtcBNO055Imu("imu");
            gyro = imu.gyro;
        }
    }   //Robot

    @Override
    public String toString()
    {
        return robotName;
    }   //toString

    public void startMode(TrcRobot.RunMode runMode)
    {
        final String funcName = "startMode";
        //
        // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
        //
        if (gyro != null)
        {
            gyro.setEnabled(true);
        }
        //
        // Enable odometry only for autonomous or test modes.
        //
        if (driveBase != null && (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            driveBase.setOdometryEnabled(true);
        }
        //
        // Vision generally will impact performance, so we only enable it if it's needed such as in autonomous.
        //
        if (vuforiaVision != null && (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE))
        {
            globalTracer.traceInfo(funcName, "Enabling Vuforia.");
            vuforiaVision.setEnabled(true, preferences.getBoolean("useFlashLight"));
        }
    }   //startMode

    public void stopMode(TrcRobot.RunMode runMode)
    {
        if (vuforiaVision != null)
        {
            vuforiaVision.setEnabled(false, preferences.getBoolean("useFlashLight"));
        }

        if (tensorFlowVision != null)
        {
            globalTracer.traceInfo("RobotStopMode", "Shutting down TensorFlow.");
            tensorFlowVision.shutdown();
            tensorFlowVision = null;
        }
        //
        // Disable the gyro integrator.
        //
        if (driveBase != null)
        {
            driveBase.setOdometryEnabled(false);
        }

        if (gyro != null)
        {
            gyro.setEnabled(false);
        }

        if (textToSpeech != null)
        {
            textToSpeech.stop();
            textToSpeech.shutdown();
        }
    }   //stopMode

    public void traceStateInfo(double elapsedTime, String stateName, double xDistance, double yDistance, double heading)
    {
        if (driveBase != null)
        {
            if (battery != null)
            {
                globalTracer.traceInfo(
                        robotName,
                        "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f,volt=%5.2fV(%5.2fV)",
                        elapsedTime, stateName,
                        driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance, driveBase.getHeading(),
                        heading, battery.getVoltage(), battery.getLowestVoltage());
            }
            else
            {
                globalTracer.traceInfo(
                        robotName,
                        "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f",
                        elapsedTime, stateName,
                        driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance, driveBase.getHeading(),
                        heading);
            }
        }
    }   //traceStateInfo

    public void speak(String sentence)
    {
        if (textToSpeech != null)
        {
            textToSpeech.speak(sentence, TextToSpeech.QUEUE_FLUSH, null);
        }
    }   //speak

    protected void initVuforia(TrcHashMap<String, Object> phoneParams, double robotLength, double robotWidth)
    {
        float phoneXRotate;
        float phoneYRotate;
        float phoneZRotate = 0.0f;
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
        final int CAMERA_FORWARD_DISPLACEMENT =
                (int)((robotLength/2.0 - phoneParams.getDouble("phoneFrontOffset"))*TrcUtil.MM_PER_INCH);
        final int CAMERA_VERTICAL_DISPLACEMENT =
                (int)(phoneParams.getDouble("phoneHeightOffset")*TrcUtil.MM_PER_INCH);
        final int CAMERA_LEFT_DISPLACEMENT =
                (int)((robotWidth/2.0 - phoneParams.getDouble("phoneLeftOffset"))*TrcUtil.MM_PER_INCH);
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
        phoneYRotate = phoneParams.get("cameraDir") == BACK ? -90.0f : 90.0f;

        // Rotate the phone vertical about the X axis if it's in portrait mode
        phoneXRotate = phoneParams.getBoolean("phoneIsPortrait") ? 90.0f : 0.0f;

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        phoneYRotate, phoneZRotate, phoneXRotate));

        vuforiaVision = new VuforiaVision(this, vuforia, robotFromCamera);
    }   //initVuforia

    protected void initTensorFlow(
            boolean showTensorFlowView, TrcHomographyMapper.Rectangle cameraRect,
            TrcHomographyMapper.Rectangle worldRect)
    {
        System.loadLibrary(OPENCV_NATIVE_LIBRARY_NAME);//CodeReview: Is this the right place to load OpenCV?!

        int tfodMonitorViewId = !showTensorFlowView ? -1 :
                opMode.hardwareMap.appContext.getResources().getIdentifier(
                        "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        tensorFlowVision = new TensorFlowVision(
                vuforia, tfodMonitorViewId, cameraRect, worldRect, globalTracer);
        tensorFlowVision.setEnabled(true, preferences.getBoolean("useFlashLight"));
        globalTracer.traceInfo(robotName, "Enabling TensorFlow.");
    } //initTensorFlow

    public TrcPose2D getSkyStonePose()
    {
        final String funcName = "getSkyStonePose";
        TrcPose2D pose = null;
        targetFinder = null;

        if (vuforiaVision != null)
        {
            OpenGLMatrix robotLocation = vuforiaVision.getRobotLocation(VuforiaVision.skystoneTargetName);
            if (robotLocation != null)
            {
                VectorF translation = vuforiaVision.getLocationTranslation(robotLocation);
                Orientation orientation = vuforiaVision.getLocationOrientation(robotLocation);
                pose = new TrcPose2D(
                        -translation.get(0)/TrcUtil.MM_PER_INCH, -translation.get(1)/TrcUtil.MM_PER_INCH,
                        orientation.thirdAngle);
                targetFinder = "Vuforia:" + vuforiaVision.getLastSeenImageName();
            }
        }

        if (pose == null && tensorFlowVision != null)
        {
            TensorFlowVision.TargetInfo[] targetsInfo;

            targetsInfo = tensorFlowVision.getDetectedTargetsInfo(TensorFlowVision.LABEL_SKYSTONE);
            if (targetsInfo != null && targetsInfo.length > 0)
            {
                pose = new TrcPose2D(
                        targetsInfo[0].targetBottomCenter.x, targetsInfo[0].targetBottomCenter.y, targetsInfo[0].angle);
                targetFinder = "TensorFlow";
            }
        }

        if (pose == null)
        {
            globalTracer.traceInfo(funcName, "***** Skystone not found!");
        }
        else
        {
            globalTracer.traceInfo(funcName, "***** %s: x=%.1f, y=%.1f, angle=%.1f",
                    targetFinder, pose.x, pose.y, pose.heading);
        }

        return pose;
    }   //getSkyStonePose

    public void setFieldOrigin(TrcPose2D pose)
    {
        fieldOrigin = pose;
    }   //setFieldOrigin

    public TrcPose2D getRobotPose()
    {
        TrcPose2D robotPose = vuforiaVision.getRobotPose("Stone Target", true);
        if (robotPose != null && fieldOrigin != null)
        {
            // There is a field origin, make the robot pose relative to it.
            robotPose = robotPose.relativeTo(fieldOrigin);
        }

        return robotPose;
    }   //getRobotPose

}   //class Robot
