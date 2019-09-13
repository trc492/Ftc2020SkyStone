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

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftclib.FtcAndroidTone;
import ftclib.FtcBNO055Imu;
import ftclib.FtcDcMotor;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcRobotBattery;
import hallib.HalDashboard;
import team3543.R;
import trclib.TrcDbgTrace;
import trclib.TrcDriveBase;
import trclib.TrcGyro;
import trclib.TrcPidController;
import trclib.TrcPidDrive;
import trclib.TrcRobot;

public class Robot implements FtcMenu.MenuButtons
{
    protected enum DriveMode
    {
        TANK_MODE,
        HOLONOMIC_MODE,
    }   //enum DriveMode

    //
    // Global objects.
    //
    public String moduleName;
    public FtcOpMode opMode;
    public HalDashboard dashboard;
    public TrcDbgTrace globalTracer;
    public FtcAndroidTone androidTone;
    public TextToSpeech textToSpeech = null;
    public FtcRobotBattery battery = null;
    //
    // Sensors.
    //
    public FtcBNO055Imu imu;
    public TrcGyro gyro;
    public double targetHeading = 0.0;
    //
    // Vision subsystems.
    //
    public VuforiaVision vuforiaVision = null;
    public TensorFlowVision tensorFlowVision = null;
    public TensorFlowVision.TargetInfo[] targetsInfo = null;
    public long detectionIntervalTotalTime = 0;
    public long detectionIntervalStartTime = 0;
    public int detectionSuccessCount = 0;
    public int detectionFailedCount = 0;
    //
    // DriveBase subsystem.
    //
    public FtcDcMotor leftFrontWheel;
    public FtcDcMotor rightFrontWheel;
    public FtcDcMotor leftRearWheel;
    public FtcDcMotor rightRearWheel;

    public TrcDriveBase driveBase;
    public DriveMode driveMode;
    public TrcPidController encoderXPidCtrl;
    public TrcPidController encoderYPidCtrl;
    public TrcPidController gyroPidCtrl;
    public TrcPidDrive pidDrive;

    public TrcPidController.PidCoefficients tunePidCoeff = new TrcPidController.PidCoefficients();
    //
    // Other common subsystems.
    //

    public Robot(TrcRobot.RunMode runMode, String robotName, boolean useSpeech, boolean useBatteryMonitor)
    {
        //
        // Initialize global objects.
        //
        moduleName = robotName;
        opMode = FtcOpMode.getInstance();
        opMode.hardwareMap.logDevices();
        dashboard = HalDashboard.getInstance();
        globalTracer = FtcOpMode.getGlobalTracer();
        dashboard.setTextView(
                ((FtcRobotControllerActivity)opMode.hardwareMap.appContext).findViewById(R.id.textOpMode));
        androidTone = new FtcAndroidTone("AndroidTone");

        if (useSpeech)
        {
            textToSpeech = FtcOpMode.getInstance().getTextToSpeech();
            speak("Init starting");
        }

        if (useBatteryMonitor)
        {
            battery = new FtcRobotBattery();
        }
        //
        // Initialize sensors.
        //
        imu = new FtcBNO055Imu("imu");
        gyro = imu.gyro;
    }   //Robot

    public void startMode(TrcRobot.RunMode runMode)
    {
        final String funcName = "startMode";
        //
        // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
        //
        gyro.setEnabled(true);
        targetHeading = 0.0;
        //
        // Vision generally will impact performance, so we only enable it if it's needed such as in autonomous.
        //
        if (vuforiaVision != null && runMode == TrcRobot.RunMode.AUTO_MODE)
        {
            globalTracer.traceInfo(funcName, "Enabling Vuforia.");
            vuforiaVision.setEnabled(true);
        }
        //
        // Enable odometry only for autonomous or test modes.
        //
        if (runMode == TrcRobot.RunMode.AUTO_MODE || runMode == TrcRobot.RunMode.TEST_MODE)
        {
            driveBase.setOdometryEnabled(true);
        }
    }   //startMode

    public void stopMode(TrcRobot.RunMode runMode)
    {
        //
        // Disable the gyro integrator.
        //
        gyro.setEnabled(false);

        if (textToSpeech != null)
        {
            textToSpeech.stop();
            textToSpeech.shutdown();
        }

        if (vuforiaVision != null)
        {
            vuforiaVision.setEnabled(false);
        }

        if (tensorFlowVision != null)
        {
            globalTracer.traceInfo("RobotStopMode", "Shutting down TensorFlow.");
            tensorFlowVision.shutdown();
            tensorFlowVision = null;
        }

        driveBase.setOdometryEnabled(false);
    }   //stopMode

    public void traceStateInfo(double elapsedTime, String stateName, double xDistance, double yDistance, double heading)
    {
        if (battery != null)
        {
            globalTracer.traceInfo(
                    moduleName,
                    "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f,volt=%5.2fV(%5.2fV)",
                    elapsedTime, stateName,
                    driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance, driveBase.getHeading(),
                    heading, battery.getVoltage(), battery.getLowestVoltage());
        }
        else
        {
            globalTracer.traceInfo(
                    moduleName,
                    "[%5.3f] >>>>> %s: xPos=%6.2f/%6.2f,yPos=%6.2f/%6.2f,heading=%6.1f/%6.1f",
                    elapsedTime, stateName,
                    driveBase.getXPosition(), xDistance, driveBase.getYPosition(), yDistance, driveBase.getHeading(),
                    heading);
        }
    }   //traceStateInfo

    public void speak(String sentence)
    {
        if (textToSpeech != null)
        {
            textToSpeech.speak(sentence, TextToSpeech.QUEUE_FLUSH, null);
        }
    }   //speak

    //
    // Implements FtcMenu.MenuButtons interface.
    //

    @Override
    public boolean isMenuUpButton()
    {
        return opMode.gamepad1.dpad_up;
    }   //isMenuUpButton

    @Override
    public boolean isMenuDownButton()
    {
        return opMode.gamepad1.dpad_down;
    }   //isMenuDownButton

    @Override
    public boolean isMenuAltUpButton()
    {
        return opMode.gamepad1.left_bumper;
    }   //isMenuAltUpButton

    @Override
    public boolean isMenuAltDownButton()
    {
        return opMode.gamepad1.right_bumper;
    }   //isMenuAltDownButton

    @Override
    public boolean isMenuEnterButton()
    {
        return opMode.gamepad1.dpad_right;
    }   //isMenuEnterButton

    @Override
    public boolean isMenuBackButton()
    {
        return opMode.gamepad1.dpad_left;
    }   //isMenuBackButton

}   //class Robot
