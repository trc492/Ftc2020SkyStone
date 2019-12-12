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

import ftclib.FtcGamepad;
import ftclib.FtcOpMode;
import hallib.HalDashboard;
import trclib.TrcGameController;
import trclib.TrcRobot;

public abstract class CommonTeleOp extends FtcOpMode
{
    private static final double SLOW_DRIVE_POWER_SCALE = 0.5;
    //
    // Common objects.
    //
    protected HalDashboard dashboard;
    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;
    //
    // Team specific objects.
    //
    private Robot robot = null;

    private double drivePowerScale = SLOW_DRIVE_POWER_SCALE;
    private boolean invertedDrive = false;

    protected void setRobot(Robot robot)
    {
        this.robot = robot;
    }   //setRobot

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //
        dashboard = robot.dashboard;
        //
        // Initializing Gamepads.
        //
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1, this::buttonEvent);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2, this::buttonEvent);
        driverGamepad.setYInverted(true);
        operatorGamepad.setYInverted(true);
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        dashboard.clearDisplay();
        robot.startMode(nextMode);
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        robot.stopMode(prevMode);
        printPerformanceMetrics(robot.globalTracer);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        //
        // DriveBase subsystem.
        //
        if (robot.driveBase != null)
        {
            switch (robot.driveMode)
            {
                case TANK_MODE:
                    double leftPower = driverGamepad.getLeftStickY(true) * drivePowerScale;
                    double rightPower = driverGamepad.getRightStickY(true) * drivePowerScale;
                    robot.driveBase.tankDrive(leftPower, rightPower, invertedDrive);
                    dashboard.displayPrintf(1, "Tank:left=%.1f,right=%.1f,inv=%s",
                            leftPower, rightPower, Boolean.toString(invertedDrive));
                    break;

                case HOLONOMIC_MODE:
                    double x = driverGamepad.getLeftStickX(true) * drivePowerScale;
                    double y = driverGamepad.getRightStickY(true) * drivePowerScale;
                    double rot = (driverGamepad.getRightTrigger(true) -
                            driverGamepad.getLeftTrigger(true)) * drivePowerScale;
                    robot.driveBase.holonomicDrive(x, y, rot, invertedDrive);
                    dashboard.displayPrintf(1, "Mecan:x=%.1f,y=%.1f,rot=%.1f,inv=%s",
                            x, y, rot, Boolean.toString(invertedDrive));
                    break;
            }

            dashboard.displayPrintf(2, "DriveBase: x=%.2f,y=%.2f,heading=%.2f",
                    robot.driveBase.getXPosition(),
                    robot.driveBase.getYPosition(),
                    robot.driveBase.getHeading());

        }
        //
        // Other subsystems.
        //
        double elevatorPower = operatorGamepad.getRightStickY(true);
        double elevatorPos = 0.0;
        boolean elevatorDownSwitch = false;
        boolean elevatorUpSwitch = false;
        if (robot.elevator != null)
        {
            robot.elevator.setPower(elevatorPower);
            elevatorPos = robot.elevator.getPosition();
            elevatorDownSwitch = robot.elevator.isLowerLimitSwitchActive();
            elevatorUpSwitch = robot.elevator.isUpperLimitSwitchActive();
        }

        dashboard.displayPrintf(
                3, "ElevatorPower=%.1f, ElevatorPos=%.1f, ElevatorLimitSwitch=[%b, %b]",
                elevatorPower, elevatorPos, elevatorDownSwitch, elevatorUpSwitch);

        if (robot.grabber != null && robot.backFoundationLatch != null)
        {
            dashboard.displayPrintf(
                    4, "GrabberPos=%.1f, FoundationLatchPos=%.1f",
                    robot.grabber.getPosition(), robot.backFoundationLatch.getPosition());
        }
    }   //runPeriodic

    public void buttonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        if (gamepad == driverGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    if (pressed)
                    {
                        robot.startSong(0, !robot.lesMiserablesPlaying);
                    }
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (pressed)
                    {
                        robot.startSong(1, !robot.starWarsPlaying);
                    }
                    break;

                case FtcGamepad.GAMEPAD_X:
                    if (robot.backFoundationLatch != null && pressed)
                    {
                        robot.backFoundationLatch.grab();
                    }
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    if (robot.backFoundationLatch != null && pressed)
                    {
                        robot.backFoundationLatch.release();
                    }
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    drivePowerScale = pressed? 1.0: SLOW_DRIVE_POWER_SCALE;
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    invertedDrive = pressed;
                    break;
            }
        }
        else if (gamepad == operatorGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    if (robot.grabber != null && pressed)
                    {
                        robot.grabber.grab();
                    }
                    break;

                case FtcGamepad.GAMEPAD_B:
                    if (robot.grabber != null && pressed)
                    {
                        robot.grabber.release();
                    }
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    robot.elevator.setManualOverride(pressed);
                    break;

                case FtcGamepad.GAMEPAD_DPAD_UP:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    break;

                case FtcGamepad.GAMEPAD_BACK:
                    if (pressed)
                    {
                        robot.elevator.zeroCalibrate();
                    }
                    break;
            }
        }
    }   //buttonEvent

}   //class CommonTeleOp
