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
    protected enum DriveMode
    {
        TANK_MODE,
        HOLONOMIC_MODE,
    }   //enum DriveMode

    protected String moduleName = null;
    protected Robot robot = null;

    protected HalDashboard dashboard;

    protected FtcGamepad driverGamepad;
    protected FtcGamepad operatorGamepad;

    protected DriveMode driveMode = DriveMode.HOLONOMIC_MODE;
    protected double drivePowerScale = 1.0;
    protected boolean invertedDrive = false;

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
            switch (driveMode)
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
        if (robot.elevator != null)
        {
            robot.elevator.setPower(elevatorPower);
            elevatorPos = robot.elevator.getPosition();
        }

        double armExtenderPower = operatorGamepad.getLeftStickY(true);
        if (robot.armExtender != null)
        {
            robot.armExtender.setPower(armExtenderPower);
        }

        double wristPower = operatorGamepad.getLeftStickX(true);
        if (robot.wrist != null)
        {
            robot.wrist.setPower(wristPower);
        }

        dashboard.displayPrintf(3, "ElevatorPower=%.1f, ArmExtenderPower=%.1f, WristPower=%.1f, EleLowerLimit=%s, EleUpperLimit=%s, elePos=%.1f",
                elevatorPower, armExtenderPower, wristPower, robot.elevator.isLowerLimitSwitchActive(), robot.elevator.isUpperLimitSwitchActive(), robot.elevator.getPosition());

    }   //runPeriodic

    public void buttonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        if (gamepad == driverGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    drivePowerScale = pressed? 0.5: 1.0;
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
                    if (robot.foundationLatch != null && pressed)
                    {
                        robot.foundationLatch.grab();
                    }
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    if (robot.foundationLatch != null && pressed)
                    {
                        robot.foundationLatch.release();
                    }
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    robot.elevator.setManualOverride(pressed);
                    break;

                case FtcGamepad.GAMEPAD_DPAD_UP:
                    if (pressed)
                        robot.elevator.setPosition(12.0);
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    break;

                case FtcGamepad.GAMEPAD_BACK:
                    if (pressed)
                        robot.elevator.zeroCalibrate();
                    break;

                case FtcGamepad.GAMEPAD_START:
                    break;
            }
        }
    }   //buttonEvent

}   //class CommonTeleOp
