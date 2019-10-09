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

/**
 * @param <TeamSpecificRobotType> The specific subclass of {@link Robot} to use for the
 *                                {@link #robot} field. This allows team-specific code extending
 *                                this class to use the {@link #robot} field to access team-specific
 *                                subsystems without needing to cast it (to {@link team3543.Robot3543}
 *                                or {@link team6541.Robot6541} accordingly).
 */
public abstract class CommonTeleOp<TeamSpecificRobotType extends Robot>
        extends FtcOpMode
        implements TrcGameController.ButtonHandler
{
    protected enum DriveMode
    {
        TANK_MODE,
        HOLONOMIC_MODE,
    }   //enum DriveMode

    protected String moduleName = null;
    protected TeamSpecificRobotType robot = null;

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
        driverGamepad = new FtcGamepad("DriverGamepad", gamepad1, this);
        operatorGamepad = new FtcGamepad("OperatorGamepad", gamepad2, this);
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
                    double leftPower = driverGamepad.getLeftStickY(true)*drivePowerScale;
                    double rightPower = driverGamepad.getRightStickY(true)*drivePowerScale;
                    robot.driveBase.tankDrive(leftPower, rightPower, invertedDrive);
                    dashboard.displayPrintf(1, "Tank:left=%.1f,right=%.1f,inv=%s",
                            leftPower, rightPower, Boolean.toString(invertedDrive));
                    break;

                case HOLONOMIC_MODE:
                    double x = driverGamepad.getLeftStickX(true)*drivePowerScale;
                    double y = driverGamepad.getRightStickY(true)*drivePowerScale;
                    double rot = (driverGamepad.getRightTrigger(true) -
                            driverGamepad.getLeftTrigger(true))*drivePowerScale;
                    robot.driveBase.holonomicDrive(x, y, rot, invertedDrive);
                    dashboard.displayPrintf(1, "Mecan:x=%.1f,y=%.1f,rot=%.1f,inv=%s",
                            x, y, rot, Boolean.toString(invertedDrive));
                    break;
            }

            double elevatorPower = operatorGamepad.getRightStickY(true);
            robot.elevator.setPower(elevatorPower);

            double armExtenderPower = operatorGamepad.getLeftStickY(true);
            robot.armExtender.setPower(armExtenderPower);

            double wristPower = operatorGamepad.getLeftStickX(true);
            robot.wrist.setPower(wristPower);

            dashboard.displayPrintf(2, "DriveBase: x=%.2f,y=%.2f,heading=%.2f",
                    robot.driveBase.getXPosition(),
                    robot.driveBase.getYPosition(),
                    robot.driveBase.getHeading());
        }
    }   //runPeriodic

}   //class CommonTeleOp
