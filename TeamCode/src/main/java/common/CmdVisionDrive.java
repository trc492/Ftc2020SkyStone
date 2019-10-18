/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.Locale;

import trclib.TrcEvent;
import trclib.TrcPose2D;
import trclib.TrcRobot;
import trclib.TrcStateMachine;

public class CmdVisionDrive implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private enum State
    {
        MOVE_FORWARD,
        DO_VISION_DRIVE,
        GOTO_SKYSTONE,
        DONE
    }   //enum State

    private static final String moduleName = "CmdVisionDrive";

    private final Robot robot;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    public CmdVisionDrive(Robot robot)
    {
        robot.globalTracer.traceInfo(moduleName, "robot=%s", robot);

        this.robot = robot;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.MOVE_FORWARD);
    }   //CmdVisionDrive

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        if (robot.pidDrive.isActive())
        {
            robot.pidDrive.cancel();
        }
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    @SuppressWarnings("unused")
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: Disabled or waiting...");
        }
        else
        {
            double xTarget = 0.0;
            double yTarget = 0.0;

            robot.dashboard.displayPrintf(1, "State: %s", state);

            switch (state)
            {
                case MOVE_FORWARD:
                    robot.pidDrive.getYPidCtrl().setOutputLimit(0.5);
                    yTarget = 22.0;
                    robot.pidDrive.setTarget(xTarget, yTarget, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DO_VISION_DRIVE);
                    break;

                case DO_VISION_DRIVE:
                    TrcPose2D skystonePose = robot.getSkyStonePose();

                    if (skystonePose != null)
                    {
                        xTarget = skystonePose.x;
                        robot.speak(String.format(Locale.US, "Skystone is found at %.1f", xTarget));
                        robot.pidDrive.setTarget(xTarget, yTarget, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.GOTO_SKYSTONE);
                    }
                    break;

                case GOTO_SKYSTONE:
                    yTarget = 6.0;
                    robot.pidDrive.setTarget(xTarget, yTarget, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done, restore everything.
                    //
                    sm.stop();
                    break;
            }

            robot.traceStateInfo(elapsedTime, state.toString(), xTarget, yTarget, robot.targetHeading);
        }

        if (robot.pidDrive.isActive() && (debugXPid || debugYPid || debugTurnPid))
        {
            if (robot.battery != null)
            {
                robot.globalTracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
                        robot.battery.getVoltage(), robot.battery.getLowestVoltage());
            }

            if (debugXPid && robot.encoderXPidCtrl != null)
            {
                robot.pidDrive.getXPidCtrl().printPidInfo(robot.globalTracer, elapsedTime);
            }

            if (debugYPid)
            {
                robot.pidDrive.getYPidCtrl().printPidInfo(robot.globalTracer, elapsedTime);
            }

            if (debugTurnPid)
            {
                robot.pidDrive.getTurnPidCtrl().printPidInfo(robot.globalTracer, elapsedTime);
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdVisionDrive
