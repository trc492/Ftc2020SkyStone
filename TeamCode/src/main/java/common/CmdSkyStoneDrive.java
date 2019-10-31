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

import trclib.TrcEvent;
import trclib.TrcPose2D;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcUtil;

public class CmdSkyStoneDrive implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;
    private static final double VISION_TIMEOUT = 1.0;

    private enum State
    {
        MOVE_CLOSER,
        SETUP_VISION,
        GET_TARGET_POSE,
        NEXT_SKYSTONE_POSITION,
        GOTO_SKYSTONE,
        DONE
    }   //enum State

    private static final String moduleName = "CmdSkyStoneDrive";

    private final Robot robot;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private double visionTimeout = 0.0;
    private int scootCount = 2;
    private double xPos = 0.0, yPos = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    public CmdSkyStoneDrive(Robot robot)
    {
        robot.globalTracer.traceInfo(moduleName, "robot=%s", robot);

        this.robot = robot;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.MOVE_CLOSER);
    }   //CmdSkyStoneDrive

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
                case MOVE_CLOSER:
                    //
                    // Move closer slowly for a distance so Vuforia can detect the target.
                    //
                    robot.pidDrive.getYPidCtrl().saveAndSetOutputLimit(0.5);
                    xPos = robot.driveBase.getXPosition();
                    yPos = robot.driveBase.getYPosition();
                    yTarget = 18.0;
                    yPos += yTarget;
                    robot.pidDrive.setTarget(xTarget, yTarget, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.SETUP_VISION);
                    break;

                case SETUP_VISION:
                    robot.pidDrive.getYPidCtrl().restoreOutputLimit();
                    visionTimeout = TrcUtil.getCurrentTime() + VISION_TIMEOUT;
                    sm.setState(State.GET_TARGET_POSE);
                    break;

                case GET_TARGET_POSE:
                    //
                    // Get the detected skystone pose. If pose is null, it could be because Vuforia is still
                    // processing the image. So keep repeating this state until it finds the target or we pass
                    // vision timeout.
                    //
                    TrcPose2D pose = robot.getSkyStonePose();

                    if (pose == null)
                    {
                        // Vuforia either did not detect the target or it's still busy processing the image.
                        robot.globalTracer.traceInfo("getTargetPose", "Skystone not found.");
                        if (TrcUtil.getCurrentTime() > visionTimeout)
                        {
                            // Can't find any skystone here, move on to the next position.
                            sm.setState(State.NEXT_SKYSTONE_POSITION);
                        }
                    }
                    else if (Math.abs(pose.x) > 4.0)
                    {
                        // Vuforia found the skystone but it is to our right.
                        robot.globalTracer.traceInfo(
                                "getTargetPose", "Skystone found to the right (x=%.1f, y=%.1f).",
                                pose.x, pose.y);
                        sm.setState(State.NEXT_SKYSTONE_POSITION);
                    }
                    else
                    {
                        // Vuforia found the skystone right in front.
                        robot.globalTracer.traceInfo(
                                "getTargetPose", "Skystone found in front (x=%.1f, y=%.1f).",
                                pose.x, pose.y);
                        sm.setState(State.GOTO_SKYSTONE);
                    }
                    break;

                case NEXT_SKYSTONE_POSITION:
                    if (scootCount > 0)
                    {
                        scootCount--;
                        xPos += 8.0;
                        xTarget = xPos - robot.driveBase.getXPosition();
                        robot.pidDrive.setTarget(xTarget, yTarget, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.SETUP_VISION);
                    }
                    else
                    {
                        // Still can't detect the target. Just assume the one in front is a skystone. We will still
                        // get some points even if we are wrong, better than nothing!
                        sm.setState(State.GOTO_SKYSTONE);
                    }
                    break;

                case GOTO_SKYSTONE:
                    yPos += 30.0;
                    yTarget = yPos - robot.driveBase.getYPosition();
                    robot.pidDrive.setTarget(xTarget, yTarget, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
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

}   //class CmdSkyStoneDrive
