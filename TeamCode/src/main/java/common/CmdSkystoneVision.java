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

import java.util.Locale;

import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPose2D;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTrigger;
import trclib.TrcUtil;

public class CmdSkystoneVision implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;
    private static final double VISION_TIMEOUT = 1.0;

    private enum State
    {
        SETUP_VISION,
        GET_TARGET_POSE,
        SCAN_FOR_SKYSTONE,
        NEXT_SKYSTONE_POSITION,
        ALIGN_SKYSTONE,
        GOTO_SKYSTONE,
        DONE
    }   //enum State

    private static final String moduleName = "CmdSkystoneVision";

    private final Robot robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private TrcTrigger visionTrigger;
    private TrcPose2D skystonePose = null;
    private double visionTimeout = 0.0;
    private int scootCount = 2;
    private boolean scanningForSkyStone = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    public CmdSkystoneVision(Robot robot, CommonAuto.AutoChoices autoChoices, boolean useVisionTrigger)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);

        if (useVisionTrigger)
        {
            visionTrigger = new TrcTrigger("VisionTrigger", this::isTriggered, this::targetDetected);
        }

        sm.start(State.SETUP_VISION);
    }   //CmdSkystoneVision

    private boolean isTriggered()
    {
        TrcPose2D pose = robot.getSkyStonePose();

        if (pose != null)
        {
            skystonePose = pose;
        }

        return pose != null;
    }   //isTriggered

    private void targetDetected()
    {
        if (robot.pidDrive.isActive())
        {
            robot.pidDrive.cancel();
        }
    }   //targetDetected

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
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

        if (visionTrigger != null)
        {
            visionTrigger.setEnabled(false);
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
            double turnTarget = 0.0;
            State nextState = null;

            robot.dashboard.displayPrintf(1, "State: %s", state);

            switch (state)
            {
                case SETUP_VISION:
                    //
                    // Vuforia may take time to detect target, set a timeout for detection.
                    //
                    visionTimeout = TrcUtil.getCurrentTime() + VISION_TIMEOUT;
                    sm.setState(State.GET_TARGET_POSE);
                    //
                    // Intentionally falling through to the next state.
                    //
                case GET_TARGET_POSE:
                    //
                    // Get the detected skystone pose. If pose is null, it could be because Vuforia is still
                    // processing the image. So keep repeating this state until it finds the target or we pass
                    // vision timeout.
                    //
                    skystonePose = robot.getSkyStonePose();
                    if (skystonePose == null)
                    {
                        // Vuforia either did not detect the target or it's still busy processing the image.
                        robot.globalTracer.traceInfo("getTargetPose", "Skystone not found.");
                        if (TrcUtil.getCurrentTime() > visionTimeout)
                        {
                            // Can't find any skystone here, move on to the next position.
                            sm.setState(visionTrigger == null? State.NEXT_SKYSTONE_POSITION:
                                        scanningForSkyStone? State.GOTO_SKYSTONE: State.SCAN_FOR_SKYSTONE);
                            robot.speak("Not found.");
                        }
                    }
                    else
                    {
                        // Vuforia found the skystone.
                        robot.globalTracer.traceInfo(
                                "getTargetPose", "Skystone found at x=%.1f, y=%.1f.",
                                skystonePose.x, skystonePose.y);
                        robot.speak(String.format(Locale.US, "Sky stone found at %.1f inches.", skystonePose.x));
                        sm.setState(State.ALIGN_SKYSTONE);
                    }
                    break;

                case SCAN_FOR_SKYSTONE:
                    //
                    // Strafe across all three stone to find the skystone. When a skystone is spotted, the strafe
                    // will be interrupted by visionTrigger.
                    //
                    visionTrigger.setEnabled(true);
                    scanningForSkyStone = true;

                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE? -16.0: 16.0;
                    robot.pidDrive.setRelativeXTarget(xTarget, event);
                    sm.waitForSingleEvent(event, State.SETUP_VISION);
                    break;

                case NEXT_SKYSTONE_POSITION:
                    //
                    // We did not see the skystone, move one stone over and try again.
                    //
                    if (scootCount > 0)
                    {
                        scootCount--;
                        nextState = scootCount == 0? State.GOTO_SKYSTONE: State.SETUP_VISION;
                        xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE? -8.5: 8.5;
                        robot.pidDrive.setRelativeXTarget(xTarget, event);
                        sm.waitForSingleEvent(event, nextState);
                    }
                    else
                    {
                        //
                        // This is the last stone and we still can't detect the target. Just assume the one in front
                        // is a skystone. We will still get some points even if we are wrong, better than nothing!
                        //
                        robot.globalTracer.traceInfo(
                                "NextSkyStonePos", "Skystone not found, giving up.");
                        sm.setState(State.GOTO_SKYSTONE);
                    }
                    break;

                case ALIGN_SKYSTONE:
                    //
                    // We found the skystone but the robot may be slightly mis-aligned. Align the robot before
                    // grabbing the skystone.
                    //
                    if (visionTrigger != null)
                    {
                        visionTrigger.setEnabled(false);
                    }
                    xTarget = skystonePose.x;
                    robot.pidDrive.setRelativeXTarget(xTarget, event);
                    sm.waitForSingleEvent(event, State.GOTO_SKYSTONE);
                    break;

                case GOTO_SKYSTONE:
                    if (visionTrigger != null)
                    {
                        visionTrigger.setEnabled(false);
                    }

                    yTarget = 8.0;
                    robot.pidDrive.setRelativeYTarget(yTarget, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    cancel();
                    break;
            }

            robot.traceStateInfo(elapsedTime, state.toString(), xTarget, yTarget, turnTarget);
        }

        if (robot.pidDrive.isActive() && (debugXPid || debugYPid || debugTurnPid))
        {
            if (robot.battery != null)
            {
                robot.globalTracer.traceInfo("Battery", "Voltage=%5.2fV (%5.2fV)",
                        robot.battery.getVoltage(), robot.battery.getLowestVoltage());
            }

            robot.globalTracer.traceInfo(moduleName, "%s", robot.driveBase.getAbsolutePose());

            TrcPidController pidCtrl = robot.pidDrive.getXPidCtrl();
            if (debugXPid && pidCtrl != null)
            {
                pidCtrl.printPidInfo(robot.globalTracer, elapsedTime);
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

}   //class CmdSkystoneVision
