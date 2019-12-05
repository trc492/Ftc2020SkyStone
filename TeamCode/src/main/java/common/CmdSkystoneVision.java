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
    private static final double VISION_TIMEOUT = 0.5;

    private enum State
    {
        SCAN_FOR_SKYSTONE,
        SETUP_VISION,
        GET_TARGET_POSE,
        ALIGN_SKYSTONE,
        GOTO_SKYSTONE,
        DONE
    }   //enum State

    private static final String moduleName = "CmdSkystoneVision";

    private final Robot robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final double grabberOffset;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private final TrcTrigger visionTrigger;
    private final double allianceDirection;
    private final boolean wallStart;
    private int scootCount;
    private TrcPose2D skystonePose = null;
    private double visionTimeout = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the object containing all autonomous choices.
     * @param grabberOffset specifies the grabber X offset from the center of the robot in inches.
     * @param useVisionTrigger specifies true to use vision trigger, false otherwise.
     */
    public CmdSkystoneVision(
            Robot robot, CommonAuto.AutoChoices autoChoices, double grabberOffset, boolean useVisionTrigger)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;
        this.grabberOffset = grabberOffset;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        visionTrigger = useVisionTrigger?
                new TrcTrigger("VisionTrigger", this::isTriggered, this::targetDetected): null;
        allianceDirection = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE? 1.0: -1.0;
        wallStart = Math.abs(robot.driveBase.getXPosition()) < RobotInfo.ROBOT_START_X_MID;
        scootCount = wallStart? 1: 2;
        sm.start(useVisionTrigger? State.SCAN_FOR_SKYSTONE: State.SETUP_VISION);
    }   //CmdSkystoneVision

    private boolean isTriggered()
    {
        TrcPose2D pose = robot.getSkyStonePose();

        if (pose != null)
        {
            skystonePose = pose;
            robot.globalTracer.traceInfo(
                    "visionTrigger", "Skystone found at x=%.1f, y=%.1f.",
                    skystonePose.x, skystonePose.y);
            robot.speak(String.format(Locale.US, "Sky stone found at %.1f inches.", skystonePose.x));
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
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            double xTarget = 0.0;
            double yTarget = 0.0;
            double turnTarget = 0.0;
            State nextState;

            robot.dashboard.displayPrintf(1, "State: %s", state);

            switch (state)
            {
                case SCAN_FOR_SKYSTONE:
                    //
                    // Strafe across all three stone to find the skystone. When a skystone is spotted, the strafe
                    // will be interrupted by visionTrigger.
                    //
                    visionTrigger.setEnabled(true);

                    xTarget = wallStart? RobotInfo.SKYSTONE_SCAN_DISTANCE_WALL: RobotInfo.SKYSTONE_SCAN_DISTANCE_FAR;
                    xTarget *= allianceDirection;
                    robot.pidDrive.setRelativeXTarget(xTarget, event);
                    sm.waitForSingleEvent(event, State.ALIGN_SKYSTONE);
                    break;

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
                        if (TrcUtil.getCurrentTime() > visionTimeout)
                        {
                            if (scootCount > 0)
                            {
                                robot.globalTracer.traceInfo(
                                        "GetTargetPose", "Skystone not found, try next stone.");
                                robot.speak("Not found, try next.");
                                scootCount--;
                                xTarget = -9.0*allianceDirection;
                                // If this is the last stone, don't need to check it's a skystone, just grab and go.
                                nextState = scootCount == 0? State.ALIGN_SKYSTONE: State.SETUP_VISION;
                                robot.pidDrive.setRelativeXTarget(xTarget, event);
                                sm.waitForSingleEvent(event, nextState);
                            }
                            else
                            {
                                //
                                // Should never come here but handle it just in case.
                                //
                                robot.globalTracer.traceInfo(
                                        "GetTargetPose", "Skystone not found, giving up.");
                                robot.speak("Not found, give up.");
                                sm.setState(State.ALIGN_SKYSTONE);
                            }
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

                case ALIGN_SKYSTONE:
                    //
                    // Either we found the skystone or we pretend the one in front is the skystone.
                    // If we did find the skystone, it may be mis-aligned, so let's align to it before we grab it.
                    //
                    if (visionTrigger != null)
                    {
                        visionTrigger.setEnabled(false);
                    }

                    xTarget = grabberOffset;
                    if (skystonePose != null)
                    {
                        xTarget += skystonePose.x;
                    }

                    if (xTarget == 0.0)
                    {
                        sm.setState(State.GOTO_SKYSTONE);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        robot.pidDrive.setRelativeXTarget(xTarget, event);
                        sm.waitForSingleEvent(event, State.GOTO_SKYSTONE);
                        break;
                    }

                case GOTO_SKYSTONE:
                    yTarget = RobotInfo.ABS_GRAB_SKYSTONE_POS_Y;
                    robot.pidDrive.setAbsoluteYTarget(yTarget, event);
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

            robot.globalTracer.traceInfo(moduleName, "RobotPose: %s", robot.driveBase.getAbsolutePose());

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
