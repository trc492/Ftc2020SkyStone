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

import trclib.TrcAbsTargetDrive;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPose2D;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTrigger;
import trclib.TrcUtil;

public class CmdAutoLoadingZone2 implements TrcRobot.RobotCommand
{
    private static final boolean useVisionTrigger = false;
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;
    private static final double VISION_TIMEOUT = 1.0;

    private enum State
    {
        MOVE_CLOSER,
        SETUP_VISION,
        GET_TARGET_POSE,
        SCAN_FOR_SKYSTONE,
        NEXT_SKYSTONE_POSITION,
        ALIGN_SKYSTONE,
        GOTO_SKYSTONE,
        GRAB_SKYSTONE,
        PULL_SKYSTONE,
        GOTO_FOUNDATION,
        APPROACH_FOUNDATION,
        DROP_SKYSTONE,
        BACK_OFF_FOUNDATION,
        TURN_AROUND,
        BACKUP_TO_FOUNDATION,
        HOOK_FOUNDATION,
        PULL_FOUNDATION_TO_WALL,
        UNHOOK_FOUNDATION,
        PARK_UNDER_BRIDGE,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoLoadingZone2";

    private final Robot robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private final TrcAbsTargetDrive<State> absTargetDrive;
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
    public CmdAutoLoadingZone2(Robot robot, CommonAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, "robot=%s", robot);

        this.robot = robot;
        this.autoChoices = autoChoices;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        absTargetDrive = new TrcAbsTargetDrive<>(
                "SkyStoneDrive", robot.driveBase, robot.pidDrive, event, sm);
        if (useVisionTrigger)
        {
            visionTrigger = new TrcTrigger("VisionTrigger", this::isTriggered, this::targetDetected);
        }
        sm.start(State.MOVE_CLOSER);
    }   //CmdAutoLoadingZone2

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

        if (visionTrigger != null)
        {
            visionTrigger.setEnabled(false);
        }

        robot.pidDrive.getXPidCtrl().restoreOutputLimit();
        robot.pidDrive.getYPidCtrl().restoreOutputLimit();
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
            double turnTarget = 0.0;

            robot.dashboard.displayPrintf(1, "State: %s", state);

            switch (state)
            {
                case MOVE_CLOSER:
                    //
                    // Move closer slowly for a distance so Vuforia can detect the target.
                    //
                    robot.extenderArm.extend();
                    robot.wrist.setPosition(0.0);
                    robot.grabber.release();
                    robot.pidDrive.getXPidCtrl().saveAndSetOutputLimit(0.5);
                    robot.pidDrive.getYPidCtrl().saveAndSetOutputLimit(0.5);
                    yTarget = 22.0;
                    absTargetDrive.setYTarget(yTarget, State.SETUP_VISION);
                    break;

                case SETUP_VISION:
                    visionTimeout = TrcUtil.getCurrentTime() + VISION_TIMEOUT;
                    sm.setState(State.GET_TARGET_POSE);
                    break;

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
                        }
                    }
                    else
                    {
                        // Vuforia found the skystone.
                        robot.globalTracer.traceInfo(
                                "getTargetPose", "Skystone found at x=%.1f, y=%.1f.",
                                skystonePose.x, skystonePose.y);
                        sm.setState(State.ALIGN_SKYSTONE);
                    }
                    break;

                case SCAN_FOR_SKYSTONE:
                    visionTrigger.setEnabled(true);
                    scanningForSkyStone = true;
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE? -16.0: 16.0;
                    absTargetDrive.setXTarget(xTarget, State.SETUP_VISION);
                    break;

                case NEXT_SKYSTONE_POSITION:
                    if (scootCount > 0)
                    {
                        scootCount--;
                        xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE? -8.0: 8.0;
                        absTargetDrive.setXTarget(xTarget, State.SETUP_VISION);
                    }
                    else
                    {
                        // Still can't detect the target. Just assume the one in front is a skystone. We will still
                        // get some points even if we are wrong, better than nothing!
                        robot.globalTracer.traceInfo(
                                "NextSkyStonePos", "Skystone not found, giving up.");
                        sm.setState(State.GOTO_SKYSTONE);
                    }
                    break;

                case ALIGN_SKYSTONE:
                    if (visionTrigger != null)
                    {
                        visionTrigger.setEnabled(false);
                    }
                    xTarget = skystonePose.x;
                    absTargetDrive.setXTarget(xTarget, State.GOTO_SKYSTONE);
                    break;

                case GOTO_SKYSTONE:
                    if (visionTrigger != null)
                    {
                        visionTrigger.setEnabled(false);
                    }
                    // If we did not detect the skystone, assume it's right in front of us.
                    yTarget = 8.0;
                    absTargetDrive.setYTarget(yTarget, State.GRAB_SKYSTONE);
                    break;

                case GRAB_SKYSTONE:
                    robot.extenderArm.retract(event);
                    sm.waitForSingleEvent(event, State.PULL_SKYSTONE);
                    break;

                case PULL_SKYSTONE:
                    robot.grabber.grab();
                    yTarget = -12.0;
                    absTargetDrive.setYTarget(yTarget, State.GOTO_FOUNDATION);
                    break;

                case GOTO_FOUNDATION:
                    robot.extenderArm.extend();
                    robot.pidDrive.getXPidCtrl().restoreOutputLimit();
                    robot.pidDrive.getYPidCtrl().restoreOutputLimit();
                    xTarget = (autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE? 72.0: -72.0)
                              - robot.driveBase.getXPosition();
                    absTargetDrive.setXTarget(xTarget, State.APPROACH_FOUNDATION);
                    break;

                case APPROACH_FOUNDATION:
                    robot.pidDrive.getXPidCtrl().saveAndSetOutputLimit(0.5);
                    robot.pidDrive.getYPidCtrl().saveAndSetOutputLimit(0.5);
                    yTarget = 12.0;
                    absTargetDrive.setYTarget(yTarget, State.DROP_SKYSTONE);
                    break;

                case DROP_SKYSTONE:
                    robot.grabber.release(event);
                    sm.waitForSingleEvent(event, State.BACK_OFF_FOUNDATION);
                    break;

                case BACK_OFF_FOUNDATION:
                    robot.extenderArm.retract();
                    yTarget = -6.0;
                    absTargetDrive.setYTarget(yTarget, State.TURN_AROUND);
                    break;

                case TURN_AROUND:
                    robot.targetHeading = turnTarget = 180.0;
                    absTargetDrive.setTurnTarget(turnTarget, State.BACKUP_TO_FOUNDATION);
                    break;

                case BACKUP_TO_FOUNDATION:
                    yTarget = -6.0;
                    absTargetDrive.setYTarget(yTarget, State.HOOK_FOUNDATION);
                    break;

                case HOOK_FOUNDATION:
                    robot.foundationLatch.grab(event);
                    sm.waitForSingleEvent(event, State.PULL_FOUNDATION_TO_WALL);
                    break;

                case PULL_FOUNDATION_TO_WALL:
                    yTarget = 36.0;
                    absTargetDrive.setYTarget(yTarget, State.UNHOOK_FOUNDATION);
                    break;

                case UNHOOK_FOUNDATION:
                    robot.foundationLatch.release(event);
                    sm.waitForSingleEvent(event, State.PARK_UNDER_BRIDGE);
                    break;

                case PARK_UNDER_BRIDGE:
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE? 48.0: -48.0;
                    absTargetDrive.setXTarget(xTarget, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    robot.pidDrive.getXPidCtrl().restoreOutputLimit();
                    robot.pidDrive.getYPidCtrl().restoreOutputLimit();
                    sm.stop();
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

}   //class CmdAutoLoadingZone2
