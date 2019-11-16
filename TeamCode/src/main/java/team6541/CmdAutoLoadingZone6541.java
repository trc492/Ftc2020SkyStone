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

package team6541;

import common.CmdSkystoneVision;
import common.CommonAuto;
import common.RobotInfo;
import common.SimplePidDrive;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

// CodeReview: This code need to be revised to be compatible with the new grabber.
public class CmdAutoLoadingZone6541 implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private enum State
    {
        DO_DELAY,
        MOVE_CLOSER,
        ELEVATOR_TO_RELEASE_HEIGHT,
        ELEVATOR_TO_PICKUP_HEIGHT,
        START_VISION,
        DO_VISION,
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

    private static final String moduleName = "CmdAutoLoadingZone6541";

    private final Robot6541 robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private final SimplePidDrive<State> simplePidDrive;
    private final double allianceDirection;
    private final boolean useVisionTrigger;
    private CmdSkystoneVision skystoneVisionCommand = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    public CmdAutoLoadingZone6541(Robot6541 robot, CommonAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, "robot=%s", robot);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        allianceDirection = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 1.0 : -1.0;
        useVisionTrigger = robot.preferences.useVisionTrigger;

        robot.encoderXPidCtrl.setNoOscillation(true);
        robot.encoderYPidCtrl.setNoOscillation(true);
        robot.gyroPidCtrl.setNoOscillation(true);

        double startX = (autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_WALL?
                RobotInfo.ROBOT_START_X_WALL: RobotInfo.ROBOT_START_X_FAR)* allianceDirection;
        double startY = (autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_WALL?
                RobotInfo.ROBOT_START_Y_WALL: RobotInfo.ROBOT_START_Y_FAR)* allianceDirection;
        simplePidDrive = new SimplePidDrive<>(robot.pidDrive, event, sm, startX, startY);

        sm.start(State.DO_DELAY);
    }   //CmdAutoLoadingZone6541

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

        robot.pidDrive.getXPidCtrl().setOutputLimit(1.0);
        robot.pidDrive.getYPidCtrl().setOutputLimit(1.0);
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
            boolean traceState = true;
            double xTarget = 0.0;
            double yTarget = 0.0;
            double turnTarget = 0.0;

            robot.dashboard.displayPrintf(1, "State: %s", state);

            switch (state)
            {
                case DO_DELAY:
                    robot.elevator.setPosition(RobotInfo6541.ELEVATOR_RELEASE_HEIGHT);
                    robot.elbow.extend();
                    //
                    // Do delay if any.
                    //
                    if (autoChoices.delay == 0.0)
                    {
                        sm.setState(State.MOVE_CLOSER);
                        //
                        // Intentionally falling through to the next state.
                        //
                    } else
                    {
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, State.MOVE_CLOSER);
                        break;
                    }

                case MOVE_CLOSER:
                    //
                    // Move closer slowly for a distance so Vuforia can detect the target.
                    //
                    robot.grabber.release();
                    robot.pidDrive.getXPidCtrl().setOutputLimit(0.5);
                    robot.pidDrive.getYPidCtrl().setOutputLimit(0.5);
                    yTarget = 22.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.ELEVATOR_TO_RELEASE_HEIGHT);
                    break;

                case ELEVATOR_TO_RELEASE_HEIGHT:
                    robot.elevator.setPosition(RobotInfo6541.ELEVATOR_RELEASE_HEIGHT, event, 0);
                    sm.waitForSingleEvent(event, State.ELEVATOR_TO_PICKUP_HEIGHT);
                    break;

                case ELEVATOR_TO_PICKUP_HEIGHT:
                    robot.elevator.setPosition(RobotInfo6541.ELEVATOR_PICKUP_HEIGHT, event, 0);
                    sm.waitForSingleEvent(event, State.START_VISION);
                    break;

                case START_VISION:
                    skystoneVisionCommand = new CmdSkystoneVision(
                            robot, autoChoices, RobotInfo6541.GRABBER_OFFSET, useVisionTrigger);
                    sm.setState(State.DO_VISION);
                    //
                    // Intentionally falling through to the next state.
                    //
                case DO_VISION:
                    //
                    // Do vision to detect and go to the skystone. If vision did not detect skystone, it will stop
                    // at the last stone.
                    //
                    if (skystoneVisionCommand.cmdPeriodic(elapsedTime))
                    {
                        //
                        // Skystone vision is done. Sync our absolute target pose with the last robot position from
                        // skystone vision and continue.
                        //
                        sm.setState(State.GRAB_SKYSTONE);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        traceState = false;
                        break;
                    }

                case GRAB_SKYSTONE:
                    robot.grabber.grab(1.5, event);
                    sm.waitForSingleEvent(event, State.PULL_SKYSTONE);
                    break;

                case PULL_SKYSTONE:
                    yTarget = -12.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.GOTO_FOUNDATION);
                    break;

                case GOTO_FOUNDATION:
                    robot.pidDrive.getXPidCtrl().setOutputLimit(1.0);
                    robot.pidDrive.getYPidCtrl().setOutputLimit(1.0);
                    xTarget = 72.0 * allianceDirection - robot.driveBase.getXPosition();
                    simplePidDrive.setRelativeXTarget(xTarget, State.APPROACH_FOUNDATION);
                    break;

                case APPROACH_FOUNDATION:
                    yTarget = 15.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.DROP_SKYSTONE);
                    break;

                case DROP_SKYSTONE:
                    robot.grabber.release();
                    timer.set(1.5, event);
                    sm.waitForSingleEvent(event, State.BACK_OFF_FOUNDATION);
                    break;

                case BACK_OFF_FOUNDATION:
                    yTarget = -6.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.TURN_AROUND);
                    break;

                case TURN_AROUND:
                    turnTarget = 180.0;
                    simplePidDrive.setRelativeTurnTarget(turnTarget, State.BACKUP_TO_FOUNDATION);
                    break;

                case BACKUP_TO_FOUNDATION:
                    yTarget = -10.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.HOOK_FOUNDATION);
                    break;

                case HOOK_FOUNDATION:
                    robot.foundationLatch.grab(event);
                    sm.waitForSingleEvent(event, State.PULL_FOUNDATION_TO_WALL);
                    break;

                case PULL_FOUNDATION_TO_WALL:
                    yTarget = 48;
                    simplePidDrive.setRelativeYTarget(yTarget, State.UNHOOK_FOUNDATION);
                    break;

                case UNHOOK_FOUNDATION:
                    robot.foundationLatch.release(event);
                    sm.waitForSingleEvent(event, State.PARK_UNDER_BRIDGE);
                    break;

                case PARK_UNDER_BRIDGE:
                    xTarget = 50.0 * allianceDirection;
                    simplePidDrive.setRelativeXTarget(xTarget, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    cancel();
                    break;
            }

            if (traceState)
            {
                robot.traceStateInfo(elapsedTime, state.toString(), xTarget, yTarget, turnTarget);
            }
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

}   //CmdAutoLoadingZone6541
