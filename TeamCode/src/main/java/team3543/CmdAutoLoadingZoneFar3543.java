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

package team3543;

import common.CmdSkystoneVision;
import common.CommonAuto;
import common.SimplePidDrive;
import common.Robot;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class CmdAutoLoadingZoneFar3543 implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;
    private static final boolean useVisionTrigger = false;

    // Absolute position waypoint coordinates
    private static final double FOUNDATION_DROP_ABS_POS_X_INCHES = 120.0;
    private static final double WALL_ABS_POS_Y_INCHES = 9.0;
    private static final double ON_LINE_ABS_POS_X_INCHES = 69.0;
    private static final double CENTER_FIELD_ABS_POS_Y_INCHES = 35.0;
    private static final double AVOID_PARTNER_ABS_POS_X_INCHES = 89.0;

    private enum State
    {
        DO_DELAY,
        MOVE_CLOSER,
        START_VISION,
        DO_VISION,
        GO_DOWN_ON_SKYSTONE,
        GRAB_SKYSTONE,
        PULL_SKYSTONE,
        START_EXTENDER_ARM_RETRACTION,
        GOTO_FOUNDATION,
        APPROACH_FOUNDATION,
        EXTEND_ARM_OVER_FOUNDATION,
        DROP_SKYSTONE,
        BACK_OFF_FOUNDATION,
        TURN_AROUND,
        BACKUP_TO_FOUNDATION,
        HOOK_FOUNDATION,
        PULL_FOUNDATION_TO_WALL,
        UNHOOK_FOUNDATION,
        MOVE_CLOSER_TO_BRIDGE,
        MOVE_BACK_TO_CENTER,
        MOVE_UNDER_BRIDGE,
        SKIP_MOVE_FOUNDATION_PARK_WALL,
        STRAFE_TO_PARK,
        MOVE_TOWARDS_CENTER,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoLoadingZoneFar3543";

    private final Robot robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private final SimplePidDrive<State> simplePidDrive;
    private CmdSkystoneVision skystoneVisionCommand = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    public CmdAutoLoadingZoneFar3543(Robot robot, CommonAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, "robot=%s", robot);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);

        robot.encoderXPidCtrl.setNoOscillation(true);
        robot.encoderYPidCtrl.setNoOscillation(true);
        robot.gyroPidCtrl.setNoOscillation(true);

        double startX;
        double startY;
        double allianceInversion = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 1.0 : -1.0;
        startX = (autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_WALL?
                RobotInfo3543.ROBOT_START_X_WALL: RobotInfo3543.ROBOT_START_X_FAR) * allianceInversion;
        startY = (autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_WALL?
                RobotInfo3543.ROBOT_START_Y_WALL: RobotInfo3543.ROBOT_START_Y_FAR) * allianceInversion;
        simplePidDrive = new SimplePidDrive<State>(robot.pidDrive, event, sm, startX, startY);

        sm.start(State.DO_DELAY);
    }   //CmdAutoLoadingZoneFar3543

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
            robot.dashboard.displayPrintf(1, "State: Disabled or waiting...");
        }
        else
        {
            boolean traceState = true;
            double xTarget = 0.0;
            double yTarget = 0.0;
            double turnTarget = 0.0;
            State nextState = null;

            robot.dashboard.displayPrintf(1, "State: %s", state);

            switch (state)
            {
                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (autoChoices.delay == 0.0)
                    {
                        sm.setState(State.MOVE_CLOSER);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, State.MOVE_CLOSER);
                        break;
                    }

                case MOVE_CLOSER:
                    //
                    // Move closer slowly to a distance so Vuforia can detect the target.
                    //
                    robot.grabber.release();
                    robot.wrist.extend();

                    robot.pidDrive.getXPidCtrl().setOutputLimit(0.5);
                    robot.pidDrive.getYPidCtrl().setOutputLimit(0.5);
                    yTarget = 23.5;
                    simplePidDrive.setRelativeYTarget(yTarget, State.START_VISION);
                    break;

                case START_VISION:
                    skystoneVisionCommand = new CmdSkystoneVision(robot, autoChoices, useVisionTrigger);
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
                        // Skystone vision is done, continue on.
                        //
                        sm.setState(State.GO_DOWN_ON_SKYSTONE);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        traceState = false;
                        break;
                    }

                case GO_DOWN_ON_SKYSTONE:
                    robot.extenderArm.extend(2.5, event);
                    sm.waitForSingleEvent(event, State.GRAB_SKYSTONE);
                    break;

                case GRAB_SKYSTONE:
                    robot.grabber.grab(2.0, event);
                    sm.waitForSingleEvent(event, State.PULL_SKYSTONE);
                    break;

                case PULL_SKYSTONE:
                    yTarget = -9.5;
                    simplePidDrive.setRelativeYTarget(yTarget, State.START_EXTENDER_ARM_RETRACTION);
                    break;

                case START_EXTENDER_ARM_RETRACTION:
                    robot.extenderArm.retract(1.0, event);
                    sm.waitForSingleEvent(event, State.GOTO_FOUNDATION);
                    break;

                case GOTO_FOUNDATION:
                    robot.pidDrive.getXPidCtrl().setOutputLimit(1.0);
                    robot.pidDrive.getYPidCtrl().setOutputLimit(1.0);
                    //xTarget = (autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE? 72.0: -72.0)
                    //          - robot.driveBase.getXPosition();
                    //simplePidDrive.setRelativeXTarget(xTarget, State.APPROACH_FOUNDATION);
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE?
                            FOUNDATION_DROP_ABS_POS_X_INCHES : -FOUNDATION_DROP_ABS_POS_X_INCHES;
                    simplePidDrive.setAbsoluteXTarget(xTarget, State.APPROACH_FOUNDATION);
                    break;

                case APPROACH_FOUNDATION:
                    yTarget = 12.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.EXTEND_ARM_OVER_FOUNDATION);
                    break;

                case EXTEND_ARM_OVER_FOUNDATION:
                    robot.elevator.setPosition(6.0);
                    robot.extenderArm.extend(3.0, event);
                    sm.waitForSingleEvent(event, State.DROP_SKYSTONE);
                    break;

                case DROP_SKYSTONE:
                    robot.grabber.release(1.5, event);
                    sm.waitForSingleEvent(event, State.BACK_OFF_FOUNDATION);
                    break;

                case BACK_OFF_FOUNDATION:
                    robot.elevator.setPosition(0.0);
                    robot.extenderArm.retract();
                    nextState = autoChoices.moveFoundation? State.TURN_AROUND:
                                autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                                        State.STRAFE_TO_PARK:
                                        State.SKIP_MOVE_FOUNDATION_PARK_WALL;
                    yTarget = -6.0;
                    simplePidDrive.setRelativeYTarget(yTarget, nextState);
                    break;

                case TURN_AROUND:
                    turnTarget = 180.0;
                    simplePidDrive.setRelativeTurnTarget(turnTarget, State.BACKUP_TO_FOUNDATION);
                    break;

                case BACKUP_TO_FOUNDATION:
                    robot.extenderArm.retract();
                    robot.wrist.retract();
                    yTarget = -10.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.HOOK_FOUNDATION);
                    break;

                case HOOK_FOUNDATION:
                    robot.foundationLatch.grab(event);
                    sm.waitForSingleEvent(event, State.PULL_FOUNDATION_TO_WALL);
                    break;

                case PULL_FOUNDATION_TO_WALL:
                    yTarget = 46.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.UNHOOK_FOUNDATION);
                    //yTarget = WALL_ABS_POS_Y_INCHES;
                    //simplePidDrive.setAbsoluteYTarget(yTarget, State.UNHOOK_FOUNDATION);
                    break;

                case UNHOOK_FOUNDATION:
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL?
                                    State.STRAFE_TO_PARK:
                                autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                                    State.MOVE_CLOSER_TO_BRIDGE: State.DONE;
                    robot.foundationLatch.release(event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case MOVE_CLOSER_TO_BRIDGE:
                    //xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE? 30.0: -30.0;
                    //simplePidDrive.setRelativeXTarget(xTarget, State.MOVE_BACK_TO_CENTER);
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE?
                            AVOID_PARTNER_ABS_POS_X_INCHES : -AVOID_PARTNER_ABS_POS_X_INCHES;
                    simplePidDrive.setAbsoluteXTarget(xTarget,State.MOVE_BACK_TO_CENTER);
                    break;

                case MOVE_BACK_TO_CENTER:
                    //yTarget = -20.0;
                    //simplePidDrive.setRelativeYTarget(yTarget, State.MOVE_UNDER_BRIDGE);
                    yTarget = CENTER_FIELD_ABS_POS_Y_INCHES;
                    simplePidDrive.setAbsoluteYTarget(yTarget, State.MOVE_UNDER_BRIDGE);
                    break;

                case MOVE_UNDER_BRIDGE:
                    //xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE? 20.0: -20.0;
                    //simplePidDrive.setRelativeXTarget(xTarget, State.DONE);
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE?
                            ON_LINE_ABS_POS_X_INCHES : -ON_LINE_ABS_POS_X_INCHES;
                    simplePidDrive.setAbsoluteXTarget(xTarget, State.DONE);
                    break;

                case SKIP_MOVE_FOUNDATION_PARK_WALL:
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL?
                                    State.STRAFE_TO_PARK: State.DONE;
                    //yTarget = -44.0;
                    //simplePidDrive.setRelativeYTarget(yTarget, nextState);
                    yTarget = WALL_ABS_POS_Y_INCHES;
                    simplePidDrive.setAbsoluteYTarget(yTarget, nextState);
                    break;

                case STRAFE_TO_PARK:
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                                    State.MOVE_TOWARDS_CENTER: State.DONE;
                    //xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE? -50.0: 50.0;
                    //if (autoChoices.moveFoundation) xTarget = -xTarget;
                    //simplePidDrive.setRelativeXTarget(xTarget, nextState);
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE?
                            ON_LINE_ABS_POS_X_INCHES : -ON_LINE_ABS_POS_X_INCHES;
                    simplePidDrive.setAbsoluteXTarget(xTarget, nextState);
                    break;

                case MOVE_TOWARDS_CENTER:
                    //yTarget = 8.0;
                    //simplePidDrive.setRelativeYTarget(yTarget, State.DONE);
                    yTarget = CENTER_FIELD_ABS_POS_Y_INCHES;
                    simplePidDrive.setAbsoluteYTarget(yTarget, State.DONE);
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

}   //class CmdAutoLoadingZoneFar3543
