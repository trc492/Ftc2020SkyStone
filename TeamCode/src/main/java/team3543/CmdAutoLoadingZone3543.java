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
import common.RobotInfo;
import common.SimplePidDrive;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class CmdAutoLoadingZone3543 implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private enum State
    {
        DO_DELAY,
        SETUP_VISION,
        MOVE_CLOSER,
        MOVE_TO_FIRST_STONE,
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

    private static final String moduleName = "CmdAutoLoadingZone3543";

    private final Robot3543 robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private final double allianceDirection;
    private final boolean useVisionTrigger;
    private final SimplePidDrive<State> simplePidDrive;
    private CmdSkystoneVision skystoneVisionCommand = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    CmdAutoLoadingZone3543(Robot3543 robot, CommonAuto.AutoChoices autoChoices)
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
    }   //CmdAutoLoadingZone3543

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
            State nextState;

            robot.dashboard.displayPrintf(1, "State: %s", state);

            switch (state)
            {
                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (autoChoices.delay == 0.0)
                    {
                        sm.setState(State.SETUP_VISION);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, State.SETUP_VISION);
                        break;
                    }

                case SETUP_VISION:
                    skystoneVisionCommand = new CmdSkystoneVision(
                            robot, autoChoices, RobotInfo3543.GRABBER_OFFSET, useVisionTrigger);
                    sm.setState(State.MOVE_CLOSER);
                    //
                    // Intentionally falling through to the next state.
                    //
                case MOVE_CLOSER:
                    //
                    // Move closer slowly to a distance so Vuforia can detect the target.
                    //
                    robot.grabber.release();
                    robot.wrist.extend();

                    robot.pidDrive.getXPidCtrl().setOutputLimit(0.5);
                    robot.pidDrive.getYPidCtrl().setOutputLimit(0.5);
                    yTarget = 23.5;
                    nextState = useVisionTrigger? State.DO_VISION: State.MOVE_TO_FIRST_STONE;
                    simplePidDrive.setRelativeYTarget(yTarget, nextState);
                    break;

                case MOVE_TO_FIRST_STONE:
                    xTarget = autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_MID?
                                0.0:
                              autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_FAR?
                                RobotInfo.LEFT_STONE_FAR_X: RobotInfo.LEFT_STONE_WALL_X;
                    if (xTarget == 0.0)
                    {
                        sm.setState(State.DO_VISION);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        xTarget *= allianceDirection;
                        simplePidDrive.setRelativeXTarget(xTarget, State.DO_VISION);
                        break;
                    }

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
                    if (robot.extenderArm != null)
                    {
                        robot.extenderArm.extend(2.5, event);
                        sm.waitForSingleEvent(event, State.GRAB_SKYSTONE);
                        break;
                    }
                    //
                    // Intentionally falling through to the next state.
                    //
                case GRAB_SKYSTONE:
                    robot.grabber.grab(2.0, event);
                    sm.waitForSingleEvent(event, State.PULL_SKYSTONE);
                    break;

                case PULL_SKYSTONE:
                    yTarget = -9.5;
                    simplePidDrive.setRelativeYTarget(yTarget, State.START_EXTENDER_ARM_RETRACTION);
                    break;

                case START_EXTENDER_ARM_RETRACTION:
                    if (robot.extenderArm != null)
                    {
                        robot.extenderArm.retract(1.0, event);
                        sm.waitForSingleEvent(event, State.GOTO_FOUNDATION);
                        break;
                    }
                    //
                    // Intentionally falling through to the next state.
                    //
                case GOTO_FOUNDATION:
                    // Need to go full speed to save time.
                    robot.pidDrive.getXPidCtrl().setOutputLimit(1.0);
                    robot.pidDrive.getYPidCtrl().setOutputLimit(1.0);
//                    xTarget = 72.0*allianceDirection - robot.driveBase.getXPosition();
//                    simplePidDrive.setRelativeXTarget(xTarget, State.APPROACH_FOUNDATION);
                    xTarget = RobotInfo.FOUNDATION_DROP_ABS_POS_X_INCHES * allianceDirection;
                    simplePidDrive.setAbsoluteXTarget(xTarget, State.APPROACH_FOUNDATION);
                    break;

                case APPROACH_FOUNDATION:
                    robot.elevator.setPosition(6.0);
                    if (robot.extenderArm != null) robot.extenderArm.extend(); //start extending early
                    yTarget = 10.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.EXTEND_ARM_OVER_FOUNDATION);
                    break;

                case EXTEND_ARM_OVER_FOUNDATION:
                    if (robot.extenderArm != null)
                    {
                        robot.extenderArm.extend(2.0, event);
                        sm.waitForSingleEvent(event, State.DROP_SKYSTONE);
                        break;
                    }
                    //
                    // Intentionally falling through to the next state.
                    //
                case DROP_SKYSTONE:
                    robot.grabber.release(1.5, event);
                    sm.waitForSingleEvent(event, State.BACK_OFF_FOUNDATION);
                    break;

                case BACK_OFF_FOUNDATION:
                    if (robot.extenderArm != null) robot.extenderArm.retract();
                    robot.wrist.retract();
                    yTarget = -6.0;
                    nextState = autoChoices.moveFoundation?
                                    State.TURN_AROUND:
                                autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                                        //CodeReview: may have to move back more before strafing to clear the bridge.
                                        State.STRAFE_TO_PARK:
                                        State.SKIP_MOVE_FOUNDATION_PARK_WALL;
                    simplePidDrive.setRelativeYTarget(yTarget, nextState);
                    break;

                case TURN_AROUND:
                    robot.elevator.setPosition(0.0);
                    turnTarget = 180.0;
                    simplePidDrive.setRelativeTurnTarget(turnTarget, State.BACKUP_TO_FOUNDATION);
                    break;

                case BACKUP_TO_FOUNDATION:
                    yTarget = -8.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.HOOK_FOUNDATION);
                    break;

                case HOOK_FOUNDATION:
                    robot.foundationLatch.grab(event);
                    sm.waitForSingleEvent(event, State.PULL_FOUNDATION_TO_WALL);
                    break;

                case PULL_FOUNDATION_TO_WALL:
                    yTarget = 50.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.UNHOOK_FOUNDATION);
                    break;

                case UNHOOK_FOUNDATION:
                    robot.foundationLatch.release(event);
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL?
                                    State.STRAFE_TO_PARK:
                                autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                                    State.MOVE_CLOSER_TO_BRIDGE: State.DONE;
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case MOVE_CLOSER_TO_BRIDGE:
//                    xTarget = 30.0 * allianceDirection;
//                    simplePidDrive.setRelativeXTarget(xTarget, State.MOVE_BACK_TO_CENTER);
                    xTarget = RobotInfo.AVOID_PARTNER_ABS_POS_X_INCHES * allianceDirection;
                    simplePidDrive.setAbsoluteXTarget(xTarget,State.MOVE_BACK_TO_CENTER);
                    break;

                case MOVE_BACK_TO_CENTER:
//                    yTarget = -20.0;
//                    simplePidDrive.setRelativeYTarget(yTarget, State.MOVE_UNDER_BRIDGE);
                    yTarget = RobotInfo.CENTER_FIELD_ABS_POS_Y_INCHES;
                    simplePidDrive.setAbsoluteYTarget(yTarget, State.MOVE_UNDER_BRIDGE);
                    break;

                case MOVE_UNDER_BRIDGE:
//                    xTarget = 20.0 * allianceDirection;
//                    simplePidDrive.setRelativeXTarget(xTarget, State.DONE);
                    xTarget = RobotInfo.ON_LINE_ABS_POS_X_INCHES * allianceDirection;
                    simplePidDrive.setAbsoluteXTarget(xTarget, State.DONE);
                    break;

                case SKIP_MOVE_FOUNDATION_PARK_WALL:
//                    yTarget = -44.0;
//                    simplePidDrive.setRelativeYTarget(yTarget, nextState);
                    robot.elevator.setPosition(0.0);
                    yTarget = RobotInfo.WALL_ABS_POS_Y_INCHES;
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL?
                                    State.STRAFE_TO_PARK: State.DONE;
                    simplePidDrive.setAbsoluteYTarget(yTarget, nextState);
                    break;

                case STRAFE_TO_PARK:
                    robot.elevator.setPosition(0.0);
//                    xTarget = -50.0 * allianceDirection;
//                    if (autoChoices.moveFoundation) xTarget = -xTarget;
//                    simplePidDrive.setRelativeXTarget(xTarget, nextState);
                    xTarget = RobotInfo.ON_LINE_ABS_POS_X_INCHES * allianceDirection;
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                                    State.MOVE_TOWARDS_CENTER: State.DONE;
                    simplePidDrive.setAbsoluteXTarget(xTarget, nextState);
                    break;

                case MOVE_TOWARDS_CENTER:
//                    yTarget = 8.0;
//                    simplePidDrive.setRelativeYTarget(yTarget, State.DONE);
                    yTarget = RobotInfo.CENTER_FIELD_ABS_POS_Y_INCHES;
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

}   //class CmdAutoLoadingZone3543
