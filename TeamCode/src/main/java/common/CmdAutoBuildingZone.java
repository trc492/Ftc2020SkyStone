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
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class CmdAutoBuildingZone implements TrcRobot.RobotCommand
{
    private static final boolean useSimpleRoute = true;

    private enum State
    {
        DO_DELAY,
        //MOVE_UP,
        MOVE_TO_FOUNDATION,
        HOOK_FOUNDATION,
        MOVE_FOUNDATION_BACK,
        LET_GO_FOUNDATION,
        SCOOT_TO_LINE,
        MOVE_FOUNDATION_DOWN,
        TURN_FOUNDATION,
        MOVE_FOUNDATION_IN,
        UNHOOK_FOUNDATION,
        MOVE_TO_LINE,
        SCOOT_TO_SIDE,
        NOT_FOUNDATION_TURN,
        NOT_FOUNDATION_MOVE,
        DECIDE_INITIAL_ROUTE_6541,
        DRIVE_DIRECTLY_UNDER_BRIDGE_IF_NOT_MOVING_FOUNDATION_6541,
        RAISE_ELEVATOR_6541,
        DRIVE_UNTIL_GRABBER_ALIGNED_WITH_FOUNDATION_6541,
        CRAB_TO_ALIGN_WITH_FOUNDATION_6541,
        GO_FORWARD_A_BIT_MORE_6541,
        HOOK_FOUNDATION_6541,
        ROTATE_FOUNDATION_TO_CORNER_6541,
        PUSH_FOUNDATION_TO_WALL_6541,
        UNHOOK_FOUNDATION_6541,
        BACK_OFF_FROM_FOUNDATION_6541,
        LOWER_ELEVATOR_AFTER_BACKING_OFF_6541,
        CRAB_TOWARD_WALL_6541,
        ALIGN_WITH_BRIDGE_6541,
        PARK_UNDER_BRIDGE_6541,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoBuildingZone";

    private final Robot robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;
    private final TrcAbsTargetDrive<State> absTargetDrive;

    public CmdAutoBuildingZone(Robot robot, CommonAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
        absTargetDrive = new TrcAbsTargetDrive<>(
                "CmdAutoBuildingZone", robot.driveBase, robot.pidDrive, event, sm);
    }   //CmdAutoBuildingZone

    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    @Override
    public void cancel()
    {
        robot.pidDrive.getXPidCtrl().restoreOutputLimit();
        robot.pidDrive.getYPidCtrl().restoreOutputLimit();
        sm.stop();
    }   //cancel

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
            State nextState = null;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    robot.pidDrive.getXPidCtrl().saveAndSetOutputLimit(0.5);
                    robot.pidDrive.getYPidCtrl().saveAndSetOutputLimit(0.5);

                    nextState = robot.preferences.get("team3543") ?
                            State.MOVE_TO_FOUNDATION :
                            State.DECIDE_INITIAL_ROUTE_6541;

                    if (autoChoices.delay == 0.0)
                    {
                        sm.setState(nextState);
                        break;
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, nextState);
                        break;
                    }

//                case MOVE_UP:
//                    // Robot will move sideways toward the foundation
//                    if (!autoChoices.moveFoundation)
//                    {
//                        sm.setState(State.DONE);
//                    }
//                    else
//                    {
//                        xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? -12.0 : 12.0;
//                        simpleMovements.driveSidewaysUntilDone(xTarget, State.MOVE_TO_FOUNDATION);
//                    }
//                    break;

                case MOVE_TO_FOUNDATION:
                    // Robot will move backwards so that the hook is facing the foundation
                    yTarget = -30;
                    absTargetDrive.setYTarget(yTarget, State.HOOK_FOUNDATION);
                    break;

                case HOOK_FOUNDATION:
                    // The hook latches onto the foundation
                    nextState = useSimpleRoute ? State.MOVE_FOUNDATION_BACK : State.MOVE_FOUNDATION_DOWN;

                    if(robot.foundationLatch != null) {
                        robot.foundationLatch.grab(event);
                        sm.waitForSingleEvent(event, nextState);
                    } else {
                        sm.setState(nextState);
                    }
                    break;

                case MOVE_FOUNDATION_BACK:
                    yTarget = 38.0;
                    absTargetDrive.setYTarget(yTarget, State.LET_GO_FOUNDATION);
                    break;

                case LET_GO_FOUNDATION:
                    if(robot.foundationLatch != null)
                    {
                        robot.foundationLatch.release(event);
                        sm.waitForSingleEvent(event, State.SCOOT_TO_LINE);
                    }
                    else
                    {
                        sm.setState(State.SCOOT_TO_LINE);
                    }
                    break;

                case SCOOT_TO_LINE:
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 48.0 : -48.0;
                    absTargetDrive.setXTarget(xTarget, State.DONE);
                    break;

                case MOVE_FOUNDATION_DOWN:
                    // It moves the foundation down, with the robot moving sideways
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 8.0 : -8.0;
                    absTargetDrive.setXTarget(xTarget, State.TURN_FOUNDATION);
                    break;

                case TURN_FOUNDATION:
                    // The robot turns the foundation sideways
                    turnTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 90.0 : -90.0;
                    absTargetDrive.setTurnTarget(turnTarget, State.MOVE_FOUNDATION_IN);
                    break;

                case MOVE_FOUNDATION_IN:
                    // The robot pushes the foundation into the corner
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? -19.0 : 19.0;
                    yTarget = -12.0;
                    absTargetDrive.setXYTarget(xTarget, yTarget, State.UNHOOK_FOUNDATION);
                    break;

                case UNHOOK_FOUNDATION:
                    // The hook releases the foundation
                    nextState = State.MOVE_TO_LINE;

                    if(robot.foundationLatch != null) {
                        robot.foundationLatch.release(event);
                        sm.waitForSingleEvent(event, nextState);
                    } else {
                        sm.setState(nextState);
                    }
                    break;

                case MOVE_TO_LINE:
                    // The robot drives forward into the line
                    yTarget = 44;
                    absTargetDrive.setYTarget(yTarget, State.SCOOT_TO_SIDE);
                    break;

                case SCOOT_TO_SIDE:
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 12.0 : -12.0;
                    absTargetDrive.setXTarget(xTarget, State.DONE);
                    break;

                case NOT_FOUNDATION_TURN:
                    turnTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 90.0 : -90.0;
                    absTargetDrive.setTurnTarget(turnTarget, State.NOT_FOUNDATION_MOVE);
                    break;

                case NOT_FOUNDATION_MOVE:
                    yTarget = 5;
                    absTargetDrive.setYTarget(yTarget, state.DONE);
                    break;

                case DECIDE_INITIAL_ROUTE_6541:
                    // are we going to move the foundation?
                    // if so, we will set the next state to raising the elevator and moving the foundation.
                    // if not so, we will check if we will park under the bridge.
                    // if we will park under the bridge, then we will drive forwards directly under the bridge.
                    // if we will not park under the bridge, then we will do nothing, and go to done.
                    nextState = autoChoices.moveFoundation ?
                            State.RAISE_ELEVATOR_6541 :
                            (autoChoices.parkUnderBridge ?
                                    State.DRIVE_DIRECTLY_UNDER_BRIDGE_IF_NOT_MOVING_FOUNDATION_6541 :
                                    State.DONE);
                    sm.setState(nextState);
                    break;

                case DRIVE_DIRECTLY_UNDER_BRIDGE_IF_NOT_MOVING_FOUNDATION_6541:
                    // if not moving the foundation and parking under bridge, drive directly forwards to bridge.
                    yTarget = 24.0;
                    absTargetDrive.setYTarget(yTarget, State.DONE);
                    break;

                case RAISE_ELEVATOR_6541:
                    // raise the elevator to prevent end effector from colliding with bridge.
                    robot.elevator.setPosition(15.0, event, 0.0);
                    sm.waitForSingleEvent(event, State.DRIVE_UNTIL_GRABBER_ALIGNED_WITH_FOUNDATION_6541);
                    break;

                case DRIVE_UNTIL_GRABBER_ALIGNED_WITH_FOUNDATION_6541:
                    // drive forward 3 inches to prevent robot base from colliding orthogonally to foundation.
                    yTarget = 3.0;
                    absTargetDrive.setYTarget(yTarget, State.CRAB_TO_ALIGN_WITH_FOUNDATION_6541);
                    break;

                case CRAB_TO_ALIGN_WITH_FOUNDATION_6541:
                    // crab over to the foundation, and target the center of the foundation with the grabber.
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 50.0 : -50.0;
                    absTargetDrive.setXTarget(xTarget, State.GO_FORWARD_A_BIT_MORE_6541);
                    break;

                case GO_FORWARD_A_BIT_MORE_6541:
                    // drive backward 5 inches to align grabber vertically with foundation.
                    yTarget = -5.0;
                    absTargetDrive.setYTarget(yTarget, State.HOOK_FOUNDATION_6541);
                    break;

                case HOOK_FOUNDATION_6541:
                    // hook the foundation.
                    // after that, rotate a magnitude of 60 degrees to face the corner.
                    nextState = State.ROTATE_FOUNDATION_TO_CORNER_6541;
                    if (robot.foundationLatch != null)
                    {
                        robot.foundationLatch.grab(event);
                        sm.waitForSingleEvent(event, nextState);
                    }
                    else
                    {
                        sm.setState(nextState);
                    }
                    break;

                case ROTATE_FOUNDATION_TO_CORNER_6541:
                    // rotate with an angular displacement magnitude of 60 degrees to face the building zone corner.
                    // direction of rotation will vary based on red alliance or blue alliance.
                    // if red alliance, rotate 60 degrees clockwise.
                    // if blue alliance, rotate 60 degrees anti-clockwise.
                    turnTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 60.0 : -60.0;
                    absTargetDrive.setTurnTarget(turnTarget, State.PUSH_FOUNDATION_TO_WALL_6541);
                    break;

                case PUSH_FOUNDATION_TO_WALL_6541:
                    // pid drive is unreliable due to wheel slip when robot is overcoming significant kinetic friction,
                    // which is present when the robot is pushing heavy loads such as foundation.
                    // as a result, blindly drive the board into the corner for 2 seconds, applying moderate torque.
                    // afterwards, unhook the foundation and decide whether to park under the bridge.
                    robot.driveBase.holonomicDrive(0.0, -0.6, 0.0);
                    timer.set(1.0, event);
                    sm.waitForSingleEvent(event, State.UNHOOK_FOUNDATION_6541);
                    break;

                case UNHOOK_FOUNDATION_6541:
                    // will we park under the bridge?
                    // for both cases, we will first unhook the foundation.
                    // if we are parking under bridge, we will drive to the bridge.
                    // otherwise, we will remain at the site of the foundation until match end. (set state to done)
                    nextState = autoChoices.parkUnderBridge ?
                            State.BACK_OFF_FROM_FOUNDATION_6541 :
                            State.DONE;
                    if(robot.foundationLatch != null)
                    {
                        robot.foundationLatch.release(event);
                        sm.waitForSingleEvent(event, nextState);
                    }
                    else
                    {
                        sm.setState(nextState);
                    }
                    break;

                case BACK_OFF_FROM_FOUNDATION_6541:
                    yTarget = 4.0;
                    absTargetDrive.setYTarget(yTarget, State.LOWER_ELEVATOR_AFTER_BACKING_OFF_6541);
                    break;

                case LOWER_ELEVATOR_AFTER_BACKING_OFF_6541:
                    robot.elevator.zeroCalibrate();
                    timer.set(2.0, event);
                    sm.waitForSingleEvent(event, State.CRAB_TOWARD_WALL_6541);
                    break;

                case CRAB_TOWARD_WALL_6541:
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? -48.0 : 48.0;
                    absTargetDrive.setXTarget(xTarget, State.ALIGN_WITH_BRIDGE_6541);
                    break;

                case ALIGN_WITH_BRIDGE_6541:
                    turnTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 30.0 : -30.0;
                    absTargetDrive.setTurnTarget(turnTarget, State.PARK_UNDER_BRIDGE_6541);
                    break;

                case PARK_UNDER_BRIDGE_6541:
                    yTarget = -12.0;
                    absTargetDrive.setYTarget(yTarget, State.DONE);
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

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //CmdAutoBuildingZone
