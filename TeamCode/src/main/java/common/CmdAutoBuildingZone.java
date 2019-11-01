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
        RAISE_ELEVATOR_6541,
        STRAFE_TO_FOUNDATION_6541,
        ADVANCE_TO_FOUNDATION_6541,
        HOOK_FOUNDATION_6541,
        STRAFE_TO_WALL_6541,
        PUSH_FOUNDATION_IN_6541,
        RELEASE_FOUNDATION_6541,
        DRIVE_BACK_TO_LINE_6541,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoBuildingZone";

    private final Robot robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;
    private final SimpleRobotMovements<State> simpleMovements;

    public CmdAutoBuildingZone(Robot robot, CommonAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
        simpleMovements = new SimpleRobotMovements<>(robot, sm, event);
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
                            State.MOVE_TO_FOUNDATION : State.RAISE_ELEVATOR_6541;
                    if (autoChoices.delay == 0.0)
                    {
                        sm.setState(nextState);
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
                    simpleMovements.driveStraightUntilDone(yTarget, State.HOOK_FOUNDATION);
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
                    yTarget = 36.0;
                    simpleMovements.driveStraightUntilDone(yTarget, State.LET_GO_FOUNDATION);
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
                    simpleMovements.driveSidewaysUntilDone(xTarget, State.DONE);
                    break;

                case MOVE_FOUNDATION_DOWN:
                    // It moves the foundation down, with the robot moving sideways
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 8.0 : -8.0;
                    simpleMovements.driveSidewaysUntilDone(xTarget, State.TURN_FOUNDATION);
                    break;

                case TURN_FOUNDATION:
                    // The robot turns the foundation sideways
                    double deltaHeading = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 90.0 : -90.0;
                    simpleMovements.turnInPlaceUntilDone(deltaHeading, State.MOVE_FOUNDATION_IN);
                    break;

                case MOVE_FOUNDATION_IN:
                    // The robot pushes the foundation into the corner
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? -19.0 : 19.0;
                    yTarget = -12.0;
                    simpleMovements.driveDiagonallyUntilDone(xTarget, yTarget, State.UNHOOK_FOUNDATION);
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
                    simpleMovements.driveStraightUntilDone(yTarget, State.SCOOT_TO_SIDE);
                    break;

                case SCOOT_TO_SIDE:
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 12.0 : -12.0;
                    simpleMovements.driveSidewaysUntilDone(xTarget, State.DONE);
                    break;

                case NOT_FOUNDATION_TURN:
                    deltaHeading = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 90.0 : -90.0;
                    simpleMovements.turnInPlaceUntilDone(deltaHeading, State.NOT_FOUNDATION_MOVE);
                    break;

                case NOT_FOUNDATION_MOVE:
                    yTarget = 5;
                    simpleMovements.driveStraightUntilDone(yTarget, state.DONE);
                    break;

                case RAISE_ELEVATOR_6541:
                    robot.elevator.setPosition(12.0, event, 0.0);
                    sm.waitForSingleEvent(event, State.STRAFE_TO_FOUNDATION_6541);
                    break;

                case STRAFE_TO_FOUNDATION_6541:
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 52.0 : -52.0;
                    simpleMovements.driveSidewaysUntilDone(xTarget, State.ADVANCE_TO_FOUNDATION_6541);
                    break;

                case ADVANCE_TO_FOUNDATION_6541:
                    yTarget = -6.0;
                    simpleMovements.driveStraightUntilDone(yTarget, State.HOOK_FOUNDATION_6541);
                    break;

                case HOOK_FOUNDATION_6541:
                    if(robot.foundationLatch != null) {
                        robot.foundationLatch.grab(event);
                        sm.waitForSingleEvent(event, State.STRAFE_TO_WALL_6541);
                    } else {
                        sm.setState(State.STRAFE_TO_WALL_6541);
                    }
                    break;

//                case CRAWL_BACK_6541:
//                    yTarget = 3.0;
//                    simpleMovements.driveStraightUntilDone(yTarget, State.STRAFE_TO_WALL_6541);
//                    break;
//
                case STRAFE_TO_WALL_6541:
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? -52.0 : 52.0;
                    simpleMovements.driveSidewaysUntilDone(xTarget, State.PUSH_FOUNDATION_IN_6541);
                    break;

                case PUSH_FOUNDATION_IN_6541:
                    yTarget = -6.0;
                    simpleMovements.driveStraightUntilDone(yTarget, State.RELEASE_FOUNDATION_6541);
                    break;

                case RELEASE_FOUNDATION_6541:
                    if(robot.foundationLatch != null) {
                        robot.foundationLatch.release(event);
                        sm.waitForSingleEvent(event, State.DRIVE_BACK_TO_LINE_6541);
                    } else {
                        sm.setState(State.DRIVE_BACK_TO_LINE_6541);
                    }
                    break;

                case DRIVE_BACK_TO_LINE_6541:
                    yTarget = 24.0;
                    simpleMovements.driveSidewaysUntilDone(yTarget, State.DONE);
                    break;

                case DONE:
                default:
                    // We are done.
                    //
                    robot.pidDrive.getXPidCtrl().restoreOutputLimit();
                    robot.pidDrive.getYPidCtrl().restoreOutputLimit();
                    sm.stop();
                    break;
            }

            robot.traceStateInfo(elapsedTime, state.toString(), xTarget, yTarget, robot.targetHeading);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //CmdAutoBuildingZone
