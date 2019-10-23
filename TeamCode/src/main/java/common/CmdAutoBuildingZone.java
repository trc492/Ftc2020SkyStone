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
    private enum State
    {
        /**
         * Delay if necessary
         * Determine whether we are moving the foundation or not
         * If yes,
         * Move to the foundation
         * Align ourselves
         * Grab onto the foundation
         * Move towards the building site
         * Unlatch
         * Move backwards
         * Stop when you go over the middle line
         * If no,
         * Wait, and then move
         * Stop when you go over the middle line
         */
        DO_DELAY,
        MOVE_UP,
        MOVE_TO_FOUNDATION,
        HOOK_FOUNDATION,
        MOVE_FOUNDATION_DOWN,
        TURN_FOUNDATION,
        MOVE_FOUNDATION_IN,
        UNHOOK_FOUNDATION,
        MOVE_TO_LINE,
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
                    if (autoChoices.delay == 0.0)
                    {
                        sm.setState(State.MOVE_UP);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, State.MOVE_UP);
                        break;
                    }

                case MOVE_UP:
                    // Robot will move sideways toward the foundation
                    if (!autoChoices.moveFoundation)
                    {
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? -12.0 : 12.0;
                        simpleMovements.driveSidewaysUntilDone(xTarget, State.MOVE_TO_FOUNDATION);
                    }
                    break;

                case MOVE_TO_FOUNDATION:
                    // Robot will move backwards so that the hook is facing the foundation
                    yTarget = -32.5;
                    simpleMovements.driveStraightUntilDone(yTarget, State.HOOK_FOUNDATION);
                    break;

                case HOOK_FOUNDATION:
                    // The hook latches onto the foundation

                    nextState = State.MOVE_FOUNDATION_DOWN;

                    if(robot.foundationLatch != null) {
                        robot.foundationLatch.grab();
                        timer.set(0.5, event);
                        sm.waitForSingleEvent(event, nextState);
                    } else {
                        sm.setState(nextState);
                    }
                    break;

                case MOVE_FOUNDATION_DOWN:
                    // It moves the foundation down, with the robot moving sideways
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 12.0 : -12.0;
                    simpleMovements.driveSidewaysUntilDone(xTarget, State.TURN_FOUNDATION);
                    break;

                case TURN_FOUNDATION:
                    // The robot turns the foundation sideways
                    double deltaHeading = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 90.0 : -90.0;
                    simpleMovements.turnInPlaceUntilDone(deltaHeading, State.MOVE_FOUNDATION_IN);
                    break;

                case MOVE_FOUNDATION_IN:
                    // The robot pushes the foundation into the corner
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? -12.0 : 12.0;
                    yTarget = -47.25;
                    simpleMovements.driveDiagonallyUntilDone(xTarget, yTarget, State.UNHOOK_FOUNDATION);
                    break;

                case UNHOOK_FOUNDATION:
                    // The hook releases the foundation
                    nextState = State.MOVE_TO_LINE;

                    if(robot.foundationLatch != null) {
                        robot.foundationLatch.release();
                        timer.set(0.5, event);
                        sm.waitForSingleEvent(event, nextState);
                    } else {
                        sm.setState(nextState);
                    }
                    break;

                case MOVE_TO_LINE:
                    // The robot drives forward into the line
                    yTarget = 50;
                    simpleMovements.driveStraightUntilDone(yTarget, State.DONE);
                    break;

                case DONE:
                default:
                    // We are done.
                    //
                    sm.stop();
                    break;
            }

            robot.traceStateInfo(elapsedTime, state.toString(), xTarget, yTarget, robot.targetHeading);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //CmdAutoBuildingZone
