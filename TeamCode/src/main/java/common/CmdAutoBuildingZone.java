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
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoBuildingZone";

    private final Robot robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;

    public CmdAutoBuildingZone(Robot robot, CommonAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
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
            robot.dashboard.displayPrintf(1, "State: Disabled");
        }
        else
        {
            double xTarget = 0.0;
            double yTarget = 0.0;

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
                    if (!autoChoices.moveFoundation)
                    {
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? -12.0 : 12.0;
                        robot.pidDrive.setTarget(xTarget, yTarget, robot.targetHeading, false, event);
                        sm.waitForSingleEvent(event, State.MOVE_TO_FOUNDATION);
                    }
                    break;

                case MOVE_TO_FOUNDATION:
                    yTarget = -38.25;
                    robot.pidDrive.setTarget(xTarget, yTarget, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.HOOK_FOUNDATION);
                    break;

                case HOOK_FOUNDATION:
                    robot.foundationLatch.grab();
                    timer.set(0.5, event);
                    sm.waitForSingleEvent(event, State.MOVE_FOUNDATION_DOWN);
                    break;

                case MOVE_FOUNDATION_DOWN:
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 12.0 : -12.0;
                    robot.pidDrive.setTarget(xTarget, yTarget, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.TURN_FOUNDATION);
                    break;

                case TURN_FOUNDATION:
                    robot.targetHeading += autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 90.0 : -90.0;
                    robot.pidDrive.setTarget(xTarget, yTarget, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.MOVE_FOUNDATION_IN);
                    break;

                case MOVE_FOUNDATION_IN:
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? -12.0 : 12.0;
                    yTarget = -47.25;
                    robot.pidDrive.setTarget(xTarget, yTarget, robot.targetHeading, false, event);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                //Move under the bridge

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
