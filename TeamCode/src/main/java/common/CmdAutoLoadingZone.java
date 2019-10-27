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

public class CmdAutoLoadingZone implements TrcRobot.RobotCommand
{
    private enum State
    {
        DO_DELAY,
        // CodeReview: you may want to go forward 12 to 15 inches before looking because I found the starting distance
        // is a little too far for reliable object recognition.
        LOOKING_FOR_SKYSTONES,
        FIRST_SKYSTONE_ALIGN_GRABBER_TO_SKYSTONE,
        // drive to skystone and pick it up
        FIRST_SKYSTONE_OPEN_GRABBER_AND_EXTEND_ARM,
        FIRST_SKYSTONE_DRIVE_FORWARD,
        FIRST_SKYSTONE_ARM_GOES_DOWN_ON_SKYSTONE,
        FIRST_SKYSTONE_GRAB_SKYSTONE,
        FIRST_SKYSTONE_EXTEND_ARM_WITH_SKYSTONE,
        // move skystone to foundation
        FIRST_SKYSTONE_BACK_UP,
        FIRST_SKYSTONE_TURN_TOWARDS_BUILDING_SIDE,
        FIRST_SKYSTONE_GO_FORWARDS,
        FIRST_SKYSTONE_TURN_TOWARD_MIDDLE,
        FIRST_SKYSTONE_MOVE_TOWARD_MIDDLE,
        FIRST_SKYSTONE_TURN_TO_FOUNDATION,
        FIRST_SKYSTONE_MOVE_TO_FOUNDATION,
        FIRST_SKYSTONE_RELEASE_SKYSTONE,
        // go back to loadingzone
        FIRST_SKYSTONE_REVERSE,
        FIRST_SKYSTONE_TURN_TOWARDS_WALL,
        FIRST_SKYSTONE_MOVE_TOWARDS_WALL,
        FIRST_SKYSTONE_TURN_TOWARDS_LOADINGZONE,
        FIRST_SKYSTONE_MOVE_TOWARDS_LOADINGZONE,

        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoLoadingZone";

    private final Robot robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;
    private final SimpleRobotMovements<State> simpleMovements;

    private double xSkyStoneOffSetPosition;

    public CmdAutoLoadingZone(Robot robot, CommonAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.DO_DELAY);
        simpleMovements = new SimpleRobotMovements<>(robot, sm, event);
    }   //CmdAutoLoadingZone

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
            robot.dashboard.displayPrintf(1, "State: %s", state);

            State nextState = null;

            switch (state)
            {

                case DO_DELAY:
                    //
                    // Do delay if any.
                    //
                    if (autoChoices.delay == 0.0)
                    {
                        sm.setState(State.LOOKING_FOR_SKYSTONES);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, State.LOOKING_FOR_SKYSTONES);
                        break;
                    }
                case LOOKING_FOR_SKYSTONES:
                    //TODO: look for the skystones
                    sm.setState(State.FIRST_SKYSTONE_OPEN_GRABBER_AND_EXTEND_ARM);
                    break;

                case FIRST_SKYSTONE_ALIGN_GRABBER_TO_SKYSTONE:
                    simpleMovements.driveSidewaysUntilDone(xSkyStoneOffSetPosition, State.FIRST_SKYSTONE_OPEN_GRABBER_AND_EXTEND_ARM);
                    break;

                case FIRST_SKYSTONE_OPEN_GRABBER_AND_EXTEND_ARM:
                    if(robot.armExtender!= null)
                    {
                        robot.armExtender.extend();
                    }

                    if(robot.grabber != null) {
                        robot.grabber.release();
                    }

                    timer.set(1.5, event);
                    sm.waitForSingleEvent(event, State.FIRST_SKYSTONE_DRIVE_FORWARD);
                    break;

                case FIRST_SKYSTONE_DRIVE_FORWARD:
                    simpleMovements.driveStraightUntilDone(27, State.FIRST_SKYSTONE_ARM_GOES_DOWN_ON_SKYSTONE);
                    break;

                case FIRST_SKYSTONE_ARM_GOES_DOWN_ON_SKYSTONE:
                    nextState = State.FIRST_SKYSTONE_GRAB_SKYSTONE;

                    if(robot.armExtender != null)
                    {
                        robot.armExtender.retract(event);
                        sm.waitForSingleEvent(event, nextState);
                    } else {
                        sm.setState(nextState);
                    }

                    break;

                case FIRST_SKYSTONE_GRAB_SKYSTONE:
                    nextState = State.FIRST_SKYSTONE_EXTEND_ARM_WITH_SKYSTONE;

                    if(robot.grabber != null) {
                        robot.grabber.grab(event);
                        sm.waitForSingleEvent(event, nextState);
                    } else {
                        sm.setState(nextState);
                    }

                    break;

                case FIRST_SKYSTONE_EXTEND_ARM_WITH_SKYSTONE:
                    nextState = State.FIRST_SKYSTONE_BACK_UP;
                    if (robot.armExtender != null) {
                        robot.armExtender.extend(event);
                        sm.waitForSingleEvent(event, nextState);
                    } else {
                        sm.setState(nextState);
                    }
                    break;

                case FIRST_SKYSTONE_BACK_UP:
                    simpleMovements.driveStraightUntilDone(-7, State.FIRST_SKYSTONE_TURN_TOWARDS_BUILDING_SIDE);
                    break;

                case FIRST_SKYSTONE_TURN_TOWARDS_BUILDING_SIDE:
                    int turnDegrees = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE?90:-90;
                    simpleMovements.turnInPlaceUntilDone(turnDegrees, State.FIRST_SKYSTONE_GO_FORWARDS);
                    break;

                case FIRST_SKYSTONE_GO_FORWARDS:
                    simpleMovements.driveStraightUntilDone(81, State.FIRST_SKYSTONE_TURN_TOWARD_MIDDLE);
                    break;

                case FIRST_SKYSTONE_TURN_TOWARD_MIDDLE:
                    turnDegrees = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE?-90:90;
                    simpleMovements.turnInPlaceUntilDone(turnDegrees, State.FIRST_SKYSTONE_MOVE_TOWARD_MIDDLE);
                    break;

                case FIRST_SKYSTONE_MOVE_TOWARD_MIDDLE:
                    simpleMovements.driveStraightUntilDone(28, State.FIRST_SKYSTONE_TURN_TO_FOUNDATION);
                    break;

                case FIRST_SKYSTONE_TURN_TO_FOUNDATION:
                    turnDegrees = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE?90:-90;
                    simpleMovements.turnInPlaceUntilDone(turnDegrees, State.FIRST_SKYSTONE_MOVE_TO_FOUNDATION);
                    break;

                case FIRST_SKYSTONE_MOVE_TO_FOUNDATION:
                    simpleMovements.driveStraightUntilDone(5.5,State.FIRST_SKYSTONE_RELEASE_SKYSTONE);
                    break;

                case FIRST_SKYSTONE_RELEASE_SKYSTONE:
                    nextState = State.FIRST_SKYSTONE_REVERSE;
                    if (robot.grabber != null) {
                        robot.grabber.release(event);
                        sm.waitForSingleEvent(event, nextState);
                    } else {
                        sm.setState(nextState);
                    }
                    break;

                case FIRST_SKYSTONE_REVERSE:
                    simpleMovements.driveStraightUntilDone(-5.5, State.FIRST_SKYSTONE_TURN_TOWARDS_WALL);
                    break;

                case FIRST_SKYSTONE_TURN_TOWARDS_WALL:
                    turnDegrees = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE?90:-90;
                    simpleMovements.turnInPlaceUntilDone(turnDegrees, State.FIRST_SKYSTONE_MOVE_TOWARDS_WALL);
                    break;

                case FIRST_SKYSTONE_MOVE_TOWARDS_WALL:
                    simpleMovements.driveStraightUntilDone(28, State.FIRST_SKYSTONE_TURN_TOWARDS_LOADINGZONE);
                    break;

                case FIRST_SKYSTONE_TURN_TOWARDS_LOADINGZONE:
                    turnDegrees = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE?90:-90;
                    simpleMovements.turnInPlaceUntilDone(turnDegrees, State.FIRST_SKYSTONE_MOVE_TOWARDS_LOADINGZONE);
                    break;

                case FIRST_SKYSTONE_MOVE_TOWARDS_LOADINGZONE:
                    // Line up with skystones closer to center of the field
                    simpleMovements.driveStraightUntilDone(54, State.DONE);
                    break;

                default:
                    //
                    // We are done.
                    //
                    sm.stop();
                    break;
            }

            // TODO: Declare variables for xDistance and yDistance values, set them,
            // and pass in the values here
            robot.traceStateInfo(elapsedTime, state.toString(), 0, 0, robot.targetHeading);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //CmdAutoLoadingZone
