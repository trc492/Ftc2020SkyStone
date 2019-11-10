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

import common.CommonAuto;
import common.Robot;
import trclib.TrcEnhancedPidDrive;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class CmdAutoBuildingZone6541 implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private enum State
    {
        DO_DELAY,
        DRIVE_DIRECTLY_UNDER_BRIDGE_IF_NOT_MOVING_FOUNDATION,
        CRAB_TO_CENTER_IF_NOT_MOVING_FOUNDATION,
        RAISE_ELEVATOR,
        DRIVE_UNTIL_GRABBER_ALIGNED_WITH_FOUNDATION,
        CRAB_TO_ALIGN_WITH_FOUNDATION,
        GOTO_FOUNDATION,
        HOOK_FOUNDATION,
        ROTATE_FOUNDATION_TO_CORNER,
        PUSH_FOUNDATION_TO_WALL,
        UNHOOK_FOUNDATION,
        BACK_OFF_FROM_FOUNDATION,
        LOWER_ELEVATOR_AFTER_BACKING_OFF,
        CRAB_TOWARD_WALL,
        ALIGN_WITH_BRIDGE,
        PARK_UNDER_BRIDGE_TOUCHING_FENCE,
        PARK_UNDER_BRIDGE_TOUCHING_CENTER,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoBuildingZone6541";

    private final Robot robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;
    private final TrcEnhancedPidDrive<State> enhancedPidDrive;

    public CmdAutoBuildingZone6541(Robot robot, CommonAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        enhancedPidDrive = new TrcEnhancedPidDrive<>(
                "CmdAutoBuildingZone6541", robot.driveBase, robot.pidDrive, event, sm, false);
        sm.start(State.DO_DELAY);
    }   //CmdAutoBuildingZone3543

    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    @Override
    public void cancel()
    {
        if (robot.pidDrive.isActive())
        {
            robot.pidDrive.cancel();
        }

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
            double xTarget = 0.0, yTarget = 0.0, turnTarget = 0.0;
            State nextState = null;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case DO_DELAY:
                    robot.pidDrive.getXPidCtrl().saveAndSetOutputLimit(0.5);
                    robot.pidDrive.getYPidCtrl().saveAndSetOutputLimit(0.5);
                    //
                    // are we going to move the foundation?
                    // if so, we will set the next state to raising the elevator and moving the foundation.
                    // if not, we will check if we will park under the bridge.
                    // if we will park under the bridge, then we will drive forwards directly under the bridge.
                    // if we will not park under the bridge, then we will do nothing, and go to done.
                    //
                    nextState = autoChoices.moveFoundation ?
                                        State.RAISE_ELEVATOR :
                                autoChoices.parkUnderBridge != CommonAuto.ParkPosition.NO_PARK ?
                                        State.DRIVE_DIRECTLY_UNDER_BRIDGE_IF_NOT_MOVING_FOUNDATION :
                                        State.DONE;
                    //
                    // Do delay if any.
                    //
                    if (autoChoices.delay == 0.0)
                    {
                        sm.setState(nextState);
                        break;
                    }
                    else
                    {
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, nextState);
                        break;
                    }

                case DRIVE_DIRECTLY_UNDER_BRIDGE_IF_NOT_MOVING_FOUNDATION:
                    // if not moving the foundation and parking under bridge, drive directly forwards to bridge.
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL ?
                            State.DONE :
                            State.CRAB_TO_CENTER_IF_NOT_MOVING_FOUNDATION;
                    yTarget = 24.0;
                    enhancedPidDrive.setRelativeYTarget(yTarget, nextState);
                    break;

                case CRAB_TO_CENTER_IF_NOT_MOVING_FOUNDATION:
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 28.0 : -28.0;
                    enhancedPidDrive.setRelativeXTarget(xTarget, State.DONE);
                    break;

                case RAISE_ELEVATOR:
                    // raise the elevator to prevent end effector from colliding with bridge.
                    robot.elevator.setPosition(15.0, event, 0.0);
                    sm.waitForSingleEvent(event, State.DRIVE_UNTIL_GRABBER_ALIGNED_WITH_FOUNDATION);
                    break;

                case DRIVE_UNTIL_GRABBER_ALIGNED_WITH_FOUNDATION:
                    // CodeReview: why do we drive forward instead of just setting up the robot 3 inches forward?
                    // drive forward 3 inches to prevent robot base from colliding orthogonally to foundation.
                    yTarget = 3.0;
                    enhancedPidDrive.setRelativeYTarget(yTarget, State.CRAB_TO_ALIGN_WITH_FOUNDATION);
                    break;

                case CRAB_TO_ALIGN_WITH_FOUNDATION:
                    // crab over to the foundation, and target the center of the foundation with the grabber.
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 50.0 : -50.0;
                    enhancedPidDrive.setRelativeXTarget(xTarget, State.GOTO_FOUNDATION);
                    break;

                case GOTO_FOUNDATION:
                    // drive backward 5 inches to align grabber vertically with foundation.
                    yTarget = -5.0;
                    enhancedPidDrive.setRelativeYTarget(yTarget, State.HOOK_FOUNDATION);
                    break;

                case HOOK_FOUNDATION:
                    // hook the foundation.
                    // after that, rotate a magnitude of 60 degrees to face the corner.
                    robot.foundationLatch.grab(event);
                    sm.waitForSingleEvent(event, State.ROTATE_FOUNDATION_TO_CORNER);
                    break;

                case ROTATE_FOUNDATION_TO_CORNER:
                    // rotate with an angular displacement magnitude of 60 degrees to face the building zone corner.
                    // direction of rotation will vary based on red alliance or blue alliance.
                    // if red alliance, rotate 60 degrees clockwise.
                    // if blue alliance, rotate 60 degrees anti-clockwise.
                    turnTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 60.0 : -60.0;
                    enhancedPidDrive.setRelativeTurnTarget(turnTarget, State.PUSH_FOUNDATION_TO_WALL);
                    break;

                case PUSH_FOUNDATION_TO_WALL:
                    // CodeReview: you are using timed drive here. That's why EnhancedPidDrive lost track of the
                    // robot position so subsequent EnhancedPidDrive will be off!
                    //
                    // pid drive is unreliable due to wheel slip when robot is overcoming significant kinetic friction,
                    // which is present when the robot is pushing heavy loads such as foundation.
                    // as a result, blindly drive the board into the corner for 2 seconds, applying moderate torque.
                    // afterwards, unhook the foundation and decide whether to park under the bridge.
                    robot.driveBase.holonomicDrive(0.0, -0.6, 0.0);
                    timer.set(1.0, event);
                    sm.waitForSingleEvent(event, State.UNHOOK_FOUNDATION);
                    break;

                case UNHOOK_FOUNDATION:
                    // will we park under the bridge?
                    // for both cases, we will first unhook the foundation.
                    // if we are parking under bridge, we will drive to the bridge.
                    // otherwise, we will remain at the site of the foundation until match end. (set state to done)
                    nextState = autoChoices.parkUnderBridge != CommonAuto.ParkPosition.NO_PARK ?
                            State.BACK_OFF_FROM_FOUNDATION :
                            State.DONE;
                    robot.foundationLatch.release(event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case BACK_OFF_FROM_FOUNDATION:
                    yTarget = 4.0;
                    enhancedPidDrive.setRelativeYTarget(yTarget, State.LOWER_ELEVATOR_AFTER_BACKING_OFF);
                    break;

                case LOWER_ELEVATOR_AFTER_BACKING_OFF:
                    // CodeReview: you are lowering the elevator. Do you really need to wait for it to finish?
                    robot.elevator.zeroCalibrate();
                    timer.set(2.0, event);
                    sm.waitForSingleEvent(event, State.CRAB_TOWARD_WALL);
                    break;

                case CRAB_TOWARD_WALL:
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? -48.0 : 48.0;
                    enhancedPidDrive.setRelativeXTarget(xTarget, State.ALIGN_WITH_BRIDGE);
                    break;

                case ALIGN_WITH_BRIDGE:
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL ?
                                        State.PARK_UNDER_BRIDGE_TOUCHING_FENCE:
                                autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER ?
                                        State.PARK_UNDER_BRIDGE_TOUCHING_CENTER:
                                        State.DONE;
                    turnTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 30.0 : -30.0;
                    enhancedPidDrive.setRelativeTurnTarget(turnTarget, nextState);
                    break;

                case PARK_UNDER_BRIDGE_TOUCHING_FENCE:
                    yTarget = -12.0;
                    enhancedPidDrive.setRelativeYTarget(yTarget, State.DONE);
                    break;

                case PARK_UNDER_BRIDGE_TOUCHING_CENTER:
                    yTarget = 12.0;
                    enhancedPidDrive.setRelativeYTarget(yTarget, State.DONE);
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

}   //CmdAutoBuildingZone3543
