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

import common.CommonAuto;
import common.RobotInfo;
import common.SimplePidDrive;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPose2D;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdAutoBuildingZone3543 implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private enum State
    {
        BEGIN,
        DO_DELAY,
        SCOOT_CLOSER_TO_LINE,
        MOVE_CLOSER_TO_CENTER,
        GOTO_FOUNDATION,
        HOOK_FOUNDATION,
        PULL_FOUNDATION_TO_WALL,
        UNHOOK_FOUNDATION,
        CLEAR_OF_FOUNDATION,
        MOVE_TO_FOUNDATION_SIDE,
        PUSH_FOUNDATION_TO_WALL,
        MOVE_BACK_TO_WALL,
        MOVE_UNDER_BRIDGE,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoBuildingZone3543";

    private final Robot3543 robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;
    private final double allianceDirection;
    private final SimplePidDrive<State> simplePidDrive;
    private final TrcPidController xPidCtrl;
    private final TrcPidController yPidCtrl;
    private final TrcPidController turnPidCtrl;
    private TrcPidController.PidCoefficients savedYPidCoeff = null;

    CmdAutoBuildingZone3543(Robot3543 robot, CommonAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        allianceDirection = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE? 1.0: -1.0;
        simplePidDrive = new SimplePidDrive<>(robot.pidDrive, event, sm);
        xPidCtrl = robot.pidDrive.getXPidCtrl();
        yPidCtrl = robot.pidDrive.getYPidCtrl();
        turnPidCtrl = robot.pidDrive.getTurnPidCtrl();
        sm.start(State.BEGIN);
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

        xPidCtrl.restoreOutputLimit();
        yPidCtrl.restoreOutputLimit();
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
            State nextState;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case BEGIN:
                    robot.pidDrive.setAbsolutePose(new TrcPose2D(
                            RobotInfo.ABS_BUILDING_ZONE_ROBOT_START_X * allianceDirection,
                            RobotInfo.ABS_ROBOT_START_Y));
                    xPidCtrl.saveAndSetOutputLimit(0.5);
                    yPidCtrl.saveAndSetOutputLimit(0.5);
                    xPidCtrl.setNoOscillation(true);
                    yPidCtrl.setNoOscillation(true);
                    turnPidCtrl.setNoOscillation(true);
                    sm.setState(State.DO_DELAY);
                    //
                    // Intentionally falling through to the next state.
                    //
                case DO_DELAY:
                    nextState = autoChoices.moveFoundation ?
                                    State.GOTO_FOUNDATION :
                                autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL ?
                                    State.MOVE_UNDER_BRIDGE :
                                autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER ?
                                    State.SCOOT_CLOSER_TO_LINE: State.DONE;
                    //
                    // Do delay if any.
                    //
                    if (autoChoices.startDelay == 0.0)
                    {
                        sm.setState(nextState);
                    }
                    else
                    {
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, nextState);
                    }
                    break;

                case SCOOT_CLOSER_TO_LINE:
                    xTarget = RobotInfo.ABS_NEXT_TO_PARTNER_PARK_X * allianceDirection;
                    simplePidDrive.setAbsoluteXTarget(xTarget,State.MOVE_CLOSER_TO_CENTER);
                    break;

                case MOVE_CLOSER_TO_CENTER:
                    yTarget = RobotInfo.ABS_CENTER_BRIDGE_PARK_Y;
                    simplePidDrive.setAbsoluteYTarget(yTarget, State.MOVE_UNDER_BRIDGE);
                    break;

                case GOTO_FOUNDATION:
                    yTarget = 30;
                    simplePidDrive.setRelativeYTarget(yTarget, State.HOOK_FOUNDATION);
                    break;

                case HOOK_FOUNDATION:
                    //
                    // The hook latches onto the foundation.
                    //
                    robot.frontFoundationLatch.grab(event);
                    sm.waitForSingleEvent(event, State.PULL_FOUNDATION_TO_WALL);
                    break;

                case PULL_FOUNDATION_TO_WALL:
                    //
                    // Pulling the foundation added a lot of friction that causes the wheels to slip. As a result,
                    // the drivebase odometry will no longer be accurate. Let's drive the robot with extra distance
                    // to make sure it hits the wall and we will correct the odometry and Absolute Target Pose in the
                    // Y direction.
                    //
                    savedYPidCoeff = yPidCtrl.getPidCoefficients();
                    TrcPidController.PidCoefficients loadedYPidCoeff = savedYPidCoeff.clone();
                    loadedYPidCoeff.kP = RobotInfo3543.ENCODER_Y_LOADED_KP;
                    yPidCtrl.setPidCoefficients(loadedYPidCoeff);
                    yTarget = -40.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.UNHOOK_FOUNDATION);
                    break;

                case UNHOOK_FOUNDATION:
                    if (savedYPidCoeff != null)
                    {
                        yPidCtrl.setPidCoefficients(savedYPidCoeff);
                        savedYPidCoeff = null;
                    }
                    // Correct odometry and absTargetPose after wheel slippage.
                    TrcPose2D pose = robot.driveBase.getAbsolutePose();
                    pose.y = RobotInfo.ABS_ROBOT_START_Y;
                    robot.driveBase.setAbsolutePose(pose);
                    pose = robot.pidDrive.getAbsoluteTargetPose();
                    pose.y = RobotInfo.ABS_ROBOT_START_Y;
                    robot.pidDrive.setAbsoluteTargetPose(pose);
                    //
                    // Release the foundation and continue.
                    //
                    robot.frontFoundationLatch.release(event);
                    sm.waitForSingleEvent(event, State.CLEAR_OF_FOUNDATION);
                    break;

                case CLEAR_OF_FOUNDATION:
                    //
                    // Move the robot clear of the foundation otherwise the foundation move won't score if the robot
                    // is still in contact with the foundation. Besides, we need to bump the foundations closer to
                    // the wall to make sure it's in the building site.
                    //
                    xTarget = RobotInfo.ABS_NEXT_TO_PARTNER_PARK_X * allianceDirection;
                    simplePidDrive.setAbsoluteXTarget(xTarget,State.MOVE_TO_FOUNDATION_SIDE);
                    break;

                case MOVE_TO_FOUNDATION_SIDE:
                    //
                    // Move to the side of the foundation so we can bump it to the wall.
                    //
                    yTarget = RobotInfo.ABS_CENTER_BRIDGE_PARK_Y;
                    simplePidDrive.setAbsoluteYTarget(yTarget, State.PUSH_FOUNDATION_TO_WALL);
                    break;

                case PUSH_FOUNDATION_TO_WALL:
                    //
                    // Bump the foundation toward the wall to make sure it lands inside the building site.
                    //
                    xTarget = 16.0 * allianceDirection;
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                            State.MOVE_UNDER_BRIDGE: State.MOVE_BACK_TO_WALL;
                    simplePidDrive.setRelativeXTarget(xTarget, nextState);
                    break;

                case MOVE_BACK_TO_WALL:
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.NO_PARK?
                            State.DONE: State.MOVE_UNDER_BRIDGE;
                    yTarget = RobotInfo.ABS_ROBOT_START_Y;
                    simplePidDrive.setAbsoluteYTarget(yTarget, nextState);
                    break;

                case MOVE_UNDER_BRIDGE:
                    xTarget = RobotInfo.ABS_UNDER_BRIDGE_PARK_X * allianceDirection;
                    simplePidDrive.setAbsoluteXTarget(xTarget, State.DONE);
                    break;

                case DONE:
                default:
                    //
                    // We are done.
                    //
                    cancel();
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

            robot.globalTracer.traceInfo(moduleName, "RobotPose: %s", robot.driveBase.getAbsolutePose());

            if (debugXPid && xPidCtrl != null)
            {
                xPidCtrl.printPidInfo(robot.globalTracer, elapsedTime);
            }

            if (debugYPid)
            {
                yPidCtrl.printPidInfo(robot.globalTracer, elapsedTime);
            }

            if (debugTurnPid)
            {
                turnPidCtrl.printPidInfo(robot.globalTracer, elapsedTime);
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //CmdAutoBuildingZone3543
