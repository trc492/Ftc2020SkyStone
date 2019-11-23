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
import common.Robot;
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
        MOVE_TO_FOUNDATION,
        HOOK_FOUNDATION,
        MOVE_FOUNDATION_BACK,
        LET_GO_FOUNDATION,
        SCOOT_TO_LINE,
        SCOOT_CLOSER_TO_LINE,
        MOVE_CLOSER_TO_CENTER,
        SCOOT_TO_LINE_SHORT,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoBuildingZone3543";

    private final Robot robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final TrcEvent event;
    private final TrcTimer timer;
    private final TrcStateMachine<State> sm;
    private final double allianceDirection;
    private final SimplePidDrive<State> simplePidDrive;

    CmdAutoBuildingZone3543(Robot robot, CommonAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        allianceDirection = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE? 1.0: -1.0;
        simplePidDrive = new SimplePidDrive<>(robot.pidDrive, event, sm);
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
            State nextState;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case BEGIN:
                    robot.pidDrive.setAbsolutePose(new TrcPose2D(
                            RobotInfo3543.BUILDING_ZONE_ROBOT_START_X * allianceDirection, RobotInfo.ROBOT_START_Y));
                    robot.pidDrive.getXPidCtrl().saveAndSetOutputLimit(0.5);
                    robot.pidDrive.getYPidCtrl().saveAndSetOutputLimit(0.5);
                    robot.encoderXPidCtrl.setNoOscillation(true);
                    robot.encoderYPidCtrl.setNoOscillation(true);
                    robot.gyroPidCtrl.setNoOscillation(true);
                    sm.setState(State.DO_DELAY);
                    //
                    // Intentionally falling through to the next state.
                    //
                case DO_DELAY:
                    nextState = autoChoices.moveFoundation?
                                    State.MOVE_TO_FOUNDATION:
                                autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL?
                                    State.SCOOT_TO_LINE: State.SCOOT_CLOSER_TO_LINE;
                    //
                    // Do delay if any.
                    //
                    if (autoChoices.delay == 0.0)
                    {
                        sm.setState(nextState);
                    }
                    else
                    {
                        timer.set(autoChoices.delay, event);
                        sm.waitForSingleEvent(event, nextState);
                    }
                    break;

                case MOVE_TO_FOUNDATION:
                    //
                    // Robot will move backwards so that the hook is facing the foundation.
                    //
                    yTarget = -30;
                    simplePidDrive.setRelativeYTarget(yTarget, State.HOOK_FOUNDATION);
                    break;

                case HOOK_FOUNDATION:
                    //
                    // The hook latches onto the foundation.
                    //
                    robot.foundationLatch.grab(event);
                    sm.waitForSingleEvent(event, State.MOVE_FOUNDATION_BACK);
                    break;

                case MOVE_FOUNDATION_BACK:
                    yTarget = 38.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.LET_GO_FOUNDATION);
                    break;

                case LET_GO_FOUNDATION:
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL?
                                        State.SCOOT_TO_LINE: State.SCOOT_CLOSER_TO_LINE;
                    robot.foundationLatch.release(event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case SCOOT_TO_LINE:
                    xTarget = 48.0 * allianceDirection;
                    simplePidDrive.setRelativeXTarget(xTarget, State.DONE);
                    break;

                case SCOOT_CLOSER_TO_LINE:
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                            State.MOVE_CLOSER_TO_CENTER: State.DONE;
                    xTarget = 28.0 * allianceDirection;
                    simplePidDrive.setRelativeXTarget(xTarget, nextState);
                    break;

                case MOVE_CLOSER_TO_CENTER:
                    yTarget = -20.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.SCOOT_TO_LINE_SHORT);
                    break;

                case SCOOT_TO_LINE_SHORT:
                    xTarget = 20.0 * allianceDirection;
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
