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
import common.Robot;
import trclib.TrcEnhancedPidDrive;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

public class CmdAutoBuildingZone3543 implements TrcRobot.RobotCommand
{
    private static final boolean debugXPid = true;
    private static final boolean debugYPid = true;
    private static final boolean debugTurnPid = true;

    private enum State
    {
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
    private final TrcEnhancedPidDrive<State> enhancedPidDrive;

    public CmdAutoBuildingZone3543(Robot robot, CommonAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;
        event = new TrcEvent(moduleName);
        timer = new TrcTimer(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        enhancedPidDrive = new TrcEnhancedPidDrive<>(
                "CmdAutoBuildingZone3543", robot.driveBase, robot.pidDrive, event, sm);
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

                    nextState = autoChoices.moveFoundation?
                                    State.MOVE_TO_FOUNDATION:
                                autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL?
                                    State.SCOOT_TO_LINE:
                                autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                                    State.SCOOT_CLOSER_TO_LINE: State.DONE;
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
                    enhancedPidDrive.setRelativeYTarget(yTarget, State.HOOK_FOUNDATION);
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
                    enhancedPidDrive.setRelativeYTarget(yTarget, State.LET_GO_FOUNDATION);
                    break;

                case LET_GO_FOUNDATION:
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL?
                                        State.SCOOT_TO_LINE:
                                autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                                        State.SCOOT_CLOSER_TO_LINE: State.DONE;
                    robot.foundationLatch.release(event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case SCOOT_TO_LINE:
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 48.0 : -48.0;
                    enhancedPidDrive.setRelativeXTarget(xTarget, State.DONE);
                    break;

                case SCOOT_CLOSER_TO_LINE:
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 28.0 : -28.0;
                    enhancedPidDrive.setRelativeXTarget(xTarget, State.MOVE_CLOSER_TO_CENTER);
                    break;

                case MOVE_CLOSER_TO_CENTER:
                    yTarget = -20.0;
                    enhancedPidDrive.setRelativeYTarget(yTarget, State.SCOOT_TO_LINE_SHORT);
                    break;

                case SCOOT_TO_LINE_SHORT:
                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 20.0 : -20.0;
                    enhancedPidDrive.setRelativeXTarget(xTarget, State.DONE);
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
