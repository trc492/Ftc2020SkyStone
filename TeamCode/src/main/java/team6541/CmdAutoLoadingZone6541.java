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

import common.CmdSkystoneVision;
import common.CommonAuto;
import common.RobotInfo;
import common.SimplePidDrive;
import trclib.TrcEvent;
import trclib.TrcPidController;
import trclib.TrcPose2D;
import trclib.TrcRobot;
import trclib.TrcStateMachine;
import trclib.TrcTimer;

class CmdAutoLoadingZone6541 implements TrcRobot.RobotCommand
{
    private enum State
    {
        BEGIN,
        START_DELAY,
        MOVE_CLOSER,
        MOVE_TO_FIRST_STONE,
        DO_VISION,
        GRAB_SKYSTONE,
        PULL_SKYSTONE,
        TURN_TOWARDS_FOUNDATION,
        GOTO_FOUNDATION,
        TURN_BACK_TO_FOUNDATION,
        APPROACH_FOUNDATION,
        DROP_SKYSTONE,
        BACK_OFF_FOUNDATION,
        TURN_AROUND,
        BACKUP_TO_FOUNDATION,
        HOOK_FOUNDATION,
        FINISH_DELAY,
        PULL_FOUNDATION_TO_WALL,
        UNHOOK_FOUNDATION,
        CLEAR_OF_FOUNDATION,
        MOVE_TO_FOUNDATION_SIDE,
        PUSH_FOUNDATION_TO_WALL,
        MOVE_BACK_TO_WALL,
        MOVE_UNDER_BRIDGE,
        SKIP_MOVE_FOUNDATION_PARK_WALL,
        MOVE_TOWARDS_CENTER,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoLoadingZone6541";

    private final Robot6541 robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final CmdSkystoneVision.Parameters visionParams;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private final CmdSkystoneVision skystoneVisionCommand;
    private final double allianceDirection;
    private final SimplePidDrive<State> simplePidDrive;
    private final TrcPidController xPidCtrl;
    private final TrcPidController yPidCtrl;
    private final TrcPidController turnPidCtrl;
    private TrcPidController.PidCoefficients savedYPidCoeff = null;
    private double startX = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    CmdAutoLoadingZone6541(Robot6541 robot, CommonAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, "robot=%s", robot);

        this.robot = robot;
        this.autoChoices = autoChoices;
        visionParams = new CmdSkystoneVision.Parameters()
                .setUseVisionTrigger(robot.preferences.useVisionTrigger)
                .setScanTowardsWall(true)
                .setScootCount(2)
                .setGrabberOffset(RobotInfo6541.GRABBER_OFFSET_X, RobotInfo6541.GRABBER_OFFSET_Y);
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        skystoneVisionCommand = new CmdSkystoneVision(robot, autoChoices, visionParams);
        allianceDirection = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 1.0 : -1.0;
        simplePidDrive = new SimplePidDrive<>(robot.pidDrive, event, sm);
        xPidCtrl = robot.pidDrive.getXPidCtrl();
        yPidCtrl = robot.pidDrive.getYPidCtrl();
        turnPidCtrl = robot.pidDrive.getTurnPidCtrl();
        sm.start(State.BEGIN);
    }   //CmdAutoLoadingZone6541

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
        robot.setFlashLightOn(false, false);
        if (robot.pidDrive.isActive())
        {
            robot.pidDrive.cancel();
        }

        xPidCtrl.setOutputLimit(1.0);
        yPidCtrl.setOutputLimit(1.0);
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
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
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
                case BEGIN:
                    robot.setFlashLightOn(true, true);
                    //
                    // Set the robot's absolute field starting position.
                    //
                    startX = autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_SINGLE_SKYSTONE?
                                    RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_WALL:
                             autoChoices.robotStartX != 0.0?
                                    autoChoices.robotStartX:
                             autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_DOUBLE_SKYSTONE_FAR?
                                    RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_FAR:
                                    RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_WALL;
                    startX *= allianceDirection;
                    robot.pidDrive.setAbsolutePose(new TrcPose2D(startX, RobotInfo.ABS_ROBOT_START_Y));
                    robot.pidDrive.setAbsoluteTargetModeEnabled(true);

                    xPidCtrl.setNoOscillation(true);
                    yPidCtrl.setNoOscillation(true);
                    turnPidCtrl.setNoOscillation(true);
                    robot.elbow.extend();
                    robot.grabber.release();

                    if (autoChoices.strategy != CommonAuto.AutoStrategy.LOADING_ZONE_SINGLE_SKYSTONE &&
                        Math.abs(startX) < RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_MID)
                    {
                        visionParams.setScootCount(1);
                    }

                    skystoneVisionCommand.start();
                    sm.setState(State.START_DELAY);
                    //
                    // Intentionally falling through to the next state.
                    //
                case START_DELAY:
                    //
                    // Do start delay if any.
                    //
                    if (autoChoices.startDelay == 0.0)
                    {
                        sm.setState(State.MOVE_CLOSER);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, State.MOVE_CLOSER);
                        break;
                    }

                case MOVE_CLOSER:
                    //
                    // Move closer slowly to a distance so Vuforia can detect the target.
                    //
                    xPidCtrl.setOutputLimit(0.5);
                    yPidCtrl.setOutputLimit(0.5);
                    yTarget = 17.5;
                    simplePidDrive.setRelativeYTarget(yTarget, State.MOVE_TO_FIRST_STONE);
                    break;

                case MOVE_TO_FIRST_STONE:
                    xTarget = (autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_SINGLE_SKYSTONE ||
                               Math.abs(startX) > RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_MID)?
                                    RobotInfo.ABS_FAR_STONE1_X: RobotInfo.ABS_WALL_STONE3_X;
                    visionParams.setScanTowardsWall(xTarget == RobotInfo.ABS_WALL_STONE3_X);
                    xTarget *= allianceDirection;
                    simplePidDrive.setAbsoluteXTarget(xTarget, State.DO_VISION);
                    break;

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
                        sm.setState(State.GRAB_SKYSTONE);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        traceState = false;
                        break;
                    }

                case GRAB_SKYSTONE:
                    // CodeReview: Can we tune this time down? 1 sec seems very long.
                    robot.grabber.grab(1.0, event);
                    sm.waitForSingleEvent(event, State.PULL_SKYSTONE);
                    break;

                case PULL_SKYSTONE:
                    nextState = autoChoices.strafeToFoundation? State.GOTO_FOUNDATION: State.TURN_TOWARDS_FOUNDATION;
                    yTarget = -11.0;
                    simplePidDrive.setRelativeYTarget(yTarget, nextState);
                    break;

                case TURN_TOWARDS_FOUNDATION:
                    turnTarget = 90.0 * allianceDirection;
                    simplePidDrive.setAbsoluteHeadingTarget(turnTarget, State.GOTO_FOUNDATION);
                    break;

                case GOTO_FOUNDATION:
                    nextState = autoChoices.strafeToFoundation?
                                    State.APPROACH_FOUNDATION: State.TURN_BACK_TO_FOUNDATION;
                    // Need to go full speed to save time.
                    xPidCtrl.setOutputLimit(1.0);
                    yPidCtrl.setOutputLimit(1.0);
                    xTarget = (autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_SINGLE_SKYSTONE?
                                    RobotInfo.ABS_FOUNDATION_DROP_MID_X:
                               autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_DOUBLE_SKYSTONE_CENTER?
                                    RobotInfo.ABS_FOUNDATION_DROP_NEAR_X: RobotInfo.ABS_FOUNDATION_DROP_FAR_X) - 3.5;
                    xTarget *= allianceDirection;
                    simplePidDrive.setAbsoluteXTarget(xTarget, nextState);
                    break;

                case TURN_BACK_TO_FOUNDATION:
                    turnTarget = 0.0;
                    simplePidDrive.setAbsoluteHeadingTarget(turnTarget, State.APPROACH_FOUNDATION);
                    break;

                case APPROACH_FOUNDATION:
                    robot.elevator.setPosition(4.0);
                    yTarget = RobotInfo.ABS_FOUNDATION_Y + 2.0;
                    simplePidDrive.setAbsoluteYTarget(yTarget, State.DROP_SKYSTONE);
                    break;

                case DROP_SKYSTONE:
                    // CodeReview: tune this time to minimum.
                    robot.grabber.release(0.5, event);
                    sm.waitForSingleEvent(event, State.BACK_OFF_FOUNDATION);
                    break;

                case BACK_OFF_FOUNDATION:
                    yTarget = -6.0;
                    nextState = autoChoices.moveFoundation?
                                    State.TURN_AROUND:
                                autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                                    State.MOVE_UNDER_BRIDGE:
                                    State.SKIP_MOVE_FOUNDATION_PARK_WALL;
                    simplePidDrive.setRelativeYTarget(yTarget, nextState);
                    break;

                case TURN_AROUND:
                    turnTarget = 180.0;
                    simplePidDrive.setRelativeTurnTarget(turnTarget, State.BACKUP_TO_FOUNDATION);
                    break;

                case BACKUP_TO_FOUNDATION:
                    robot.elevator.zeroCalibrate();
                    robot.elbow.retract();
                    robot.grabber.setPosition(1.0);
                    yTarget = -8.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.HOOK_FOUNDATION);
                    break;

                case HOOK_FOUNDATION:
                    robot.backFoundationLatch.grab(event);
                    sm.waitForSingleEvent(event, State.FINISH_DELAY);
                    break;

                case FINISH_DELAY:
                    //
                    // Do finish delay if any.
                    // This is for performing a double-skystone autonomous with our alliance partner.
                    // In this strategy, we will go first and drop the first skystone. Our partner should follow
                    // closely. In case our partner is slow, we will wait for them to finish dropping the second
                    // skystone and clear of the foundation before we pull the foundation to the wall.
                    // Our full autonomous takes about 20 seconds, so we can afford to wait up to approx. 10 seconds.
                    // But for safety margin, we should not wait more than 5-6 seconds.
                    //
                    if (autoChoices.finishDelay == 0.0)
                    {
                        sm.setState(State.PULL_FOUNDATION_TO_WALL);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        timer.set(autoChoices.finishDelay, event);
                        sm.waitForSingleEvent(event, State.PULL_FOUNDATION_TO_WALL);
                        break;
                    }

                case PULL_FOUNDATION_TO_WALL:
                    //
                    // Pulling the foundation added a lot of friction that causes the wheels to slip. As a result,
                    // the drivebase odometry will no longer be accurate. Let's drive the robot with extra distance
                    // to make sure it hits the wall and we will correct the odometry and Absolute Target Pose in the
                    // Y direction.
                    //
                    savedYPidCoeff = yPidCtrl.getPidCoefficients();
                    TrcPidController.PidCoefficients loadedYPidCoeff = savedYPidCoeff.clone();
                    loadedYPidCoeff.kP = RobotInfo6541.ENCODER_Y_LOADED_KP;
                    yPidCtrl.setPidCoefficients(loadedYPidCoeff);
                    yTarget = 32.0;
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
                    robot.backFoundationLatch.release(event);
                    sm.waitForSingleEvent(event, State.CLEAR_OF_FOUNDATION);
                    break;

                case CLEAR_OF_FOUNDATION:
                    //
                    // Move the robot clear of the foundation otherwise the foundation move won't score if the robot
                    // is still in contact with the foundation. Besides, we need to bump the foundations closer to
                    // the wall to make sure it's in the building site.
                    //
                    xTarget = (RobotInfo.ABS_NEXT_TO_PARTNER_PARK_X + 3.0) * allianceDirection;
                    simplePidDrive.setAbsoluteXTarget(xTarget, State.MOVE_TO_FOUNDATION_SIDE);
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
                    xTarget = -12.0 * allianceDirection;
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                                    State.MOVE_UNDER_BRIDGE: State.MOVE_BACK_TO_WALL;
                    simplePidDrive.setRelativeXTarget(xTarget, nextState);
                    break;

                case MOVE_BACK_TO_WALL:
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.NO_PARK?
                                    State.DONE: State.MOVE_UNDER_BRIDGE;
                    // CodeReview: This is moving back to wall, why is yTarget ABS_ROBOT_TRAVEL_Y???
                    // I suggested ABS_ROBOT_TRAVEL_Y for clearing the alliance partner's robot when traveling towards
                    // the bridge, not for moving back to the wall?!
                    yTarget = RobotInfo.ABS_ROBOT_START_Y;
                    simplePidDrive.setAbsoluteYTarget(yTarget, nextState);
//                    yTarget = 19.0;
//                    simplePidDrive.setRelativeYTarget(yTarget, nextState);
                    break;

                case MOVE_UNDER_BRIDGE:
                    if (!autoChoices.moveFoundation)
                    {
                        robot.elevator.zeroCalibrate();
                        robot.elbow.retract();
                        robot.grabber.setPosition(1.0);
                    }
                    xTarget = RobotInfo.ABS_UNDER_BRIDGE_PARK_X * allianceDirection;
                    simplePidDrive.setAbsoluteXTarget(xTarget, State.DONE);
                    break;

                case SKIP_MOVE_FOUNDATION_PARK_WALL:
                    yTarget = RobotInfo.ABS_ROBOT_START_Y;
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL?
                                    State.MOVE_UNDER_BRIDGE: State.DONE;
                    simplePidDrive.setAbsoluteYTarget(yTarget, nextState);
                    break;

                case MOVE_TOWARDS_CENTER:
                    yTarget = RobotInfo.ABS_CENTER_BRIDGE_PARK_Y;
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

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //CmdAutoLoadingZone6541
