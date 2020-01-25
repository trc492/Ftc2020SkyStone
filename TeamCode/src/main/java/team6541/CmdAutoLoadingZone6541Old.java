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

class CmdAutoLoadingZone6541Old implements TrcRobot.RobotCommand
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
        ALIGN_FOUNDATION_FURTHER_IF_BLUE,
        BACKUP_TO_FOUNDATION,
        HOOK_FOUNDATION,
        FINISH_DELAY,
        PULL_FOUNDATION_TO_WALL,
        UNHOOK_FOUNDATION,
        CLEAR_OF_FOUNDATION,
        MOVE_TO_FOUNDATION_SIDE,
        PUSH_FOUNDATION_TO_WALL,
        RESYNC_ROBOT_X,
        MOVE_UNDER_BRIDGE,
        MOVE_TOWARDS_CENTER,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoLoadingZone6541Old";

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
//    private TrcPidController.PidCoefficients savedYPidCoeff = null;
    private double startX = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    CmdAutoLoadingZone6541Old(Robot6541 robot, CommonAuto.AutoChoices autoChoices)
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
    }   //CmdAutoLoadingZone6541Old

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

                    /*
                    startX = autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_SINGLE_SKYSTONE?
                                    RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_WALL:
                             autoChoices.robotStartX != 0.0?
                                    autoChoices.robotStartX:
                             autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_DOUBLE_SKYSTONE_FAR?
                                    RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_FAR:
                                    RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_WALL;
                     */

                    // move to area where all visible.
                    visionParams.setScootCount(0);
                    visionParams.setAssumeLeftIfNotFound(true);
                    startX = (RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_MID * allianceDirection) - RobotInfo6541.GRABBER_OFFSET_X;

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
                    simplePidDrive.setRelativeYTarget(yTarget, State.DO_VISION);
                    break;

                case MOVE_TO_FIRST_STONE:
                    //
                    // The robot start position may be anywhere so we need to strafe to the first stone.
                    //
                    xTarget = (autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_SINGLE_SKYSTONE ||
                               Math.abs(startX) > RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_MID)?
                                    RobotInfo.ABS_FAR_STONE1_X: RobotInfo.ABS_WALL_STONE3_X;
                    visionParams.setScanTowardsWall(xTarget == RobotInfo.ABS_WALL_STONE3_X);
                    xTarget *= allianceDirection;
                    xTarget += RobotInfo6541.GRABBER_OFFSET_X;
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
                    robot.grabber.grab(1.0, event);
                    sm.waitForSingleEvent(event, State.PULL_SKYSTONE);
                    break;

                case PULL_SKYSTONE:
                    //
                    // Pull the skystone out to the travel Y position so we don't hit the bridge.
                    //
                    nextState = autoChoices.strafeToFoundation? State.GOTO_FOUNDATION: State.TURN_TOWARDS_FOUNDATION;
                    yTarget = RobotInfo.ABS_ROBOT_TRAVEL_Y - 2.0;
                    simplePidDrive.setAbsoluteYTarget(yTarget, nextState);
                    break;

                case TURN_TOWARDS_FOUNDATION:
                    //
                    // If for some reason strafing is crappy, we will turn, run in Y and turn back.
                    //
                    turnTarget = 90.0 * allianceDirection;
                    simplePidDrive.setAbsoluteHeadingTarget(turnTarget, State.GOTO_FOUNDATION);
                    break;

                case GOTO_FOUNDATION:
                    //
                    // Travel long distance to the foundation. So we need to travel in full speed to save time.
                    // Since we are using absolute target, PidDrive will figure out whether it will strafe or
                    // travel in Y according to the robot's heading.
                    //
                    xPidCtrl.setOutputLimit(1.0);
                    yPidCtrl.setOutputLimit(1.0);
                    nextState = autoChoices.strafeToFoundation?
                                    State.APPROACH_FOUNDATION: State.TURN_BACK_TO_FOUNDATION;
                    xTarget = autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_SINGLE_SKYSTONE?
                                    RobotInfo.ABS_FOUNDATION_DROP_MID_X:
                              autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_DOUBLE_SKYSTONE_CENTER?
                                    RobotInfo.ABS_FOUNDATION_DROP_NEAR_X: RobotInfo.ABS_FOUNDATION_DROP_FAR_X;
                    xTarget *= allianceDirection;
                    simplePidDrive.setAbsoluteXTarget(xTarget, nextState);
                    break;

                case TURN_BACK_TO_FOUNDATION:
                    //
                    // We were traveling in the Y direction, so turn back to face the foundation.
                    //
                    robot.elevator.setPosition(4.0);
                    turnTarget = 0.0;
                    simplePidDrive.setAbsoluteHeadingTarget(turnTarget, State.APPROACH_FOUNDATION);
                    break;

                case APPROACH_FOUNDATION:
                    //
                    // Raise the elevator to drop position while approaching the foundation.
                    //
                    xPidCtrl.setOutputLimit(0.5); // don't push the foundation too forcefully.
                    yPidCtrl.setOutputLimit(0.5);
                    robot.elevator.setPosition(3.0);
                    yTarget = RobotInfo.ABS_FOUNDATION_Y + 2.0;
                    simplePidDrive.setAbsoluteYTarget(yTarget, State.DROP_SKYSTONE);
                    break;

                case DROP_SKYSTONE:
                    //
                    // Drop off the skystone.
                    //
                    xPidCtrl.setOutputLimit(1.0);
                    yPidCtrl.setOutputLimit(1.0);
                    robot.grabber.release(0.5, event);
                    sm.waitForSingleEvent(event, State.BACK_OFF_FOUNDATION);
                    break;

                case BACK_OFF_FOUNDATION:
                    if (autoChoices.moveFoundation)
                    {
                        //
                        // The foundation grabber is in the back, so we need to back off the foundation to give us
                        // room to turn around if we are going to pull the foundation.
                        //
                        yTarget = -6.0;
                        simplePidDrive.setRelativeYTarget(yTarget, State.TURN_AROUND);
                    }
                    else
                    {
                        if (autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER)
                        {
                            //
                            // We are going to park close to the center. So go to the appropriate Y position before
                            // we can slide under the bridge.
                            //
                            yTarget = RobotInfo.ABS_CENTER_BRIDGE_PARK_Y;
                            nextState = State.MOVE_UNDER_BRIDGE;
                        }
                        else
                        {
                            //
                            // We are either parking under the bridge closer to the wall or not park at all.
                            // Regardless, let's go back to the wall.
                            //
                            yTarget = RobotInfo.ABS_ROBOT_START_Y;
                            nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL ?
                                            State.MOVE_UNDER_BRIDGE : State.DONE;
                        }
                        simplePidDrive.setAbsoluteYTarget(yTarget, nextState);
                    }
                    break;

                case TURN_AROUND:
                    //
                    // The foundation grabber is in the back, so turn around.
                    //
                    turnTarget = 180.0;
                    simplePidDrive.setRelativeTurnTarget(turnTarget, autoChoices.alliance == CommonAuto.Alliance.BLUE_ALLIANCE ?
                            State.ALIGN_FOUNDATION_FURTHER_IF_BLUE :
                            State.BACKUP_TO_FOUNDATION);
                    break;

                case ALIGN_FOUNDATION_FURTHER_IF_BLUE:
                    xTarget = 8.0;
                    simplePidDrive.setRelativeXTarget(xTarget, State.BACKUP_TO_FOUNDATION);
                    break;

                case BACKUP_TO_FOUNDATION:
                    //
                    // Move closer to the foundation so we can grab it.
                    //
                    robot.elevator.zeroCalibrate();
                    robot.elbow.retract();
                    robot.grabber.setPosition(1.0);
                    yTarget = -8.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.HOOK_FOUNDATION);
                    break;

                case HOOK_FOUNDATION:
                    //
                    // Grab the foundation.
                    //
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
//                    // To compensate for the higher friction while pulling the foundation, we will use a stronger set
//                    // of PID constants and restore the original set after we are done pulling.
//                    //
//                    savedYPidCoeff = yPidCtrl.getPidCoefficients();
//                    TrcPidController.PidCoefficients loadedYPidCoeff = savedYPidCoeff.clone();
//                    loadedYPidCoeff.kP = RobotInfo6541.ENCODER_Y_LOADED_KP;
//                    yPidCtrl.setPidCoefficients(loadedYPidCoeff);
//                    //
//                    // Pull the foundation a little further to make sure we hit the wall because the wheels may have
//                    // slipped and the Y odometry could be off.
//                    //
//                    yTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ?
//                                RobotInfo.ABS_ROBOT_START_Y : RobotInfo.ABS_ROBOT_START_Y - 6.0;
//                    simplePidDrive.setAbsoluteYTarget(yTarget, State.UNHOOK_FOUNDATION);
                    robot.driveBase.holonomicDrive(0.0, 1.0, 0.0);
                    timer.set(1.0, event);
                    sm.waitForSingleEvent(event, State.UNHOOK_FOUNDATION);
                    break;

                case UNHOOK_FOUNDATION:
//                    //
//                    // We are done pulling the heavy foundation, restore the Y PID constants to the original set.
//                    //
//                    if (savedYPidCoeff != null)
//                    {
//                        yPidCtrl.setPidCoefficients(savedYPidCoeff);
//                        savedYPidCoeff = null;
//                    }
                    //
                    // Pulling the heavy foundation to the wall may cause the wheels to slip so we need to correct
                    // the Y odometry and Y absolute target pose.
                    //
                    robot.driveBase.stop();
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
                    TrcPose2D pose = robot.driveBase.getFieldPosition();
                    pose.y = RobotInfo.ABS_ROBOT_START_Y;
                    robot.driveBase.setFieldPosition(pose);
                    pose = robot.pidDrive.getAbsoluteTargetPose();
                    pose.y = RobotInfo.ABS_ROBOT_START_Y;
                    robot.pidDrive.setAbsoluteTargetPose(pose);

                    xTarget = RobotInfo.ABS_NEXT_TO_PARTNER_PARK_X * allianceDirection;
                    yTarget = RobotInfo.ABS_ROBOT_START_Y;
                    simplePidDrive.setAbsoluteXYTarget(xTarget, yTarget, State.MOVE_TO_FOUNDATION_SIDE);
                    break;

                case MOVE_TO_FOUNDATION_SIDE:
                    //
                    // Move to the side of the foundation so we can bump it to the wall.
                    //
                    if (!autoChoices.moveFoundation)
                    {
                        robot.elevator.zeroCalibrate();
                        robot.elbow.retract();
                        robot.grabber.setPosition(1.0);
                    }
                    yTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ?
                                    RobotInfo.ABS_CENTER_BRIDGE_PARK_Y + 4.0 : RobotInfo.ABS_CENTER_BRIDGE_PARK_Y;
                    simplePidDrive.setAbsoluteYTarget(yTarget, State.PUSH_FOUNDATION_TO_WALL);
                    break;

                case PUSH_FOUNDATION_TO_WALL:
                    //
                    // Bump the foundation toward the wall to make sure it lands inside the building site.
                    //
//                    xTarget = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? -20.0 : 6.0;
//                    simplePidDrive.setRelativeXTarget(xTarget, State.RESYNC_ROBOT_X);
//                    sm.waitForSingleEvent(event, State.RESYNC_ROBOT_X);
                    robot.driveBase.holonomicDrive(-1.0 * allianceDirection, 0.0 ,0.0);
                    timer.set(1.0, event);
                    sm.waitForSingleEvent(event, State.RESYNC_ROBOT_X);
                    break;

                case RESYNC_ROBOT_X:
                    //
                    // Bumping the foundation back to the side wall gives us an opportunity to resync the robot's
                    // X odometry because pulling the heavy foundation especially when pulling it lopsided will
                    // throw the odometry off. By bumping the foundation to the side wall, we know where we are
                    // exactly in the X direction. So let's correct the X odometry accordingly.
                    //
                    robot.driveBase.stop();
                    pose = robot.driveBase.getFieldPosition();
                    pose.x = RobotInfo.ABS_FOUNDATION_SIDE_X * allianceDirection;
                    robot.driveBase.setFieldPosition(pose);

                    pose = robot.pidDrive.getAbsoluteTargetPose();
                    pose.x = RobotInfo.ABS_FOUNDATION_SIDE_X * allianceDirection;
                    robot.pidDrive.setAbsoluteTargetPose(pose);

                    if (autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER)
                    {
                        xTarget = RobotInfo.ABS_UNDER_BRIDGE_PARK_X * allianceDirection;
                        simplePidDrive.setAbsoluteXTarget(xTarget, State.DONE);
                    }
                    else
                    {
                        yTarget = RobotInfo.ABS_ROBOT_START_Y;
                        nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL ?
                                        State.MOVE_UNDER_BRIDGE : State.DONE;
                        simplePidDrive.setAbsoluteYTarget(yTarget, nextState);
                    }
                    break;

                case MOVE_UNDER_BRIDGE:
                    //
                    // Slide under the bridge.
                    //
                    xTarget = RobotInfo.ABS_UNDER_BRIDGE_PARK_X * allianceDirection;
                    simplePidDrive.setAbsoluteXTarget(xTarget, State.DONE);
                    break;

                case MOVE_TOWARDS_CENTER:
                    //
                    // Move the robot toward the bridge so we won't hit our alliance partner's robot by the wall.
                    //
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
                robot.traceStateInfo(sm.getState());
            }
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //CmdAutoLoadingZone6541Old
