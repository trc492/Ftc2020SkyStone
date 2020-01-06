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

class CmdAutoLoadingZone3543 implements TrcRobot.RobotCommand
{
    private static final double AUTONOMOUS_END_TIME = 100.0;//30.0; //TODO: set it back to 30 sec when done debugging.
    private static final int MAX_SCORE_SINGLE_SKYSTONE = 29;

    private enum State
    {
        BEGIN,
        START_DELAY,
        MOVE_CLOSER,
        MOVE_TO_FIRST_STONE,
        SETUP_VISION,
        DO_VISION,
        GO_DOWN_ON_SKYSTONE,
        PULL_SKYSTONE,
        START_EXTENDER_ARM_RETRACTION,
        TURN_TOWARDS_FOUNDATION,
        GOTO_FOUNDATION,
        TURN_BACK_TO_FOUNDATION,
        APPROACH_FOUNDATION,
        DROP_SKYSTONE,
        BACK_OFF_FOUNDATION,
        TURN_TOWARDS_QUARRY,
        GOTO_SECOND_SKYSTONE,
        TURN_TO_SECOND_SKYSTONE,
        FINISH_DELAY,
        PULL_FOUNDATION_TO_WALL,
        UNHOOK_FOUNDATION,
        CLEAR_OF_FOUNDATION,
        MOVE_TO_FOUNDATION_SIDE,
        PUSH_FOUNDATION_TO_WALL,
        RESYNC_ROBOT_X,
        MOVE_BACK_TO_WALL,
        MOVE_UNDER_BRIDGE,
        SKIP_MOVE_FOUNDATION_PARK_WALL,
        MOVE_TOWARDS_CENTER,
        START_RETRACTIONS,
        DONE
    }   //enum State

    private static final String moduleName = "CmdAutoLoadingZone3543";

    private final Robot3543 robot;
    private final CommonAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private final CmdSkystoneVision.Parameters visionParams;
    private final CmdSkystoneVision skystoneVisionCommand;
    private final double allianceDirection;
    private final SimplePidDrive<State> simplePidDrive;
    private final TrcPidController xPidCtrl;
    private final TrcPidController yPidCtrl;
    private final TrcPidController turnPidCtrl;
    private TrcPidController.PidCoefficients savedYPidCoeff = null;
    private double startX = 0.0;
    private int skystonesDropped = 0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     */
    CmdAutoLoadingZone3543(Robot3543 robot, CommonAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s", robot);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        visionParams = new CmdSkystoneVision.Parameters()
                .setUseVisionTrigger(robot.preferences.useVisionTrigger)
                .setAssumeLeftIfNotFound(false)
                .setScanTowardsWall(true)
                .setScootCount(2)
                .setGrabberOffset(RobotInfo3543.GRABBER_OFFSET_X, RobotInfo3543.GRABBER_OFFSET_Y);
        skystoneVisionCommand = new CmdSkystoneVision(robot, autoChoices, visionParams);
        allianceDirection = autoChoices.alliance == CommonAuto.Alliance.RED_ALLIANCE ? 1.0 : -1.0;
        simplePidDrive = new SimplePidDrive<>(robot.pidDrive, event, sm);
        xPidCtrl = robot.pidDrive.getXPidCtrl();
        yPidCtrl = robot.pidDrive.getYPidCtrl();
        turnPidCtrl = robot.pidDrive.getTurnPidCtrl();
        sm.start(State.BEGIN);
    }   //CmdAutoLoadingZone3543

    private boolean doSecondSkystone(double elapsedTime)
    {
        final String funcName = "doSecondSkystone";
        boolean willDo = false;
        //
        // We will only do this if we are doing DOUBLE_SKYSTONE and we have already dropped off the first skystone.
        // We will determine if it is beneficial to do the second skystone. Doing the second skystone takes time
        // and will gain us 14 extra points (10 for fetching the skystone cross the bridge line and 4 for dropping
        // it on the foundation). But if we run out of time, we risk giving up 15 points (10 for pulling the foundation
        // to the building site and 5 to park). Giving up 15 points for 14 points doesn't make sense. So we will only
        // do the second skystone if the total score we can achieve within the 30-second autonomous time is greater
        // than the 29 points in our SINGLE_SKYSTONE strategy.
        //
        if (autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_DOUBLE_SKYSTONE_SOLO && skystonesDropped == 1)
        {
            double projectedTime = elapsedTime;
            int projectedScore = 14;    //We already have one skystone on the foundation, so that's 14 points.

            projectedTime += autoChoices.strafeToFoundation? 16.0: 20.0; //TODO: determine extra time for not strafing.
            //
            // If we are within time limit, we gain an extra 14 points (10 for carrying the skystone crossing the
            // bridge line and 4 for dropping it off to the foundation).
            //
            if (projectedTime <= AUTONOMOUS_END_TIME) projectedScore += 14;
            //
            // Now check if we could just go park without moving the foundation. If we can, we are already ahead of
            // the SINGLE_SKYSTONE strategy.
            //
            projectedTime += autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                                3.5:
                             autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL?
                                0.1: 0.0;   //TODO: determine these times.
            //
            // If we are within time limit, we gain an extra 5 points for parking under the bridge.
            //
            if (projectedTime <= AUTONOMOUS_END_TIME) projectedScore += 5;

            willDo = projectedScore > MAX_SCORE_SINGLE_SKYSTONE;

            robot.globalTracer.traceInfo(
                    funcName, "[%.3f] >>> WillDo=%s, ProjectedTotalTime=%.3f, AchievableMaxScore=%d",
                    elapsedTime, willDo, projectedTime, projectedScore);
        }
        return willDo;
    }   //doSecondSkystone

    private boolean doPullFoundation(double elapsedTime)
    {
        final String funcName = "doPullFoundation";
        boolean willDo = false;
        double projectedTime = elapsedTime;

        if (autoChoices.moveFoundation)
        {
            if (autoChoices.strategy != CommonAuto.AutoStrategy.LOADING_ZONE_DOUBLE_SKYSTONE_SOLO)
            {
                willDo = true;
            }
            else
            {
                projectedTime += 7.0;   //Time to pull foundation to wall, and bump it in.
                //
                // If we are doing DOUBLE_SKYSTONE and we want to move the foundation, let's check if we have enough time to
                // do so.
                //
//                projectedTime += autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER ?
//                                    0.6 :   //Time to just move sideways under the bridge.
//                                 autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_WALL ?
//                                    1.0 : 0.5;  //Even NO_PARK still need time to move back to the wall.
                willDo = projectedTime < AUTONOMOUS_END_TIME;
            }
        }
        robot.globalTracer.traceInfo(
                funcName, "[%.3f] >>> WillDo=%s, ProjectedTotalTime=%.3f", elapsedTime, willDo, projectedTime);

        return willDo;
    }   //doPullFoundation

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
                    if (autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_SINGLE_SKYSTONE ||
                        autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_DOUBLE_SKYSTONE_SOLO)
                    {
                        visionParams.setScootCount(0);
                        visionParams.setAssumeLeftIfNotFound(true);
                        startX = RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_MID;
                    }
                    else if (autoChoices.robotStartX != 0.0)
                    {
                        //
                        // We are doing a double skystone with our alliance partner and we are flexible enough to
                        // start at any starting position specified during auto choice menu selection.
                        //
                        if (Math.abs(startX) < RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_MID)
                        {
                            //
                            // We are handling the stone set closer to the wall. Since our grabber design cannot
                            // get to the stone closest to the wall, we will skip that stone.
                            //
                            visionParams.setScootCount(1);
                        }
                        startX = autoChoices.robotStartX;
                    }
                    else if (autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_DOUBLE_SKYSTONE_FAR)
                    {
                        //
                        // We are handling the stone set away from the wall.
                        //
                        startX = RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_FAR;
                    }
                    else
                    {
                        //
                        // We are dealing with the stone set closer to the wall. Since our grabber design cannot
                        // get to the stone closest to the wall, we will skip that stone.
                        //
                        visionParams.setScootCount(1);
                        startX = RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_WALL;
                    }
                    startX *= allianceDirection;
                    robot.pidDrive.setAbsolutePose(new TrcPose2D(startX, RobotInfo.ABS_ROBOT_START_Y));

                    xPidCtrl.setNoOscillation(true);
                    yPidCtrl.setNoOscillation(true);
                    turnPidCtrl.setNoOscillation(true);
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
                    // Move closer slowly to a distance so Vuforia can detect the target and start lowering the
                    // extender arm to save time.
                    //
                    robot.extenderArm.setPosition(3.0);
                    robot.grabber.release();

                    xPidCtrl.setOutputLimit(0.5);
                    yPidCtrl.setOutputLimit(0.5);
                    yTarget = 14.0;
                    simplePidDrive.setRelativeYTarget(yTarget, State.MOVE_TO_FIRST_STONE);
                    break;

                case MOVE_TO_FIRST_STONE:
                    //
                    // The robot start position may be anywhere so we need to strafe to the first stone.
                    //
                    xTarget = (autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_SINGLE_SKYSTONE ||
                               autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_DOUBLE_SKYSTONE_SOLO)?
                                    0.0:
                              (Math.abs(startX) > RobotInfo.ABS_LOADING_ZONE_ROBOT_START_X_MID)?
                                    RobotInfo.ABS_FAR_STONE3_X: RobotInfo.ABS_WALL_STONE3_X;
                    if (xTarget == 0.0)
                    {
                        sm.setState(State.SETUP_VISION);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        xTarget *= allianceDirection;
                        simplePidDrive.setAbsoluteXTarget(xTarget, State.SETUP_VISION);
                        break;
                    }

                case SETUP_VISION:
                    if (skystonesDropped > 0)
                    {
                        //
                        // We are looking for the second skystone. We should be in front of it approximately so
                        // we don't need to scoot to the next stone but we do need to detect the exact alignment
                        // distance of the skystone so we can navigate the robot right in front of it for the
                        // grabbing.
                        //
                        visionParams.setScootCount(0);
                    }
                    skystoneVisionCommand.start();
                    // Delay turning the wrist until now so to give vision a chance to detect skystone before the
                    // grabber blocks the camera view.
                    robot.wrist.extend();
                    sm.setState(State.DO_VISION);
                    //
                    // Intentionally falling through to the next state.
                    //
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
                        sm.setState(State.GO_DOWN_ON_SKYSTONE);
                        //
                        // Intentionally falling through to the next state.
                        //
                    }
                    else
                    {
                        traceState = false;
                        break;
                    }

                case GO_DOWN_ON_SKYSTONE:
                    //
                    // We are done with vision, extend the arm to grab the skystone.
                    //
                    robot.extenderArm.setPosition(RobotInfo3543.EXTENDER_ARM_PICKUP_POS, event);
                    sm.waitForSingleEvent(event, State.PULL_SKYSTONE);
                    break;

                case PULL_SKYSTONE:
                    //
                    // Pull the skystone out to the travel Y position so we don't hit the bridge.
                    //
                    robot.grabber.grab();
                    yTarget = RobotInfo.ABS_ROBOT_TRAVEL_Y;
                    simplePidDrive.setAbsoluteYTarget(yTarget, State.START_EXTENDER_ARM_RETRACTION);
                    break;

                case START_EXTENDER_ARM_RETRACTION:
                    //
                    // TODO: may want to use a timer so we don't need to wait for the slow arm extender.
                    // Bring the extender arm back so we don't run into the bridge during travel.
                    //
                    nextState = autoChoices.strafeToFoundation?
                                    State.GOTO_FOUNDATION: State.TURN_TOWARDS_FOUNDATION;
                    robot.extenderArm.setPosition(RobotInfo3543.EXTENDER_ARM_CARRY_POS, event);
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case TURN_TOWARDS_FOUNDATION:
                    //
                    // For some reason, strafing is crappy. So we will turn, run in Y and turn back.
                    //
                    turnTarget = 90.0 * allianceDirection;
                    simplePidDrive.setAbsoluteHeadingTarget(turnTarget, State.GOTO_FOUNDATION);
                    break;

                case GOTO_FOUNDATION:
                    //
                    // Travel long distance to the foundation. So we need to travel in full speed to save time.
                    //
                    xPidCtrl.setOutputLimit(1.0);
                    yPidCtrl.setOutputLimit(1.0);
                    nextState = autoChoices.strafeToFoundation?
                                    State.APPROACH_FOUNDATION: State.TURN_BACK_TO_FOUNDATION;
                    xTarget = autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_SINGLE_SKYSTONE?
                                    RobotInfo.ABS_FOUNDATION_DROP_MID_X:
                              (autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_DOUBLE_SKYSTONE_CENTER ||
                               autoChoices.strategy == CommonAuto.AutoStrategy.LOADING_ZONE_DOUBLE_SKYSTONE_SOLO &&
                               skystonesDropped == 0)?
                                    RobotInfo.ABS_FOUNDATION_DROP_NEAR_X: RobotInfo.ABS_FOUNDATION_DROP_FAR_X;
                    xTarget *= allianceDirection;
                    simplePidDrive.setAbsoluteXTarget(xTarget, nextState);
                    break;

                case TURN_BACK_TO_FOUNDATION:
                    //
                    // We were traveling in the Y direction, so turn back to face the foundation.
                    //
                    turnTarget = 0.0;
                    simplePidDrive.setAbsoluteHeadingTarget(turnTarget, State.APPROACH_FOUNDATION);
                    break;

                case APPROACH_FOUNDATION:
                    //
                    // Raise the elevator and extender arm to drop position while approaching the foundation.
                    //
                    robot.elevator.setPosition(RobotInfo3543.ELEVATOR_DROP_HEIGHT);
                    robot.extenderArm.setPosition(RobotInfo3543.EXTENDER_ARM_DROP_POS);
                    yTarget =  RobotInfo.ABS_FOUNDATION_Y + 1.0; //TODO: Temporary inconsistency fix
                    if (skystonesDropped > 0) yTarget += 2.0;
                    simplePidDrive.setAbsoluteYTarget(yTarget, State.DROP_SKYSTONE);
                    break;

                case DROP_SKYSTONE:
                    //
                    // Drop off the skystone. If this is the first skystone, check if we want to go for the second
                    // skystone.
                    //
                    robot.grabber.release();
                    skystonesDropped++;
                    if (doSecondSkystone(elapsedTime))
                    {
                        // Wait until the skystone has dropped before moving.
                        nextState = State.BACK_OFF_FOUNDATION;
                        timer.set(0.5, event);
                    }
                    else if (doPullFoundation(elapsedTime))
                    {
                        //
                        // Start dropping the skystone but we don't have to wait for it. Just wait for the foundation
                        // to be latched and the robot can move immediately. The skystone can continue to drop enroute.
                        //
                        nextState = State.FINISH_DELAY;
                        robot.frontFoundationLatch.grab(event);
                    }
                    else
                    {
                        //
                        // We are not moving the foundation so we need to wait until the skystone is dropped.
                        //
                        nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                                        State.MOVE_TOWARDS_CENTER: State.SKIP_MOVE_FOUNDATION_PARK_WALL;
                        timer.set(0.5, event);
                    }
                    sm.waitForSingleEvent(event, nextState);
                    break;

                case BACK_OFF_FOUNDATION:
                    //
                    // We are going for the second skystone. Need to back up a little to clear the bridge.
                    //
                    robot.elevator.setPosition(5.0);
                    robot.extenderArm.zeroCalibrate();

                    nextState = autoChoices.strafeToFoundation? State.GOTO_SECOND_SKYSTONE: State.TURN_TOWARDS_QUARRY;
                    yTarget = RobotInfo.ABS_ROBOT_TRAVEL_Y;
                    simplePidDrive.setAbsoluteYTarget(yTarget, nextState);
                    break;

                case TURN_TOWARDS_QUARRY:
                    //
                    // For some reason, strafing was crappy. So we will turn, run in Y and turn back.
                    //
                    robot.elevator.zeroCalibrate(1.0);
                    turnTarget = -90.0 * allianceDirection;
                    simplePidDrive.setAbsoluteHeadingTarget(turnTarget, State.GOTO_SECOND_SKYSTONE);
                    break;

                case GOTO_SECOND_SKYSTONE:
                    //
                    // Travel long distance back to the second skystone. Determine the X position of the skystone.
                    // If the second skystone is the one closest to the wall, we can't get to it so just get the
                    // regular stone closest to the robot.
                    //
                    robot.elevator.zeroCalibrate(1.0);
                    nextState = autoChoices.strafeToFoundation? State.SETUP_VISION: State.TURN_TO_SECOND_SKYSTONE;
                    xTarget = skystoneVisionCommand.getSkystoneXPos() - 24.0*allianceDirection;
                    if (Math.abs(xTarget) < 8.0)
                    {
                        xTarget = RobotInfo.ABS_FAR_STONE3_X * allianceDirection;
                    }

                    simplePidDrive.setAbsoluteXTarget(xTarget, nextState);
                    break;

                case TURN_TO_SECOND_SKYSTONE:
                    //
                    // We traveled in the Y direction so we must turn back to face the skystone. Then call vision to
                    // detect and align to the skystone.
                    //
                    turnTarget = 0.0;
                    simplePidDrive.setAbsoluteHeadingTarget(turnTarget, State.SETUP_VISION);
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
                    robot.elevator.setPosition(5.0);
                    robot.extenderArm.setPosition(RobotInfo3543.EXTENDER_ARM_RETRACT_POS);

                    savedYPidCoeff = yPidCtrl.getPidCoefficients();
                    TrcPidController.PidCoefficients loadedYPidCoeff = savedYPidCoeff.clone();
                    loadedYPidCoeff.kP = RobotInfo3543.ENCODER_Y_LOADED_KP;
                    yPidCtrl.setPidCoefficients(loadedYPidCoeff);

                    yTarget = RobotInfo.ABS_ROBOT_START_Y - 3.0;
                    if (skystonesDropped > 1) yTarget -= 3.0;
                    simplePidDrive.setAbsoluteYTarget(yTarget, State.UNHOOK_FOUNDATION);
                    break;

                case UNHOOK_FOUNDATION:
                    //
                    // Pulling the heavy foundation to the wall may cause the wheels to slip so we need to correct
                    // the Y odometry and Y absolute target pose.
                    //
                    if (savedYPidCoeff != null)
                    {
                        yPidCtrl.setPidCoefficients(savedYPidCoeff);
                        savedYPidCoeff = null;
                    }

                    TrcPose2D pose = robot.driveBase.getFieldPosition();
                    pose.y = RobotInfo.ABS_ROBOT_START_Y;
                    robot.driveBase.setFieldPosition(pose);

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
                    simplePidDrive.setAbsoluteXTarget(xTarget, State.MOVE_TO_FOUNDATION_SIDE);
                    break;

                case MOVE_TO_FOUNDATION_SIDE:
                    //
                    // We can safely retract everything here because the skystone should be dropped by now.
                    //
                    robot.wrist.retract();
                    robot.elevator.zeroCalibrate();
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
                    xTarget = (skystonesDropped > 1? 16.0: 13.0) * allianceDirection;
                    simplePidDrive.setRelativeXTarget(xTarget, State.RESYNC_ROBOT_X);
                    break;

                case RESYNC_ROBOT_X:
                    pose = robot.driveBase.getFieldPosition();
                    pose.x = RobotInfo.ABS_FOUNDATION_SIDE_X * allianceDirection;
                    robot.driveBase.setFieldPosition(pose);

                    pose = robot.pidDrive.getAbsoluteTargetPose();
                    pose.x = RobotInfo.ABS_FOUNDATION_SIDE_X * allianceDirection;
                    robot.pidDrive.setAbsoluteTargetPose(pose);

                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.PARK_CLOSE_TO_CENTER?
                                    State.MOVE_UNDER_BRIDGE: State.MOVE_BACK_TO_WALL;
                    sm.setState(nextState);
                    break;

                case MOVE_BACK_TO_WALL:
                    //
                    // We are going to either park by the wall under the bridge or just park by the wall.
                    // Either way, let's get back to the wall.
                    //
                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.NO_PARK?
                                    State.DONE: State.MOVE_UNDER_BRIDGE;
                    yTarget = RobotInfo.ABS_ROBOT_START_Y;
                    simplePidDrive.setAbsoluteYTarget(yTarget, nextState);
                    break;

                case MOVE_UNDER_BRIDGE:
                    //
                    // Slide under the bridge.
                    //
                    xTarget = RobotInfo.ABS_UNDER_BRIDGE_PARK_X * allianceDirection;
                    simplePidDrive.setAbsoluteXTarget(xTarget, State.DONE);
                    break;

                case SKIP_MOVE_FOUNDATION_PARK_WALL:
                    //
                    // We did not move the foundation and we are going to park by the wall under the bridge.
                    // So let's get back to the wall.
                    //
                    yTarget = RobotInfo.ABS_ROBOT_START_Y;
                    simplePidDrive.setAbsoluteYTarget(yTarget, State.START_RETRACTIONS);
                    break;

                case MOVE_TOWARDS_CENTER:
                    //
                    // Move the robot toward the bridge so we won't hit our alliance partner's robot.
                    //
                    yTarget = RobotInfo.ABS_CENTER_BRIDGE_PARK_Y;
                    simplePidDrive.setAbsoluteYTarget(yTarget, State.START_RETRACTIONS);
                    break;

                case START_RETRACTIONS:
                    //
                    // We are done with the grabber, arm and elevator. Let's retract everything before moving under
                    // the bridge.
                    //
                    robot.elevator.zeroCalibrate();
                    robot.extenderArm.zeroCalibrate();
                    robot.wrist.retract(event);

                    nextState = autoChoices.parkUnderBridge == CommonAuto.ParkPosition.NO_PARK?
                                    State.DONE: State.MOVE_UNDER_BRIDGE;
                    sm.waitForSingleEvent(event, nextState);
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

}   //class CmdAutoLoadingZone3543
