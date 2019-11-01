package common;

import trclib.TrcEvent;
import trclib.TrcStateMachine;

/**
 * A collection of common simple robot movements that may be used when implementing autonomous modes.
 *
 * @param <StateType> The type of states used in the state machine.
 */
public class SimpleRobotMovements<StateType> {
    private final Robot robot;
    private final TrcEvent event;
    private final TrcStateMachine<StateType> stateMachine;
    private double xPos, yPos;

    /**
     * @param robot The Robot object to use.
     * @param sm The state machine to use.
     * @param event The event object to use when waiting for drive actions to complete.
     */
    public SimpleRobotMovements(Robot robot, TrcStateMachine<StateType> sm, TrcEvent event) {
        this.robot = robot;
        this.stateMachine = sm;
        this.event = event;
        xPos = robot.driveBase.getXPosition();
        yPos = robot.driveBase.getYPosition();
    }

    /**
     * Drives the robot in a straight line, along the current {@link Robot#targetHeading}.
     *
     * @param distance The distance to drive. Positive values make the robot go forward, negative
     *                 make the robot go in reverse.
     * @param nextState The next state to advance the state machine to when this action is complete.
     */
    public void driveStraightUntilDone(double distance, StateType nextState) {
        yPos += distance;
        robot.pidDrive.setTarget(
                yPos - robot.driveBase.getYPosition(), robot.targetHeading, false, event);
        stateMachine.waitForSingleEvent(event, nextState);
    } // driveStraightUntilDone

    /**
     * Drives the robot sideways, perpendicular to the current {@link Robot#targetHeading}.
     *
     * @param distance The distance to drive. Positive values make the robot go right, negative
     *                 make the robot go in left.
     * @param nextState The next state to advance the state machine to when this action is complete.
     */
    public void driveSidewaysUntilDone(double distance, StateType nextState) {
        xPos += distance;
        robot.pidDrive.setTarget(
                xPos - robot.driveBase.getXPosition(), 0.0, robot.targetHeading, false, event);
        stateMachine.waitForSingleEvent(event, nextState);
    } // driveSidewaysUntilDone

    /**
     * Drives the robot diagonally, preserving current {@link Robot#targetHeading}.
     *
     * @param xDistance The distance to drive along the robot's X axis. Positive values make the
     *                  robot go right, negative make the robot go in left.
     * @param yDistance It is the amount of distance to travel forward or backward. If positive
     *                  then forward, and if negative, it is backwards.
     * @param nextState The next state to advance the state machine to when this action is complete.
     */
    public void driveDiagonallyUntilDone(double xDistance, double yDistance, StateType nextState) {
        xPos += xDistance;
        yPos += yDistance;
        robot.pidDrive.setTarget(
                xPos - robot.driveBase.getXPosition(), yPos - robot.driveBase.getYPosition(),
                robot.targetHeading, false, event);
        stateMachine.waitForSingleEvent(event, nextState);
    } // driveDiagonallyUntilDone

    /**
     * Turns the robot in-place. Updates the current {@link Robot#targetHeading}.
     *
     * @param deltaHeading The number of degrees to turn. Positive values turn the robot clockwise;
     *                     negative values turn the robot counter-clockwise. {@link Robot#targetHeading}
     *                     is updated by this amount.
     * @param nextState THe next state to advance the state machine to when this action is complete.
     */
    public void turnInPlaceUntilDone(double deltaHeading, StateType nextState) {
        robot.targetHeading += deltaHeading;
        robot.pidDrive.setTarget(0.0, robot.targetHeading, false, event);
        stateMachine.waitForSingleEvent(event, nextState);
    } // turnInPlaceUntilDone

} // SimpleRobotMovements
