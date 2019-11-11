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
import trclib.TrcPidDrive;
import trclib.TrcStateMachine;

public class EnhancedPidDrive<StateType>
{
    private final TrcPidDrive pidDrive;
    private final TrcEvent event;
    private final TrcStateMachine<StateType> sm;
    private final double startX, startY;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param pidDrive specifies the PidDrive object to use for PID controlled drive.
     * @param event specifies the event to signal at the end of the drive.
     * @param sm specifies the state machine to advance to the next state at the end of the drive.
     * @param startX specifies the robot's starting X position in the field.
     * @param startY specifies the robot's starting Y position in the field.
     */
    public EnhancedPidDrive(
            TrcPidDrive pidDrive, TrcEvent event, TrcStateMachine<StateType> sm, double startX, double startY)
    {
        this.pidDrive = pidDrive;
        this.event = event;
        this.sm = sm;
        this.startX = startX;
        this.startY = startY;
    }   //EnhancedPidDrive

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absX specifies the absolute X target position.
     * @param absY specifies the absolute Y target position.
     * @param absHeading specifies the absolute target heading.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setAbsoluteTarget(double absX, double absY, double absHeading, StateType nextState)
    {
        pidDrive.setAbsoluteTarget(absX - startX, absY - startY, absHeading, event);
        sm.waitForSingleEvent(event, nextState);
    }   //setAbsoluteTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absX specifies the absolute X target position.
     * @param absY specifies the absolute Y target position.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setAbsoluteXYTarget(double absX, double absY, StateType nextState)
    {
        pidDrive.setAbsoluteXYTarget(absX - startX, absY - startY, event);
        sm.waitForSingleEvent(event, nextState);
    }   //setAbsoluteXYTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absX specifies the absolute X target position.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setAbsoluteXTarget(double absX, StateType nextState)
    {
        pidDrive.setAbsoluteXTarget(absX - startX, event);
        sm.waitForSingleEvent(event, nextState);
    }   //setAbsoluteXTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absY specifies the absolute Y target position.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setAbsoluteYTarget(double absY, StateType nextState)
    {
        pidDrive.setAbsoluteYTarget(absY - startY, event);
        sm.waitForSingleEvent(event, nextState);
    }   //setAbsoluteYTarget

    /**
     * This method sets the PID controlled absolute drive targets.
     *
     * @param absHeading specifies the absolute target heading.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setAbsoluteHeadingTarget(double absHeading, StateType nextState)
    {
        pidDrive.setAbsoluteHeadingTarget(absHeading, event);
        sm.waitForSingleEvent(event, nextState);
    }   //setAbsoluteHeadingTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param xDelta specifies the X target relative to the current X position.
     * @param yDelta specifies the Y target relative to the current Y position.
     * @param turnDelta specifies the turn target relative to the current heading.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setRelativeTarget(double xDelta, double yDelta, double turnDelta, StateType nextState)
    {
        pidDrive.setRelativeTarget(xDelta, yDelta, turnDelta, event);
        sm.waitForSingleEvent(event, nextState);
    }   //setRelativeTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param xDelta specifies the X target relative to the current X position.
     * @param yDelta specifies the Y target relative to the current Y position.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setRelativeXYTarget(double xDelta, double yDelta, StateType nextState)
    {
        pidDrive.setRelativeXYTarget(xDelta, yDelta, event);
        sm.waitForSingleEvent(event, nextState);
    }   //setRelativeXYTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param xDelta specifies the X target relative to the current X position.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setRelativeXTarget(double xDelta, StateType nextState)
    {
        pidDrive.setRelativeXTarget(xDelta, event);
        sm.waitForSingleEvent(event, nextState);
    }   //setRelativeXTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param yDelta specifies the Y target relative to the current Y position.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setRelativeYTarget(double yDelta, StateType nextState)
    {
        pidDrive.setRelativeYTarget(yDelta, event);
        sm.waitForSingleEvent(event, nextState);
    }   //setRelativeYTarget

    /**
     * This method sets the PID controlled relative drive targets.
     *
     * @param turnDelta specifies the turn target relative to the current heading.
     * @param nextState specifies the next state the state machine should advance to at the end of the drive.
     */
    public void setRelativeTurnTarget(double turnDelta, StateType nextState)
    {
        pidDrive.setRelativeTurnTarget(turnDelta, event);
        sm.waitForSingleEvent(event, nextState);
    }   //setRelativeTurnTarget

}   //EnhancedPidDrive
