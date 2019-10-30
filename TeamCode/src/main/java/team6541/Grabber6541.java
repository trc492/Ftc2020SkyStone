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

import common.Grabber;
import ftclib.FtcDcMotor;
import trclib.TrcEvent;
import trclib.TrcTimer;

/**
 * This class implements the grabber for team 6541. It has a motor wheel that sucks up the yellow block, holds it,
 * and then releases by turning the motor the other way.
 */

public class Grabber6541 implements Grabber
{
    private FtcDcMotor motor = new FtcDcMotor("grabberMotor");
    private TrcTimer timer = new TrcTimer("grabberTimer");
    private TrcEvent finishedEvent = null;

    private void timerNotify(Object context)
    {
        //Stop the motor.
        motor.set(0.0);
        //Signal finishedEvent and consume it if any.
        if (finishedEvent != null)
        {
            finishedEvent.set(true);
            finishedEvent = null;
        }
    }   //timerNotify

    private void setTimedPower(double power, double time, TrcEvent finishedEvent)
    {
        motor.set(power);
        timer.set(time, this::timerNotify);
        this.finishedEvent = finishedEvent;
    }   //setTimedPower

    //
    // Implements Grabber interface
    //

    @Override
    public void grab()
    {
        setTimedPower(RobotInfo6541.GRABBER_GRAB_POWER, RobotInfo6541.GRABBER_GRAB_TIME, null);
    }   //grab

    @Override
    public void grab(TrcEvent finishedEvent)
    {
        setTimedPower(RobotInfo6541.GRABBER_GRAB_POWER, RobotInfo6541.GRABBER_GRAB_TIME, finishedEvent);
    }   //grab

    @Override
    public void release()
    {
        setTimedPower(RobotInfo6541.GRABBER_RELEASE_POWER, RobotInfo6541.GRABBER_RELEASE_TIME, null);
    }   //release

    @Override
    public void release(TrcEvent finsihedEvent)
    {
        setTimedPower(RobotInfo6541.GRABBER_RELEASE_POWER, RobotInfo6541.GRABBER_RELEASE_TIME, finishedEvent);
    }   //release

}   //class Grabber6541
