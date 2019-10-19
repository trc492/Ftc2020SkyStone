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
import ftclib.FtcDigitalInput;
import trclib.TrcDigitalTrigger;
import trclib.TrcEvent;

/**
 * This class implements the grabber for team 6541. It has a motor wheel that sucks up the yellow block, holds it,
 * and then releases by turning the motor the other way.
 */

public class Grabber6541 implements Grabber
{
    private FtcDcMotor motor = new FtcDcMotor("grabberMotor");
    private FtcDigitalInput touch = new FtcDigitalInput("grabberTouchSensor");
    private TrcDigitalTrigger trigger = new TrcDigitalTrigger(
            "touchSensorTrigger", touch, this::onHasBrickStatusChanged);

    //
    // Implements Grabber interface
    //

    @Override
    public void grab()
    {
        //sets power to grab
        motor.setMotorPower(RobotInfo6541.GRABBER_GRAB_POWER);
    }   //grab

    @Override
    public void grab(TrcEvent event) {
        throw new RuntimeException("Not implemented yet");
    }

    @Override
    public void release()
    {
        //sets power to release
        motor.setMotorPower(-RobotInfo6541.GRABBER_GRAB_POWER);
    }   //release

    @Override
    public void release(TrcEvent event) {
        throw new RuntimeException("Not implemented yet");
    }

    //event
    private void onHasBrickStatusChanged(boolean hasBrick)
    {
        if (hasBrick)
        {
            //just grabbed a brick
            //set the power to keep going, but is a little bit to not burn the robot.
            motor.setMotorPower(RobotInfo6541.GRABBER_HOLD_POWER);
        }
        else
        {
            //lost the brick.
            //not have brick, stop motor.
            motor.setMotorPower(0.0);
        }
    }   //onHasBrickStatusChanged

}   //class Grabber6541
