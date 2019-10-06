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

import ftclib.FtcDcMotor;
import ftclib.FtcDigitalInput;
import trclib.TrcDigitalTrigger;

//class Grabber6541
//the grabber has a motor wheel that sucks up the yellow block, holds it, and then releases by turning the motor the other way.

public class Grabber6541
{
    //these are fields:
    private FtcDcMotor motor = new FtcDcMotor("grabMotor");
    private FtcDigitalInput touch = new FtcDigitalInput("touchSensor");
    private TrcDigitalTrigger trigger = new TrcDigitalTrigger(
            "touchSensorTrigger", touch, this::onHasBrickStatusChanged);


    public void grab()
    {
        //sets power to grab
        motor.setMotorPower(0.5);
    }

    public void release()
    {
        //sets power to release
        motor.setMotorPower(-0.5);
    }

    //event
    private void onHasBrickStatusChanged(boolean hasBrick)
    {
        if(hasBrick)
        {
            //just grabbed a brick
            //set the power to keep going, but is a little bit to not burn the robot.
            motor.setMotorPower(0.1);
        }
        else
        {
            //lost the brick.
            //not have brick, stop motor.
            motor.setMotorPower(0);
        }
    }

}
