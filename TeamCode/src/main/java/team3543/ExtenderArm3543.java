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

import ftclib.FtcServo;
import trclib.TrcEnhancedServo;
import trclib.TrcEvent;

class ExtenderArm3543
{
    private final FtcServo extender = new FtcServo("extenderArmServo");
    private final TrcEnhancedServo enhancedExtender = new TrcEnhancedServo("enhancedExtenderArm", extender);

    ExtenderArm3543()
    {
        enhancedExtender.setStepMode(
                RobotInfo3543.EXTENDER_ARM_MAX_STEPRATE, RobotInfo3543.EXTENDER_ARM_LOGICAL_MIN_POS,
                RobotInfo3543.EXTENDER_ARM_LOGICAL_MAX_POS);
    }   //ExtenderArm3543

    void setPower(double power)
    {
        enhancedExtender.setPower(power);
    }   //setPower

    void extend()
    {
        enhancedExtender.setPosition(RobotInfo3543.EXTENDER_ARM_PLACEMENT_POS);
    }   //extend

    void extendMax()
    {
        enhancedExtender.setPosition(RobotInfo3543.EXTENDER_ARM_DOWN_POS);
    }   //extendMax

    void extend(TrcEvent event)
    {
        enhancedExtender.setPosition(
                RobotInfo3543.EXTENDER_ARM_PLACEMENT_POS, RobotInfo3543.EXTENDER_ARM_MOVE_TIME, event);
    }   //extend

    void extend(double time, TrcEvent event)
    {
        enhancedExtender.setPosition(
                RobotInfo3543.EXTENDER_ARM_PLACEMENT_POS, time, event);
    }   //extend

    void retract()
    {
        enhancedExtender.setPosition(RobotInfo3543.EXTENDER_ARM_RETRACTED_POS);
    }   //extend

    void retract(TrcEvent event)
    {
        enhancedExtender.setPosition(
                RobotInfo3543.EXTENDER_ARM_RETRACTED_POS, RobotInfo3543.EXTENDER_ARM_MOVE_TIME, event);
    }   //retract

    void retract(double time, TrcEvent event)
    {
        enhancedExtender.setPosition(
                RobotInfo3543.EXTENDER_ARM_RETRACTED_POS, time, event);
    }   //retract

    double getPosition()
    {
        return extender.getPosition();
    }

}   //class ExtenderArm3543
