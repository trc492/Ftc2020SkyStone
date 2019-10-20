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

import ftclib.FtcServo;
import trclib.TrcEnhancedServo;
import trclib.TrcEvent;
import trclib.TrcTimer;

public class ArmExtender
{
    private final FtcServo extender = new FtcServo("armExtenderServo");
    private final TrcEnhancedServo enhancedExtender = new TrcEnhancedServo("enhancedArmExtender", extender);
    private final TrcTimer moveTime = new TrcTimer("armExtenderMoveTimer");

    public ArmExtender(double maxStepRate, double minPos, double maxPos)
    {
        enhancedExtender.setStepMode(maxStepRate, minPos, maxPos);
    }   //ArmExtender

    public void setPower(double power)
    {
        enhancedExtender.setPower(power);
    }   //setPower

    public void setPosition(double position)
    {
        enhancedExtender.setPosition(position);
    }   //setPosition

    public void extend() {
        extender.setPosition(RobotInfoCommon.ARM_EXTENDER_EXTENDED_POS);
    }

    public void retract() {
        extender.setPosition(RobotInfoCommon.ARM_EXTENDER_RETRACTED_POS);
    }

    public void extend(TrcEvent event) {
        extend();
        moveTime.set(RobotInfoCommon.ARM_EXTENDER_MOVE_TIME, event);
    }

    public void retract(TrcEvent event) {
        retract();
        moveTime.set(RobotInfoCommon.ARM_EXTENDER_MOVE_TIME, event);
    }
}   //class ArmExtender
