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

import common.ServoEndEffector;
import trclib.TrcEnhancedServo;

class ExtenderArm3543 extends ServoEndEffector
{
    private final TrcEnhancedServo enhancedExtender;

    ExtenderArm3543(ServoEndEffector.Parameters params)
    {
        super("extenderArmServo", params);
        enhancedExtender = new TrcEnhancedServo("enhancedExtenderArm", this.endEffector);
        enhancedExtender.setStepMode(
                RobotInfo3543.EXTENDER_ARM_MAX_STEPRATE, RobotInfo3543.EXTENDER_ARM_LOGICAL_MIN_POS,
                RobotInfo3543.EXTENDER_ARM_LOGICAL_MAX_POS);
    }   //ExtenderArm3543

    void setPower(double power)
    {
        enhancedExtender.setPower(power);
    }   //setPower

    void extendMax()
    {
        enhancedExtender.setPosition(RobotInfo3543.EXTENDER_ARM_DOWN_POS);
    }   //extendMax

}   //class ExtenderArm3543
