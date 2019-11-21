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

class Wrist3543 extends ServoEndEffector
{
    private final TrcEnhancedServo enhancedWrist;

    Wrist3543(ServoEndEffector.Parameters params)
    {
        super("wristServo", params);
        enhancedWrist = new TrcEnhancedServo("enhancedWrist", this.endEffector);
        enhancedWrist.setStepMode(
                RobotInfo3543.WRIST_MAX_STEPRATE, RobotInfo3543.WRIST_MIN_POS, RobotInfo3543.WRIST_MAX_POS);

        endEffector.setInverted(RobotInfo3543.WRIST_INVERTED);
    }   //Wrist3543

    void setPower(double power)
    {
        enhancedWrist.setPower(power);
    }   //setPower

}   //class Wrist3543
