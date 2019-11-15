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
import trclib.TrcHashMap;

public class Wrist
{
    private final FtcServo wrist = new FtcServo("wristServo");
    private final TrcEnhancedServo enhancedWrist = new TrcEnhancedServo("enhancedWristServo", wrist);
    private final double retractPos, extendPos;

    public Wrist(TrcHashMap<String, Object> params)
    {
        this.retractPos = params.getDouble("retractPos");
        this.extendPos = params.getDouble("extendPos");
        enhancedWrist.setStepMode(
                params.getDouble("maxStepRate"), params.getDouble("minPos"), params.getDouble("maxPos"));
        wrist.setInverted(params.getBoolean("inverted"));
    }   //Wrist

    public void setPower(double power)
    {
        enhancedWrist.setPower(power);
    }   //setPower

    public double getPosition()
    {
        return wrist.getPosition();
    }   //getPosition

    public void setPosition(double position, double timeout, TrcEvent event)
    {
        enhancedWrist.setPosition(position, timeout, event);
    }   //setPosition

    public void setPosition(double position)
    {
        enhancedWrist.setPosition(position);
    }   //setPosition

    public void retract()
    {
        setPosition(retractPos);
    }   //retract

    public void extend()
    {
        setPosition(extendPos);
    }   //extend

}   //class Wrist
