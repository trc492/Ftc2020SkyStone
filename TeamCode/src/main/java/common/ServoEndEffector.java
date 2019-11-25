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

public class ServoEndEffector
{
    public static class Parameters
    {
        double maxStepRate;
        double minPos;
        double maxPos;
        double retractPos;
        double retractTime;
        double extendPos;
        double extendTime;

        public Parameters setMaxStepRate(double maxStepRate)
        {
            this.maxStepRate = maxStepRate;
            return this;
        }

        public Parameters setMinPos(double minPos)
        {
            this.minPos = minPos;
            return this;
        }

        public Parameters setMaxPos(double maxPos)
        {
            this.maxPos = maxPos;
            return this;
        }

        public Parameters setRetractPos(double retractPos)
        {
            this.retractPos = retractPos;
            return this;
        }

        public Parameters setRetractTime(double retractTime)
        {
            this.retractTime = retractTime;
            return this;
        }

        public Parameters setExtendPos(double extendPos)
        {
            this.extendPos = extendPos;
            return this;
        }

        public Parameters setExtendTime(double extendTime)
        {
            this.extendTime = extendTime;
            return this;
        }

    }   //class Parameters

    private Parameters params;
    protected FtcServo servo1, servo2;
    protected TrcEnhancedServo endEffector;

    public ServoEndEffector(String servo1Name, String servo2Name, Parameters params)
    {
        if (servo1Name == null)
        {
            throw new IllegalArgumentException("Servo1 must not be null.");
        }

        this.params = params;
        servo1 = new FtcServo(servo1Name);
        servo2 = servo2Name == null? null: new FtcServo(servo2Name);
        endEffector = new TrcEnhancedServo("endEffector." + servo1Name, servo1, servo2);
        if (params.maxStepRate != 0.0)
        {
            endEffector.setStepMode(params.maxStepRate, params.minPos, params.maxPos);
        }
    }   //ServoEndEffector

    public ServoEndEffector(String servoName, Parameters params)
    {
        this(servoName, null, params);
    }   //ServoEndEffector

    public void retract()
    {
        endEffector.setPosition(params.retractPos);
    }   //retract

    public void retract(TrcEvent event)
    {
        endEffector.setPosition(params.retractPos, params.retractTime, event);
    }   //retract


    public void retract(double time, TrcEvent event)
    {
        endEffector.setPosition(params.retractPos, time, event);
    }   //retract

    public void extend()
    {
        endEffector.setPosition(params.extendPos);
    }   //extend

    public void extend(TrcEvent event)
    {
        endEffector.setPosition(params.extendPos, params.extendTime, event);
    }   //extend

    public void extend(double time, TrcEvent event)
    {
        endEffector.setPosition(params.extendPos, time, event);
    }   //extend

    public void setPower(double power)
    {
        endEffector.setPower(power);
    }   //setPower

    public double getPosition()
    {
        return endEffector.getPosition();
    }   //getPosition

    public void setPosition(double position)
    {
        endEffector.setPosition(position);
    }   //setPosition

    public void setPosition(double position, double time, TrcEvent event)
    {
        endEffector.setPosition(position, time, event);
    }   //setPosition

}   //class ServoEndEffector
