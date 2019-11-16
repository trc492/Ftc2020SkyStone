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
import trclib.TrcEvent;
import trclib.TrcHashMap;

public class ServoEndEffector
{
    private double retractPos, extendPos;
    private FtcServo endEffector;

    public ServoEndEffector(String instanceName, TrcHashMap<String, Double> params)
    {
        this.retractPos = params.getDouble("retractPos");
        this.extendPos = params.getDouble("extendPos");
        endEffector = new FtcServo(instanceName);
    }   //ServoEndEffector

    public void retract()
    {
        endEffector.setPosition(retractPos);
    }   //retract

    public void retract(double time, TrcEvent event)
    {
        endEffector.setPosition(retractPos, time, event);
    }   //retract

    public void extend()
    {
        endEffector.setPosition(extendPos);
    }   //extend

    public void extend(double time, TrcEvent event)
    {
        endEffector.setPosition(extendPos, time, event);
    }   //extend

}   //class ServoEndEffector
