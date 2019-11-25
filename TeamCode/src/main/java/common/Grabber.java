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

import trclib.TrcEvent;

/**
 * This class really serves very little purpose. It is basically a ServoEndEffector renaming methods retract to grab
 * and extend to release. That's it, nothing more!
 */
public class Grabber extends ServoEndEffector
{
    public Grabber(String servo1Name, String servo2Name, Parameters params)
    {
        super(servo1Name, servo2Name, params);
    }

    public Grabber(String servoName, Parameters params)
    {
        super(servoName, params);
    }

    public void grab()
    {
        super.retract();
    }

    public void grab(TrcEvent event)
    {
        super.retract(event);
    }

    public void grab(double time, TrcEvent event)
    {
        super.retract(time, event);
    }

    public void release()
    {
        super.extend();
    }

    public void release(TrcEvent event)
    {
        super.extend(event);
    }

    public void release(double time, TrcEvent event)
    {
        super.extend(time, event);
    }

}   //class Grabber
