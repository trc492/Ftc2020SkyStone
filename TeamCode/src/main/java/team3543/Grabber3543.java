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

import common.Grabber;
import ftclib.FtcServo;
import trclib.TrcEvent;
import trclib.TrcTimer;

public class Grabber3543 implements Grabber
{
    public static final int GRAB_TIME = 1;
    public static final int RELEASE_TIME = 1;

    private FtcServo grabber = new FtcServo("grabberServo");
    private final TrcTimer grabTimer = new TrcTimer("grabberTimer");

    //
    // Implements Grabber interface
    //

    @Override
    public void grab()
    {
        grabber.setPosition(RobotInfo3543.GRABBER_CLOSE_POS);
    }   //grab

    @Override
    public void grab(TrcEvent whenFinishedEvent) {
        grab();
        grabTimer.set(GRAB_TIME, whenFinishedEvent);
    }

    @Override
    public void release()
    {
        grabber.setPosition(RobotInfo3543.GRABBER_OPEN_POS);
    }   //release

    @Override
    public void release(TrcEvent whenFinishedEvent) {
        release();
        grabTimer.set(RELEASE_TIME, whenFinishedEvent);
    }

}   //class Grabber3543
