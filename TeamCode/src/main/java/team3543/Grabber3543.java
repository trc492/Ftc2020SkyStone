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

import common.GenericGrabber;
import ftclib.FtcServo;
import trclib.TrcEvent;

public class Grabber3543 implements GenericGrabber
{
    private FtcServo grabber = new FtcServo("grabberServo");

    //
    // Implements GenericGrabber interface
    //

    @Override
    public void grab()
    {
        grabber.setPosition(RobotInfo3543.GRABBER_CLOSE_POS);
    }   //grab

    @Override
    public void grab(TrcEvent event)
    {
        grabber.setPosition(RobotInfo3543.GRABBER_CLOSE_POS, RobotInfo3543.GRABBER_GRAB_TIME, event);
    }   //grab

    @Override
    public void release()
    {
        grabber.setPosition(RobotInfo3543.GRABBER_OPEN_POS);
    }   //release

    @Override
    public void release(TrcEvent event)
    {
        grabber.setPosition(RobotInfo3543.GRABBER_OPEN_POS, RobotInfo3543.GRABBER_RELEASE_TIME, event);
    }   //release

}   //class Grabber3543
