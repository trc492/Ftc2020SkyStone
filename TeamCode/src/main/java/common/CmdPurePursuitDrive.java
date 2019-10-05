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

import trclib.TrcDriveBase;
import trclib.TrcEvent;
import trclib.TrcHolonomicPurePursuitController;
import trclib.TrcPath;
import trclib.TrcPidController;
import trclib.TrcPose2D;
import trclib.TrcRobot;
import trclib.TrcWaypoint;

public class CmdPurePursuitDrive implements TrcRobot.RobotCommand
{
    private static final String instanceName = "CmdPurePursuitDrive";

    private TrcHolonomicPurePursuitController purePursuitDrive;
    private TrcEvent event;

    public CmdPurePursuitDrive(
            TrcDriveBase driveBase, TrcPidController.PidCoefficients distPid, TrcPidController.PidCoefficients turnPid,
            TrcPidController.PidCoefficients velPid)
    {
        purePursuitDrive = new TrcHolonomicPurePursuitController(
                "PurePursuitDrive", driveBase, 10, 3.0, 2,
                distPid, turnPid, velPid);
        event = new TrcEvent("event");
    }   //CmdPurePursuitDrive

    public void start(TrcPose2D... poses)
    {
        TrcWaypoint[] waypoints = new TrcWaypoint[poses.length];

        for (int i = 0; i < waypoints.length; i++)
        {
            waypoints[i] = new TrcWaypoint(poses[i]);
        }

        purePursuitDrive.start(new TrcPath(true, waypoints), event, 0.0);
    }

    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        return event.isSignaled();
    }   //cmdPeriodic

    @Override
    public boolean isActive()
    {
        return purePursuitDrive.isActive();
    }   //isActive

    @Override
    public void cancel()
    {
        purePursuitDrive.cancel();
    }   //cancel

}   //CmdPurePursuitDrive
