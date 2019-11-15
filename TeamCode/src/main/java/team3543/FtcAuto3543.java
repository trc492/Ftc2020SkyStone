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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import common.CmdPidDrive;
import common.CmdTimedDrive;
import common.CommonAuto;
import trclib.TrcRobot;

@Autonomous(name="FtcAuto3543", group="FtcAuto")
public class FtcAuto3543 extends CommonAuto
{
    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //
        Robot3543 robot3543 = new Robot3543(TrcRobot.getRunMode());
        super.initRobot();
        super.initTeamSpecifics(robot3543);
        //
        // Strategies.
        //
        boolean hasRobot = robot3543.preferences.getBoolean("hasRobot");

        switch (autoChoices.strategy)
        {
            case LOADING_ZONE_FAR:
            case LOADING_ZONE_WALL:
                if (hasRobot)
                {
                    autoCommand = new CmdAutoLoadingZone3543(robot3543, autoChoices);
                }
                break;

            case BUILDING_ZONE:
                if (hasRobot)
                {
                    autoCommand = new CmdAutoBuildingZone3543(robot3543, autoChoices);
                }
                break;

            case PURE_PURSUIT_DRIVE:
//                if (hasRobot)
//                {
//                    autoCommand = new CmdPurePursuitDrive(
//                            robot3543.driveBase, posPidCoeff, turnPidCoeff, velPidCoeff);
//                }
                break;

            case PID_DRIVE:
                if (hasRobot)
                {
                    autoCommand = new CmdPidDrive(
                            robot3543, robot3543.pidDrive, autoChoices.delay,
                            autoChoices.xTarget*12.0, autoChoices.yTarget*12.0, autoChoices.turnTarget,
                            autoChoices.drivePower, false);
                }
                break;

            case TIMED_DRIVE:
                if (hasRobot)
                {
                    autoCommand = new CmdTimedDrive(
                            robot3543, autoChoices.delay, autoChoices.driveTime,
                            0.0, autoChoices.drivePower, 0.0);
                }
                break;

            case DO_NOTHING:
            default:
                autoCommand = null;
                break;
        }
    }   //initRobot

}   //class FtcAuto3543
