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

package team6541;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import common.CmdPidDrive;
import common.CmdTimedDrive;
import common.CommonAuto;
import trclib.TrcRobot;

@Autonomous(name="FtcAuto6541", group="FtcAuto")
public class FtcAuto6541 extends CommonAuto
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
        Robot6541 robot6541 = new Robot6541(TrcRobot.getRunMode());
        super.setRobot(robot6541);
        super.initRobot();
        //
        // Strategies.
        //
        switch (autoChoices.strategy)
        {
            case LOADING_ZONE_SINGLE_SKYSTONE:
            case LOADING_ZONE_DOUBLE_SKYSTONE_SOLO:
            case LOADING_ZONE_DOUBLE_SKYSTONE_FAR:
            case LOADING_ZONE_DOUBLE_SKYSTONE_CENTER:
                if (robot6541.preferences.hasRobot)
                {
                    autoCommand = new CmdAutoLoadingZone6541(robot6541, autoChoices);
                }
                break;

            case BUILDING_ZONE:
                if (robot6541.preferences.hasRobot)
                {
                    autoCommand = new CmdAutoBuildingZone6541(robot6541, autoChoices);
                }
                break;

            case PURE_PURSUIT_DRIVE:
//                if (robot6541.preferences.hasRobot)
//                {
//                    autoCommand = new CmdPurePursuitDrive(
//                            robot3543.driveBase, posPidCoeff, turnPidCoeff, velPidCoeff);
//                }
                break;

            case PID_DRIVE:
                if (robot6541.preferences.hasRobot)
                {
                    autoCommand = new CmdPidDrive(
                            robot6541, robot6541.pidDrive, autoChoices.startDelay,
                            autoChoices.xTarget*12.0, autoChoices.yTarget*12.0, autoChoices.turnTarget,
                            autoChoices.drivePower, false, false);
                }
                break;

            case TIMED_DRIVE:
                if (robot6541.preferences.hasRobot)
                {
                    autoCommand = new CmdTimedDrive(
                            robot6541, autoChoices.startDelay, autoChoices.driveTime,
                            0.0, autoChoices.drivePower, 0.0);
                }
                break;

            case DO_NOTHING:
            default:
                autoCommand = null;
                break;
        }
    }   //initRobot

}   //class FtcAuto6541
