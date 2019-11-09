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

import common.CmdEnhancedPidDrive;
import common.CmdTimedDrive;
import common.CommonAuto;
import trclib.TrcRobot;

@Autonomous(name="FtcAuto3543", group="FtcAuto")
public class FtcAuto3543 extends CommonAuto
{
    private static final String MODULE_NAME = "FtcAuto3543";

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //
        moduleName = MODULE_NAME;
        robot = new Robot3543(TrcRobot.getRunMode());
        super.initRobot();
        //
        // Strategies.
        //
        boolean hasRobot = robot.preferences.get("hasRobot");

        switch (autoChoices.strategy)
        {
            case START_AT_LOADING_ZONE:
                if (hasRobot)
                {
                    autoCommand = new CmdAutoLoadingZone3543(robot, autoChoices);
                }
                break;

            case START_AT_BUILDING_ZONE:
                if (hasRobot)
                {
                    autoCommand = new CmdAutoBuildingZone3543(robot, autoChoices);
                }
                break;

            case PURE_PURSUIT_DRIVE:
                if (hasRobot)
                {
//                    autoCommand = new CmdPurePursuitDrive(
//                            robot.driveBase, posPidCoeff, turnPidCoeff, velPidCoeff);
                }
                break;

            case PID_DRIVE:
                if (hasRobot)
                {
                    autoCommand = new CmdEnhancedPidDrive(
                            robot, robot.driveBase, robot.pidDrive, autoChoices.delay,
                            autoChoices.xTarget*12.0, autoChoices.yTarget*12.0, autoChoices.turnTarget,
                            autoChoices.drivePower, false);
                }
                break;

            case TIMED_DRIVE:
                if (hasRobot)
                {
                    autoCommand = new CmdTimedDrive(
                            robot, autoChoices.delay, autoChoices.driveTime,
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
