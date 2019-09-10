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

import java.util.Date;
import java.util.Locale;

import common.CmdPidDrive;
import common.CmdTimedDrive;
import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import trclib.TrcRobot;
import trclib.TrcUtil;

@Autonomous(name="Autonomous", group="FtcAuto")
public class FtcAuto extends FtcOpMode
{
    private static final boolean USE_TRACELOG = true;

    public enum MatchType
    {
        PRACTICE,
        QUALIFICATION,
        SEMI_FINAL,
        FINAL
    }   //enum MatchType

    public enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    public enum Strategy
    {
        DISTANCE_DRIVE,
        TIMED_DRIVE,
        DO_NOTHING
    }   //enum Strategy

    private static final String moduleName = "FtcAuto";
    private Robot robot;

    private TrcRobot.RobotCommand autoCommand = null;
    private MatchType matchType = MatchType.PRACTICE;
    private int matchNumber = 0;
    private Alliance alliance = Alliance.RED_ALLIANCE;
    private double delay = 0.0;
    private Strategy strategy = Strategy.DO_NOTHING;
    private double driveDistance = 0.0;
    private double driveTime = 0.0;
    private double drivePower = 0.0;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //
        robot = new Robot(TrcRobot.getRunMode());
        //
        // Choice menus.
        //
        doMenus();

        if (USE_TRACELOG)
        {
            String filePrefix = String.format(Locale.US, "%s%02d", matchType, matchNumber);
            robot.globalTracer.openTraceLog("/sdcard/FIRST/tracelog", filePrefix);
        }

        //
        // Strategies.
        //
        switch (strategy)
        {
            case DISTANCE_DRIVE:
                autoCommand = new CmdPidDrive(
                        robot, robot.pidDrive, delay, 0.0, driveDistance*12.0, 0.0);
                break;

            case TIMED_DRIVE:
                autoCommand = new CmdTimedDrive(robot, delay, driveTime, 0.0, drivePower, 0.0);
                break;

            case DO_NOTHING:
            default:
                autoCommand = null;
                break;
        }
    }   //initRobot

    @Override
    public void initPeriodic()
    {
        if (robot.tensorFlowVision != null)
        {
            //
            // TensorFlowVision impacts performance, so we will only do this during init period not in autonomous
            // period.
            //
            TensorFlowVision.TargetInfo[] targetsInfo =
                    robot.tensorFlowVision.getDetectedTargetsInfo(Robot.LABEL_FIRST_ELEMENT);

            if (targetsInfo != null)
            {
                long currNanoTime = TrcUtil.getCurrentTimeNanos();

                robot.detectionSuccessCount++;
                robot.targetsInfo = targetsInfo;
                if (robot.detectionIntervalStartTime != 0)
                {
                    //
                    // Sum the interval between each successful detection.
                    //
                    robot.detectionIntervalTotalTime += currNanoTime - robot.detectionIntervalStartTime;
                }
                robot.detectionIntervalStartTime = currNanoTime;
            }
            else
            {
                robot.detectionFailedCount++;
            }
        }
        else
        {
            super.initPeriodic();
        }
    }   //initPeriodic

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (USE_TRACELOG)
        {
            robot.globalTracer.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(moduleName, "%s: ***** Starting autonomous *****", new Date());
        robot.startMode(nextMode);

        if (robot.tensorFlowVision != null)
        {
            String msg;

            if (robot.targetsInfo != null)
            {
                msg = String.format(Locale.US, "Target found at position %d",
                        robot.targetsInfo[0].rect.centerX());
            }
            else
            {
                msg = "Target not found";
            }
            robot.globalTracer.traceInfo(moduleName, "%s: DetectionAvgTime=%.3f, SuccessCount=%d, FailedCount=%d",
                    msg, robot.detectionIntervalTotalTime/robot.detectionSuccessCount/1000000000.0,
                    robot.detectionSuccessCount, robot.detectionFailedCount);
            robot.globalTracer.traceInfo(moduleName, "Shutting down TensorFlow.");
            //
            // We are done with TensorFlow, shut it down so it doesn't affect autonomous performance.
            //
            robot.tensorFlowVision.shutdown();
            robot.tensorFlowVision = null;
        }

        if (robot.battery != null)
        {
            robot.battery.setEnabled(true);
        }
        robot.dashboard.clearDisplay();
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        robot.stopMode(prevMode);
        if (robot.battery != null)
        {
            robot.battery.setEnabled(false);
        }
        printPerformanceMetrics(robot.globalTracer);

        if (USE_TRACELOG)
        {
            robot.globalTracer.closeTraceLog();
        }
    }   //stopMode

    @Override
    public void runContinuous(double elapsedTime)
    {
        if (autoCommand != null)
        {
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }   //runContinuous

    private void doMenus()
    {
        //
        // Create menus.
        //
        FtcChoiceMenu<MatchType> matchTypeMenu = new FtcChoiceMenu<>("Match type:", null, robot);
        FtcValueMenu matchNumberMenu = new FtcValueMenu(
                "Match number:", matchTypeMenu, robot,
                1.0, 50.0, 1.0, 1.0, "%.0f");
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", matchNumberMenu, robot);
        FtcValueMenu delayMenu = new FtcValueMenu(
                "Delay time:", allianceMenu, robot,
                0.0, 30.0, 1.0, 0.0, " %.0f sec");
        FtcChoiceMenu<Strategy> strategyMenu = new FtcChoiceMenu<>("Strategies:", delayMenu, robot);
        FtcValueMenu driveDistanceMenu = new FtcValueMenu(
                "Distance:", strategyMenu, robot,
                -12.0, 12.0, 0.5, 4.0, " %.0f ft");
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
                "Drive time:", strategyMenu, robot,
                0.0, 30.0, 1.0, 5.0, " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
                "Drive power:", strategyMenu, robot,
                -1.0, 1.0, 0.1, 0.5, " %.1f");

        matchNumberMenu.setChildMenu(allianceMenu);
        delayMenu.setChildMenu(strategyMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);
        //
        // Populate choice menus.
        //
        matchTypeMenu.addChoice("Practice", MatchType.PRACTICE, true, matchNumberMenu);
        matchTypeMenu.addChoice("Qualification", MatchType.QUALIFICATION, false, matchNumberMenu);
        matchTypeMenu.addChoice("Semi-final", MatchType.SEMI_FINAL, false, matchNumberMenu);
        matchTypeMenu.addChoice("Final", MatchType.FINAL, false, matchNumberMenu);

        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true, delayMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false, delayMenu);

        strategyMenu.addChoice("Distance Drive", Strategy.DISTANCE_DRIVE, false, driveDistanceMenu);
        strategyMenu.addChoice("Timed Drive", Strategy.TIMED_DRIVE, false, driveTimeMenu);
        strategyMenu.addChoice("Do nothing", Strategy.DO_NOTHING, false);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(matchTypeMenu);
        //
        // Fetch choices.
        //
        matchType = matchTypeMenu.getCurrentChoiceObject();
        matchNumber = (int)matchNumberMenu.getCurrentValue();
        alliance = allianceMenu.getCurrentChoiceObject();
        delay = delayMenu.getCurrentValue();
        strategy = strategyMenu.getCurrentChoiceObject();
        driveDistance = driveDistanceMenu.getCurrentValue();
        driveTime = driveTimeMenu.getCurrentValue();
        drivePower = drivePowerMenu.getCurrentValue();
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(1, "== Match: %s ==",
                matchType.toString() + "_" + matchNumber);
        robot.dashboard.displayPrintf(2, "Auto Strategy: %s", strategyMenu.getCurrentChoiceText());
        robot.dashboard.displayPrintf(3, "Alliance=%s,Delay=%.0f sec", alliance.toString(), delay);
        robot.dashboard.displayPrintf(4, "Drive: distance=%.0f ft,Time=%.0f,Power=%.1f",
                driveDistance, driveTime, drivePower);
    }   //doMenus

}   //class FtcAuto
