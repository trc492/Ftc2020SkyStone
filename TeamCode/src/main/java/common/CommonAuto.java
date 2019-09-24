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

import java.util.Date;
import java.util.Locale;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcOpMode;
import ftclib.FtcValueMenu;
import trclib.TrcRobot;
import trclib.TrcUtil;

public abstract class CommonAuto extends FtcOpMode
{
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

    protected static final boolean USE_TRACELOG = true;

    protected String moduleName = null;
    protected Robot robot = null;
    protected MatchType matchType = MatchType.PRACTICE;
    protected int matchNumber = 0;
    protected Alliance alliance = Alliance.RED_ALLIANCE;
    protected double delay = 0.0;
    protected TrcRobot.RobotCommand autoCommand = null;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initPeriodic()
    {
        if (robot.tensorFlowVision != null)
        {
            TensorFlowVision.TargetInfo[] targetsInfo =
                    robot.tensorFlowVision.getDetectedTargetsInfo(TensorFlowVision.LABEL_SKYSTONE);

            if (targetsInfo != null)
            {
                long currNanoTime = TrcUtil.getCurrentTimeNanos();

                robot.detectionSuccessCount++;
                robot.targetsInfo = targetsInfo;
                if (robot.detectionIntervalStartTime == 0)
                {
                    //
                    // This is the first time we detected target.
                    //
                    robot.detectionIntervalStartTime = currNanoTime;
                }
                else
                {
                    //
                    // Sum the interval between each successful detection.
                    //
                    robot.detectionIntervalTotalTime += currNanoTime - robot.detectionIntervalStartTime;
                    robot.detectionIntervalStartTime = currNanoTime;
                }
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
                msg = String.format(Locale.US, "Sky Stone found at position %d",
                        (robot.targetsInfo[0].rect.left + robot.targetsInfo[0].rect.right)/2);
            }
            else
            {
                msg = "Sky Stone not found";
            }
            robot.globalTracer.traceInfo(moduleName, "%s: DetectionAvgTime=%.3f, SuccessCount=%d, FailedCount=%d",
                    msg, robot.detectionIntervalTotalTime/robot.detectionSuccessCount/1000000000.0,
                    robot.detectionSuccessCount, robot.detectionFailedCount);
            robot.globalTracer.traceInfo(moduleName, "Shutting down TensorFlow.");
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

        if (robot.globalTracer.tracerLogIsOpened())
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

    protected void doMatchMenus()
    {
        //
        // Construct menus.
        //
        FtcChoiceMenu<MatchType> matchTypeMenu = new FtcChoiceMenu<>("Match type:", null);
        FtcValueMenu matchNumberMenu = new FtcValueMenu(
                "Match number:", matchTypeMenu, 1.0, 50.0, 1.0, 1.0,
                "%.0f");
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", matchNumberMenu);
        FtcValueMenu delayMenu = new FtcValueMenu(
                "Delay time:", allianceMenu, 0.0, 30.0, 1.0, 0.0,
                " %.0f sec");

        matchNumberMenu.setChildMenu(allianceMenu);
        //
        // Populate choice menus.
        //
        matchTypeMenu.addChoice("Practice", MatchType.PRACTICE, true, matchNumberMenu);
        matchTypeMenu.addChoice("Qualification", MatchType.QUALIFICATION, false, matchNumberMenu);
        matchTypeMenu.addChoice("Semi-final", MatchType.SEMI_FINAL, false, matchNumberMenu);
        matchTypeMenu.addChoice("Final", MatchType.FINAL, false, matchNumberMenu);

        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true, delayMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false, delayMenu);
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
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(1, "Match %s: %s, delay=%.0f",
                matchType.toString() + "_" + matchNumber, alliance, delay);
    }   //doMatchMenus

    protected void createTraceLog()
    {
        String filePrefix = String.format(Locale.US, "%s%02d", matchType, matchNumber);
        robot.globalTracer.openTraceLog("/sdcard/FIRST/tracelog", filePrefix);
    }   //createTraceLog

}   //class CommonAuto
