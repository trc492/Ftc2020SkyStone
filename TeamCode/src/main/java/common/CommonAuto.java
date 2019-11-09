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

    public class MatchInfo
    {
        public MatchType matchType;
        public int matchNumber;

        public String toString()
        {
            return matchType + "_" + matchNumber;
        }   //toString
    }   //class MatchInfo

    public enum Alliance
    {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }   //enum Alliance

    public enum AutoStrategy
    {
        START_AT_LOADING_ZONE,
        START_AT_BUILDING_ZONE,
        PURE_PURSUIT_DRIVE,
        DISTANCE_DRIVE,
        TIMED_DRIVE,
        DO_NOTHING
    }   //AutoStrategy

    public enum ParkPosition
    {
        NO_PARK,
        PARK_CLOSE_TO_WALL,
        PARK_CLOSE_TO_CENTER
    }

    public class AutoChoices
    {
        public Alliance alliance = Alliance.RED_ALLIANCE;
        public double delay = 0.0;
        public AutoStrategy strategy = AutoStrategy.DO_NOTHING;
        public double foundationXPos = 0.0;
        public double foundationYPos = 0.0;
        public double foundationHeading = 0.0;
        public boolean moveFoundation = true;
        public ParkPosition parkUnderBridge = ParkPosition.NO_PARK;
        public double driveDistance = 0.0;
        public double driveTime = 0.0;
        public double drivePower = 0.0;

        public String toString()
        {
            return String.format(Locale.US,
                    "%s: Strategy=%s, delay=%.0f, moveFoundation=%s, foundationPos=(%.1f,%.1f), park=%s, " +
                    "driveDist=%.1f, driveTime=%.1f, drivePower=%.1f",
                    alliance, strategy, delay, moveFoundation, foundationXPos, foundationYPos, parkUnderBridge,
                    driveDistance, driveTime, drivePower);
        }   //toString
    }   //class AutoChoices

    protected String moduleName = null;
    protected Robot robot = null;
    protected MatchInfo matchInfo = new MatchInfo();
    protected AutoChoices autoChoices = new AutoChoices();
    protected TrcRobot.RobotCommand autoCommand = null;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Choice menus.
        //
        doMatchInfoMenus();
        if (robot.preferences.get("useTraceLog"))
        {
            createTraceLog();
        }
        doAutoChoicesMenus();
    }   //initRobot

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (robot.preferences.get("useTraceLog"))
        {
            robot.globalTracer.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(moduleName, "%s: ***** Starting autonomous *****", new Date());
        robot.globalTracer.traceInfo(moduleName, "[%s] %s", matchInfo, autoChoices);

        robot.startMode(nextMode);

        if (robot.tensorFlowVision != null)
        {
            String msg;

            if (robot.targetsInfo != null)
            {
                msg = String.format(Locale.US, "Sky Stone found at position %d",
                        robot.targetsInfo[0].rect.x + robot.targetsInfo[0].rect.width/2);
            }
            else
            {
                msg = "Sky Stone not found";
            }

            double avgDetectionTime = robot.detectionSuccessCount > 0 ?
                    robot.detectionIntervalTotalTime / robot.detectionSuccessCount / 1000000000.0 : 0;

            robot.globalTracer.traceInfo(moduleName, "%s: DetectionAvgTime=%.3f, SuccessCount=%d, FailedCount=%d",
                    msg, avgDetectionTime, robot.detectionSuccessCount, robot.detectionFailedCount);
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

    protected void doMatchInfoMenus()
    {
        //
        // Construct menus.
        //
        FtcChoiceMenu<MatchType> matchTypeMenu = new FtcChoiceMenu<>("Match type:", null);
        FtcValueMenu matchNumberMenu = new FtcValueMenu(
                "Match number:", matchTypeMenu, 1.0, 50.0, 1.0, 1.0,
                "%.0f");
        //
        // Populate choice menus.
        //
        matchTypeMenu.addChoice("Practice", MatchType.PRACTICE, true, matchNumberMenu);
        matchTypeMenu.addChoice("Qualification", MatchType.QUALIFICATION, false, matchNumberMenu);
        matchTypeMenu.addChoice("Semi-final", MatchType.SEMI_FINAL, false, matchNumberMenu);
        matchTypeMenu.addChoice("Final", MatchType.FINAL, false, matchNumberMenu);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(matchTypeMenu);
        //
        // Fetch choices.
        //
        matchInfo.matchType = matchTypeMenu.getCurrentChoiceObject();
        matchInfo.matchNumber = (int)matchNumberMenu.getCurrentValue();
    }   //doMatchInfoMenus

    protected void doAutoChoicesMenus()
    {
        //
        // Construct menus.
        //
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", null);
        FtcValueMenu delayMenu = new FtcValueMenu(
                "Delay time:", allianceMenu, 0.0, 30.0, 1.0, 0.0,
                " %.0f sec");
        FtcChoiceMenu<AutoStrategy> strategyMenu = new FtcChoiceMenu<>("Auto Strategies:", delayMenu);
        FtcValueMenu foundationXMenu = new FtcValueMenu(
                "Foundation X:", strategyMenu, 0.0, 24.0, 1.0, 0.0,
                "%.1f");
        FtcValueMenu foundationYMenu = new FtcValueMenu(
                "Foundation Y:", foundationXMenu, 0.0, 24.0, 1.0, 0.0,
                "%.1f");
        FtcValueMenu foundationHeadingMenu = new FtcValueMenu(
                "Foundation Heading:", foundationYMenu, 0.0, 360.0, 45.0,
                0.0, "%.1f");
        FtcChoiceMenu<Boolean> moveFoundationMenu = new FtcChoiceMenu<>("Move foundation:",
                foundationHeadingMenu);
        FtcChoiceMenu<ParkPosition> parkMenu = new FtcChoiceMenu<>("Park under bridge:", moveFoundationMenu);
        FtcValueMenu driveDistanceMenu = new FtcValueMenu(
                "Distance:", strategyMenu, -12.0, 12.0, 0.5, 4.0,
                " %.0f ft");
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
                "Drive time:", strategyMenu, 0.0, 30.0, 1.0, 5.0,
                " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
                "Drive power:", strategyMenu, -1.0, 1.0, 0.1, 0.5,
                " %.1f");

        delayMenu.setChildMenu(strategyMenu);
        foundationXMenu.setChildMenu(foundationYMenu);
        foundationYMenu.setChildMenu(foundationHeadingMenu);
        foundationHeadingMenu.setChildMenu(moveFoundationMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);
        //
        // Populate choice menus.
        //
        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true, delayMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false, delayMenu);

        strategyMenu.addChoice(
                "Start at Loading Zone", AutoStrategy.START_AT_LOADING_ZONE, true, foundationXMenu);
        strategyMenu.addChoice(
                "Start at Building Zone", AutoStrategy.START_AT_BUILDING_ZONE, false,
                moveFoundationMenu);
        strategyMenu.addChoice("Pure Pursuit Drive", AutoStrategy.PURE_PURSUIT_DRIVE, false);
        strategyMenu.addChoice(
                "Distance Drive", AutoStrategy.DISTANCE_DRIVE, false, driveDistanceMenu);
        strategyMenu.addChoice("Timed Drive", AutoStrategy.TIMED_DRIVE, false, driveTimeMenu);
        strategyMenu.addChoice("Do nothing", AutoStrategy.DO_NOTHING, false);

        moveFoundationMenu.addChoice("Yes", true, true, parkMenu);
        moveFoundationMenu.addChoice("No", false, false, parkMenu);

        parkMenu.addChoice("No Parking", ParkPosition.NO_PARK, true);
        parkMenu.addChoice("Close to wall", ParkPosition.PARK_CLOSE_TO_WALL, false);
        parkMenu.addChoice("Close to center bridge", ParkPosition.PARK_CLOSE_TO_CENTER, false);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(allianceMenu);
        //
        // Fetch choices.
        //
        autoChoices.alliance = allianceMenu.getCurrentChoiceObject();
        autoChoices.delay = delayMenu.getCurrentValue();
        autoChoices.strategy = strategyMenu.getCurrentChoiceObject();
        autoChoices.foundationXPos = foundationXMenu.getCurrentValue();
        autoChoices.foundationYPos = foundationYMenu.getCurrentValue();
        autoChoices.foundationHeading = foundationHeadingMenu.getCurrentValue();
        autoChoices.moveFoundation = moveFoundationMenu.getCurrentChoiceObject();
        autoChoices.parkUnderBridge = parkMenu.getCurrentChoiceObject();
        autoChoices.driveDistance = driveDistanceMenu.getCurrentValue();
        autoChoices.driveTime = driveTimeMenu.getCurrentValue();
        autoChoices.drivePower = drivePowerMenu.getCurrentValue();
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(2, "Auto Strategy: %s", strategyMenu.getCurrentChoiceText());
        robot.dashboard.displayPrintf(3, "Drive: distance=%.0f ft,Time=%.0f,Power=%.1f",
                autoChoices.driveDistance, autoChoices.driveTime, autoChoices.drivePower);
    }   //doAutoChoicesMenus

    protected void createTraceLog()
    {
        String filePrefix = String.format(Locale.US, "%s%02d", matchInfo.matchType, matchInfo.matchNumber);
        robot.globalTracer.openTraceLog("/sdcard/FIRST/tracelog", filePrefix);
    }   //createTraceLog

}   //class CommonAuto
