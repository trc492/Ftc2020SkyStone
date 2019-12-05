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
import trclib.TrcPose2D;
import trclib.TrcRobot;

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
        MatchType matchType;
        int matchNumber;

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
        LOADING_ZONE_MID,
        LOADING_ZONE_FAR,
        LOADING_ZONE_WALL,
        BUILDING_ZONE,
        PURE_PURSUIT_DRIVE,
        PID_DRIVE,
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
        public double robotStartX = 0.0;
        public boolean moveFoundation = true;
        public ParkPosition parkUnderBridge = ParkPosition.NO_PARK;
        public double xTarget = 0.0;
        public double yTarget = 0.0;
        public double turnTarget = 0.0;
        public double driveTime = 0.0;
        public double drivePower = 0.0;

        public String toString()
        {
            return String.format(Locale.US,
                    "%s: Strategy=%s, delay=%.0f, moveFoundation=%s, robotStartX=%.1f, park=%s, " +
                    "xTarget=%.1f, yTarget=%.1f, turnTarget=%.1f, driveTime=%.1f, drivePower=%.1f",
                    alliance, strategy, delay, moveFoundation, robotStartX, parkUnderBridge,
                    xTarget, yTarget, turnTarget, driveTime, drivePower);
        }   //toString
    }   //class AutoChoices
    //
    // These field origin poses can be used to transform the Vuforia coordinate system to the alliance oriented
    // field coordinate system. Without this type of transform, the autonomous navigation will have to deal
    // with very different coordinates for different alliances. With this type of transform, the autonomous
    // for different alliances will only have to deal with changing the sign of one dimension.
    //
    private static final TrcPose2D RED_ALLIANCE_FIELD_ORIGIN =
            new TrcPose2D(-VuforiaVision.HALF_FIELD_INCHES, -VuforiaVision.HALF_FIELD_INCHES, 0.0);
    private static final TrcPose2D BLUE_ALLIANCE_FIELD_ORIGIN =
            new TrcPose2D(-VuforiaVision.HALF_FIELD_INCHES, VuforiaVision.HALF_FIELD_INCHES, 180.0);
    private String moduleName = null;
    private Robot robot = null;
    private MatchInfo matchInfo = new MatchInfo();
    protected AutoChoices autoChoices = new AutoChoices();
    protected TrcRobot.RobotCommand autoCommand = null;

    protected void setRobot(Robot robot)
    {
        this.robot = robot;
    }   //setRobot

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
        if (robot.preferences.useTraceLog)
        {
            createTraceLog();
        }
        doAutoChoicesMenus();
        robot.setFieldOrigin(
                autoChoices.alliance == Alliance.RED_ALLIANCE? RED_ALLIANCE_FIELD_ORIGIN: BLUE_ALLIANCE_FIELD_ORIGIN);
    }   //initRobot

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (robot.preferences.useTraceLog)
        {
            robot.globalTracer.setTraceLogEnabled(true);
        }
        robot.globalTracer.traceInfo(moduleName, "%s: ***** Starting autonomous *****", new Date());
        robot.globalTracer.traceInfo(moduleName, "[%s] %s", matchInfo, autoChoices);

        robot.startMode(nextMode);
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
            if (robot.vuforiaVision != null)
            {
                TrcPose2D robotPose = robot.getRobotPose(VuforiaVision.skystoneTargetName, true);
                robot.dashboard.displayPrintf(2, "RobotPose: %s", robotPose);
            }
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }   //runContinuous

    private void doMatchInfoMenus()
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

    private void doAutoChoicesMenus()
    {
        //
        // Construct menus.
        //
        FtcChoiceMenu<Alliance> allianceMenu = new FtcChoiceMenu<>("Alliance:", null);
        FtcValueMenu delayMenu = new FtcValueMenu(
                "Delay time:", allianceMenu, 0.0, 30.0, 1.0, 0.0,
                " %.0f sec");
        FtcChoiceMenu<AutoStrategy> strategyMenu = new FtcChoiceMenu<>("Auto Strategies:", delayMenu);
        FtcValueMenu robotStartXMenu = new FtcValueMenu(
                "Robot Start X:", strategyMenu, 0.0, 80.0, 1.0, 45.0,
                "%.0f in.");
        FtcChoiceMenu<Boolean> moveFoundationMenu = new FtcChoiceMenu<>("Move foundation:",
                robotStartXMenu);
        FtcChoiceMenu<ParkPosition> parkMenu = new FtcChoiceMenu<>("Park under bridge:", moveFoundationMenu);
        FtcValueMenu xTargetMenu = new FtcValueMenu(
                "xTarget:", strategyMenu, -12.0, 12.0, 0.5, 4.0,
                " %.0f ft");
        FtcValueMenu yTargetMenu = new FtcValueMenu(
                "yTarget:", strategyMenu, -12.0, 12.0, 0.5, 4.0,
                " %.0f ft");
        FtcValueMenu turnTargetMenu = new FtcValueMenu(
                "turnTarget:", strategyMenu, -180.0, 180.0, 5.0, 90.0,
                " %.0f ft");
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
                "Drive time:", strategyMenu, 0.0, 30.0, 1.0, 5.0,
                " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
                "Drive power:", strategyMenu, -1.0, 1.0, 0.1, 0.5,
                " %.1f");

        delayMenu.setChildMenu(strategyMenu);
        robotStartXMenu.setChildMenu(moveFoundationMenu);
        xTargetMenu.setChildMenu(yTargetMenu);
        yTargetMenu.setChildMenu(turnTargetMenu);
        turnTargetMenu.setChildMenu(drivePowerMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);
        //
        // Populate choice menus.
        //
        allianceMenu.addChoice("Red", Alliance.RED_ALLIANCE, true, delayMenu);
        allianceMenu.addChoice("Blue", Alliance.BLUE_ALLIANCE, false, delayMenu);

        strategyMenu.addChoice(
                "Loading Zone Middle", AutoStrategy.LOADING_ZONE_MID, true, moveFoundationMenu);
        strategyMenu.addChoice(
                "Loading Zone Far Side", AutoStrategy.LOADING_ZONE_FAR, false, robotStartXMenu);
        strategyMenu.addChoice(
                "Loading Zone Wall Side", AutoStrategy.LOADING_ZONE_WALL, false, robotStartXMenu);
        strategyMenu.addChoice(
                "Building Zone", AutoStrategy.BUILDING_ZONE, false, moveFoundationMenu);
        strategyMenu.addChoice("Pure Pursuit Drive", AutoStrategy.PURE_PURSUIT_DRIVE, false);
        strategyMenu.addChoice(
                "PID Drive", AutoStrategy.PID_DRIVE, false, xTargetMenu);
        strategyMenu.addChoice("Timed Drive", AutoStrategy.TIMED_DRIVE, false, driveTimeMenu);
        strategyMenu.addChoice("Do nothing", AutoStrategy.DO_NOTHING, false);

        moveFoundationMenu.addChoice("Yes", true, true, parkMenu);
        moveFoundationMenu.addChoice("No", false, false, parkMenu);

        parkMenu.addChoice("No Parking", ParkPosition.NO_PARK, false);
        parkMenu.addChoice("Close to wall", ParkPosition.PARK_CLOSE_TO_WALL, true);
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
        autoChoices.robotStartX = robotStartXMenu.getCurrentValue();
        autoChoices.moveFoundation = moveFoundationMenu.getCurrentChoiceObject();
        autoChoices.parkUnderBridge = parkMenu.getCurrentChoiceObject();
        autoChoices.xTarget = xTargetMenu.getCurrentValue();
        autoChoices.yTarget = yTargetMenu.getCurrentValue();
        autoChoices.turnTarget = turnTargetMenu.getCurrentValue();
        autoChoices.driveTime = driveTimeMenu.getCurrentValue();
        autoChoices.drivePower = drivePowerMenu.getCurrentValue();
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(2, "Auto Strategy: %s", strategyMenu.getCurrentChoiceText());
        robot.dashboard.displayPrintf(
                3, "Drive: xTarget=%.0f ft,yTarget=%.0f ft,turnTarget=%.0f ft,Time=%.0f,Power=%.1f",
                autoChoices.xTarget, autoChoices.yTarget, autoChoices.turnTarget, autoChoices.driveTime,
                autoChoices.drivePower);
    }   //doAutoChoicesMenus

    private void createTraceLog()
    {
        String filePrefix = String.format(Locale.US, "%s%02d", matchInfo.matchType, matchInfo.matchNumber);
        robot.globalTracer.openTraceLog("/sdcard/FIRST/tracelog", filePrefix);
    }   //createTraceLog

}   //class CommonAuto
