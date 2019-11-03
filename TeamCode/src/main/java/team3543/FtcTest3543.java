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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.CommonTest;
import ftclib.FtcGamepad;
import trclib.TrcGameController;
import trclib.TrcPidController;
import trclib.TrcRobot;

@TeleOp(name="FtcTest3543", group="FtcTest")
public class FtcTest3543 extends FtcTeleOp3543
{
    private static final String MODULE_NAME = "FtcTest3543";
    private CommonTest commonTest = new CommonTest();

    //
    // Overrides FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // TeleOp initialization.
        //
        super.initRobot();

        moduleName = MODULE_NAME;
        commonTest.init(MODULE_NAME, robot,
                new TrcPidController.PidCoefficients(
                        RobotInfo3543.PURE_PURSUIT_POS_KP, RobotInfo3543.PURE_PURSUIT_POS_KI,
                        RobotInfo3543.PURE_PURSUIT_POS_KD),
                new TrcPidController.PidCoefficients(
                        RobotInfo3543.PURE_PURSUIT_TURN_KP, RobotInfo3543.PURE_PURSUIT_TURN_KI,
                        RobotInfo3543.PURE_PURSUIT_TURN_KD),
                new TrcPidController.PidCoefficients(
                        RobotInfo3543.PURE_PURSUIT_VEL_KP, RobotInfo3543.PURE_PURSUIT_VEL_KI,
                        RobotInfo3543.PURE_PURSUIT_VEL_KD, RobotInfo3543.PURE_PURSUIT_VEL_KF));
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        super.startMode(prevMode, nextMode);
        commonTest.start();
    }   //startMode

    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        commonTest.stop();
        super.stopMode(prevMode, nextMode);
    }   //stopMode

    @Override
    public void runPeriodic(double elapsedTime)
    {
        if (commonTest.shouldRunTeleOpPeriodic())
        {
            super.runPeriodic(elapsedTime);
        }
        commonTest.runPeriodic(elapsedTime);
    }   //runPeriodic

    @Override
    public void runContinuous(double elapsedTime)
    {
        commonTest.runContinuous(elapsedTime);
    }   //runContinuous

    //
    // Overrides TrcGameController.ButtonHandler in FtcTeleOp3543.
    //

    @Override
    public void buttonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        boolean processed = false;
        //
        // In addition to or instead of the gamepad controls handled by FtcTeleOp3543, we can add to or override the
        // FtcTeleOp3543 gamepad actions.
        //
        dashboard.displayPrintf(
                7, "%s: %04x->%s", gamepad.toString(), button, pressed? "Pressed": "Released");
        if (gamepad == driverGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_DPAD_UP:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    break;
            }
        }
        else if (gamepad == operatorGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_DPAD_UP:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    break;
            }
        }
        //
        // If the control was not processed by this method, pass it back to FtcTeleOp3543.
        //
        if (!processed)
        {
            super.buttonEvent(gamepad, button, pressed);
        }
    }   //buttonEvent

}   //class FtcTest3543
