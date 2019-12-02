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

import common.CommonTeleOp;
import ftclib.FtcGamepad;
import trclib.TrcGameController;
import trclib.TrcRobot;

@TeleOp(name="FtcTeleOp3543", group="FtcTeleOp")
public class FtcTeleOp3543 extends CommonTeleOp
{
    protected Robot3543 robot3543;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //
        robot3543 = new Robot3543(TrcRobot.getRunMode());
        super.setRobot(robot3543);
        super.initRobot();
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void runPeriodic(double elapsedTime)
    {
        super.runPeriodic(elapsedTime);
        //
        // Operate other team specific subsystems.
        //
        double extenderArmPower = operatorGamepad.getLeftStickY(true);
        double extenderArmPos;
        boolean extenderArmLimitSwitch;

        robot3543.extenderArm.setPower(extenderArmPower);
        extenderArmPos = robot3543.extenderArm.getPosition();
        extenderArmLimitSwitch = robot3543.extenderArm.isLowerLimitSwitchActive();

        double wristPower = operatorGamepad.getLeftStickX(true) / 3.0;
        double wristPos = 0.0;
        if (robot3543.wrist != null)
        {
            robot3543.wrist.setPower(wristPower);
            wristPos = robot3543.wrist.getPosition();
        }

        double frontLatchPos = 0.0;
        if (robot3543.frontFoundationLatch != null)
        {
            frontLatchPos = robot3543.frontFoundationLatch.getPosition();
        }

        dashboard.displayPrintf(5,
                "ArmPower=%.1f, ArmPos=%.1f, ArmLimit=%s, WristPower=%.1f, WristPos =%.1f, FrontLatchPos=%.1f",
                extenderArmPower, extenderArmPos, extenderArmLimitSwitch, wristPower, wristPos, frontLatchPos);
    }   //runPeriodic

    //
    // Implements TrcGameController.ButtonHandler interface.
    //

    @Override
    public void buttonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        boolean processed = false;

        dashboard.displayPrintf(
                7, "%s: %04x->%s", gamepad.toString(), button, pressed? "Pressed": "Released");

        if (gamepad == driverGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    break;
            }
        }
        else if (gamepad == operatorGamepad)
        {
            switch (button)
            {
                case FtcGamepad.GAMEPAD_A:
                    break;

                case FtcGamepad.GAMEPAD_B:
                    break;

                case FtcGamepad.GAMEPAD_X:
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    break;

                case FtcGamepad.GAMEPAD_LBUMPER:
                    if (pressed)
                    {
                        robot3543.frontFoundationLatch.grab();
                    }
                    else
                    {
                        robot3543.frontFoundationLatch.release();
                    }
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    robot3543.extenderArm.setManualOverride(pressed);
                    break;

                case FtcGamepad.GAMEPAD_DPAD_UP:
                    if (pressed)
                    {
                        robot3543.extenderArm.setPosition(RobotInfo3543.EXTENDER_ARM_EXTEND_POS);
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    if (pressed)
                    {
                        robot3543.extenderArm.setPosition(RobotInfo3543.EXTENDER_ARM_RETRACT_POS);
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    if (pressed)
                    {
                        robot3543.wrist.retract();
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    if (pressed)
                    {
                        robot3543.wrist.extend();
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_BACK:
                    if (pressed)
                    {
                        robot3543.extenderArm.zeroCalibrate();
                    }
                    break;

                case FtcGamepad.GAMEPAD_START:
                    break;
            }
        }

        if (!processed)
        {
            super.buttonEvent(gamepad, button, pressed);
        }
    }   //buttonEvent

}   //class FtcTeleOp3543
