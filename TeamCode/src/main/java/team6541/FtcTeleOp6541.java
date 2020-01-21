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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.CommonTeleOp;
import ftclib.FtcGamepad;
import trclib.TrcGameController;
import trclib.TrcRobot;

@TeleOp(name="FtcTeleOp6541", group="FtcTeleOp")
public class FtcTeleOp6541 extends CommonTeleOp
{
    protected Robot6541 robot6541;

    // elbow deployed checker
    //private boolean hasDeployedElbowDefault = false;
    private boolean elbowExtended = false;
    private boolean backLatchExtended = false;
    private boolean frontLatchesExtended = false;

    //
    // Implements FtcOpMode abstract method.
    //

    @Override
    public void initRobot()
    {
        //
        // Initializing robot objects.
        //
        robot6541 = new Robot6541(TrcRobot.getRunMode());
        super.setRobot(robot6541);
        super.initRobot();
    }   //initRobot

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    @Override
    public void runPeriodic(double elapsedTime)
    {
        super.runPeriodic(elapsedTime);
        // CodeReview: why do we deploy elbow here and not in startMode?
        //
        // Operate other team specific subsystems.
        //
        /*
        if (!hasDeployedElbowDefault)
        {
            robot6541.elbow.extend();
            hasDeployedElbowDefault = true;
            elbowExtended = true;
        }

         */

        robot6541.capstoneDeployer.setPosition(1.0 - operatorGamepad.getRightTrigger());
        dashboard.displayPrintf(5, "ElbowPos=%.2f", robot6541.elbow.getPosition());
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
                    if (pressed)
                    {
                        frontLatchesExtended = !frontLatchesExtended;
                        if (frontLatchesExtended)
                        {
                            robot6541.frontFoundationLatch.grab();
                        }
                        else
                        {
                            robot6541.frontFoundationLatch.release();
                        }
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_Y:
                    if (pressed)
                    {
                        backLatchExtended = !backLatchExtended;
                        if (backLatchExtended)
                        {
                            robot6541.backFoundationLatch.grab();
                        }
                        else
                        {
                            robot6541.backFoundationLatch.release();
                        }
                    }
                    processed = true;
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
                        elbowExtended = !elbowExtended;
                        if (elbowExtended)
                        {
                            robot6541.elevator.zeroCalibrate();
                            robot6541.elbow.extend();
                            robot6541.grabber.release();
                        }
                        else
                        {
                            robot6541.elevator.zeroCalibrate();
                            robot6541.grabber.grab();
                            robot6541.elbow.retract();
                        }
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_RBUMPER:
                    break;

                case FtcGamepad.GAMEPAD_DPAD_UP:
                    if (pressed)
                    {
                        robot6541.elevator.levelUp();
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_DPAD_DOWN:
                    if (pressed)
                    {
                        robot6541.elevator.levelDown();
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_DPAD_LEFT:
                    if (pressed)
                    {
                        robot6541.elevator.setLevel(0); // set elevator to base level.
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                    if (pressed)
                    {
                        robot6541.elevator.setLevel(1); // set elevator to first level.
                    }
                    processed = true;
                    break;

                case FtcGamepad.GAMEPAD_BACK:
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

}   //class FtcTeleOp6541
