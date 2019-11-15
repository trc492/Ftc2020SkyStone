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

import ftclib.FtcDcMotor;
import ftclib.FtcDigitalInput;
import trclib.TrcEvent;
import trclib.TrcHashMap;
import trclib.TrcPidActuator;
import trclib.TrcPidController;

public class Elevator
{
    private FtcDigitalInput lowerLimitSwitch;
    private FtcDigitalInput upperLimitSwitch;
    private TrcPidActuator pidElevator;

    private double[] elevatorHeightPresets;
    private int elevatorLevel;

    public Elevator(
            TrcHashMap<String, Boolean> preferences, TrcHashMap<String, Double> params,
            double[] elevatorHeightPresets)
    {
        boolean isTeam3543 = preferences.getBoolean("team3543");

        lowerLimitSwitch = new FtcDigitalInput("elevatorLowerLimit");
        if (!isTeam3543)
        {
            upperLimitSwitch = new FtcDigitalInput("elevatorUpperLimit");
        }

        lowerLimitSwitch.setInverted(false);
        if (upperLimitSwitch != null)
        {
            upperLimitSwitch.setInverted(false);
        }

        FtcDcMotor elevatorMotor = new FtcDcMotor("elevatorMotor", lowerLimitSwitch, upperLimitSwitch);
        elevatorMotor.setBrakeModeEnabled(true);
        elevatorMotor.setOdometryEnabled(true);
        if (isTeam3543)
        {
            elevatorMotor.setInverted(true);
        }

        TrcPidController pidController = new TrcPidController(
                "elevatorPidController",
                new TrcPidController.PidCoefficients(params.get("Kp"), params.get("Ki"), params.get("Kd")),
                params.get("tolerance"), this::getPosition);

        pidElevator = new TrcPidActuator(
                "pidElevator", elevatorMotor, lowerLimitSwitch, pidController,
                params.get("calPower"), params.get("minHeight"), params.get("maxHeight"));
        pidElevator.setPositionScale(params.get("scale"), params.get("offset"));

        this.elevatorHeightPresets = elevatorHeightPresets;
        this.elevatorLevel = 0;
    }   //Elevator

    public void zeroCalibrate()
    {
        pidElevator.zeroCalibrate();
    }   //zeroCalibrate

    public void setManualOverride(boolean enabled)
    {
        pidElevator.setManualOverride(enabled);
    }   //setManualOverride

    public void setPower(double power)
    {
        pidElevator.setPower(power, true);
    }   //setPower

    public void setPosition(double target, TrcEvent event, double timeout)
    {
        pidElevator.setTarget(target, event, timeout);
    }   //setPosition

    public void setPosition(double target)
    {
        pidElevator.setTarget(target, true);
    }   //setPosition

    public void setLevel(int level)
    {
        if (elevatorHeightPresets != null)
        {
            if (level < 0)
            {
                elevatorLevel = 0;
            }
            else if (level >= elevatorHeightPresets.length)
            {
                elevatorLevel = elevatorHeightPresets.length - 1;
            }
            else
            {
                elevatorLevel = level;
            }

            setPosition(elevatorHeightPresets[elevatorLevel]);
        }
    } // setLevel

    public void levelUp()
    {
        setLevel(elevatorLevel + 1);
    } // levelUp

    public void levelDown()
    {
        setLevel(elevatorLevel - 1);
    } // levelDown

    public int getLevel()
    {
        return elevatorLevel;
    }

    public double getPosition()
    {
        return pidElevator.getPosition();
    }   //getPosition

    public boolean isLowerLimitSwitchActive()
    {
        return lowerLimitSwitch.isActive();
    }   //isLowerLimitSwitchActive

    public boolean isUpperLimitSwitchActive()
    {
        return upperLimitSwitch != null ? upperLimitSwitch.isActive() : false;
    }   //isUpperLimitSwitchActive

}   //class Elevator
