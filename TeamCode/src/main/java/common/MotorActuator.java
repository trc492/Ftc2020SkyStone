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
import trclib.TrcPidActuator;
import trclib.TrcPidController;

public class MotorActuator
{
    public static class Parameters
    {
        double minPos, maxPos;
        double scale, offset;
        double kP, kI, kD, tolerance;
        boolean inverted;
        boolean hasUpperLimitSwitch;
        double calPower;
        double[] posPresets;

        public Parameters setPosRange(double minPos, double maxPos)
        {
            this.minPos = minPos;
            this.maxPos = maxPos;
            return this;
        }

        public Parameters setScaleOffset(double scale, double offset)
        {
            this.scale = scale;
            this.offset = offset;
            return this;
        }

        public Parameters setPidParams(double kP, double kI, double kD, double tolerance)
        {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.tolerance = tolerance;
            return this;
        }

        public Parameters setMotorParams(boolean inverted, boolean hasUpperLimitSwitch, double calPower)
        {
            this.inverted = inverted;
            this.hasUpperLimitSwitch = hasUpperLimitSwitch;
            this.calPower = calPower;
            return this;
        }

        public Parameters setPosPresets(double... posPresets)
        {
            this.posPresets = posPresets;
            return this;
        }

    }   //class Parameters

    private FtcDigitalInput lowerLimitSwitch;
    private FtcDigitalInput upperLimitSwitch;
    private TrcPidActuator pidActuator;

    private double[] posPresets;
    private int posLevel;

    public MotorActuator(String instanceName, Parameters params)
    {
        lowerLimitSwitch = new FtcDigitalInput(instanceName + "LowerLimit");
        if (params.hasUpperLimitSwitch)
        {
            upperLimitSwitch = new FtcDigitalInput(instanceName + "UpperLimit");
        }

        FtcDcMotor actuatorMotor = new FtcDcMotor(
                instanceName + "Motor", lowerLimitSwitch, upperLimitSwitch);
        actuatorMotor.setBrakeModeEnabled(true);
        actuatorMotor.setOdometryEnabled(true);
        actuatorMotor.setInverted(params.inverted);

        TrcPidController pidController = new TrcPidController(
                instanceName + "PidController",
                new TrcPidController.PidCoefficients(params.kP, params.kI, params.kD),
                params.tolerance, this::getPosition);

        pidActuator = new TrcPidActuator(
                "pid" + instanceName, actuatorMotor, lowerLimitSwitch, pidController,
                params.calPower, params.minPos, params.maxPos);
        pidActuator.setPositionScale(params.scale, params.offset);

        this.posPresets = params.posPresets;
        this.posLevel = 0;
    }   //MotorActuator

    public void zeroCalibrate()
    {
        pidActuator.zeroCalibrate();
    }   //zeroCalibrate

    public void setManualOverride(boolean enabled)
    {
        pidActuator.setManualOverride(enabled);
    }   //setManualOverride

    public void setPower(double power)
    {
        pidActuator.setPower(power, true);
    }   //setPower

    public void setPosition(double target, TrcEvent event, double timeout)
    {
        pidActuator.setTarget(target, event, timeout);
    }   //setPosition

    public void setPosition(double target)
    {
        pidActuator.setTarget(target, true);
    }   //setPosition

    public void setLevel(int level)
    {
        if (posPresets != null)
        {
            if (level < 0)
            {
                posLevel = 0;
            }
            else if (level >= posPresets.length)
            {
                posLevel = posPresets.length - 1;
            }
            else
            {
                posLevel = level;
            }

            setPosition(posPresets[posLevel]);
        }
    }   //setLevel

    public void levelUp()
    {
        setLevel(posLevel + 1);
    }   //levelUp

    public void levelDown()
    {
        setLevel(posLevel - 1);
    }   //levelDown

    public int getLevel()
    {
        return posLevel;
    }   //getLevel

    public double getPosition()
    {
        return pidActuator.getPosition();
    }   //getPosition

    public boolean isLowerLimitSwitchActive()
    {
        return lowerLimitSwitch.isActive();
    }   //isLowerLimitSwitchActive

    public boolean isUpperLimitSwitchActive()
    {
        return upperLimitSwitch != null ? upperLimitSwitch.isActive() : false;
    }   //isUpperLimitSwitchActive

}   //class MotorActuator
