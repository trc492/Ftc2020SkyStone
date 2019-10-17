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

import java.util.Locale;

/**
 * This class contains variables specifying all the preferences. It is initialized by the corresponding robot.
 */
public class Preferences
{
    public final boolean hasRobot;
    public final boolean hasElevator;
    public final boolean useTraceLog;
    public final boolean useSpeech;
    public final boolean useBatteryMonitor;
    public final boolean useVelocityControl;
    public final boolean useVuforia;
    public final boolean useTensorFlow;
    public final boolean useFlashLight;
    public final boolean showVuforiaView;
    public final boolean showTensorFlowView;

    public Preferences(
            boolean hasRobot, boolean hasElevator, boolean useTraceLog, boolean useSpeech, boolean useBatteryMonitor,
            boolean useVelocityControl, boolean useVuforia, boolean useTensorFlow,
            boolean useFlashLight, boolean showVuforiaView, boolean showTensorFlowView)
    {
        this.hasRobot = hasRobot;
        this.hasElevator = hasElevator;
        this.useTraceLog = useTraceLog;
        this.useSpeech = useSpeech;
        this.useBatteryMonitor = useBatteryMonitor;
        this.useVelocityControl = useVelocityControl;
        this.useVuforia = useVuforia;
        this.useTensorFlow = useTensorFlow;
        this.useFlashLight = useFlashLight;
        this.showVuforiaView = showVuforiaView;
        this.showTensorFlowView = showTensorFlowView;
    }   //Preferences

    public String toString()
    {
        return String.format(Locale.US,
                "hasRobot=%s,hasElevator=%s,useTraceLog=%s,useSpeech=%s,useBatteryMonitor=%s,useVelControl=%s," +
                "useVuforia=%s,useTensorFlow=%s,useFlashLight=%s,showVuforiaView=%s,showTensorFlowView=%s",
                hasRobot, hasElevator, useTraceLog, useSpeech, useBatteryMonitor, useVelocityControl, useVuforia,
                useTensorFlow, useFlashLight, showVuforiaView, showTensorFlowView);
    }   //toString

}   //class Preferences
