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

import ftclib.FtcRevBlinkin;
import trclib.TrcHashMap;
import trclib.TrcRevBlinkin.LEDPattern;

public class LEDIndicator
{
    private static final LEDPattern normalPattern = LEDPattern.FixedBreathRed;
    private static final TrcHashMap<String, LEDPattern> visionPatternMap = new TrcHashMap<String, LEDPattern>()
            .add(VuforiaVision.skystoneTargetName, LEDPattern.SolidGreen)
            .add(VuforiaVision.blueBridgeBackTargetName, LEDPattern.SolidViolet)
            .add(VuforiaVision.redBridgeBackTargetName, LEDPattern.SolidAqua)
            .add(VuforiaVision.redBridgeFrontTargetName, LEDPattern.SolidYellow)
            .add(VuforiaVision.blueBridgeFrontTargetName, LEDPattern.SolidOrange)
            .add(VuforiaVision.red1TargetName, LEDPattern.SolidRed)
            .add(VuforiaVision.red2TargetName, LEDPattern.FixedStrobeRed)
            .add(VuforiaVision.front1TargetName, LEDPattern.SolidGold)
            .add(VuforiaVision.front2TargetName, LEDPattern.FixedStrobeGold)
            .add(VuforiaVision.blue1TargetName, LEDPattern.SolidBlue)
            .add(VuforiaVision.blue2TargetName, LEDPattern.FixedStrobeBlue)
            .add(VuforiaVision.back1TargetName, LEDPattern.SolidWhite)
            .add(VuforiaVision.back2TargetName, LEDPattern.FixedStrobeWhite);

    private FtcRevBlinkin blinkin;

    public LEDIndicator()
    {
        blinkin = new FtcRevBlinkin("blinkin");
        reset();
    }   //LEDIndicator

    public void reset()
    {
        blinkin.setPattern(normalPattern);
    }   //reset

    public void setDetectedTarget(String targetName)
    {
        blinkin.setPattern(visionPatternMap.get(targetName));
    }   //setDetectedTarget

}   //class LEDIndicator
