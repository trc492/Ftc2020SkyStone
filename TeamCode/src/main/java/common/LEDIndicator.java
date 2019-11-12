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
            .add("Blue Rear Bridge", LEDPattern.SolidViolet)
            .add("Red Rear Bridge", LEDPattern.SolidAqua)
            .add("Red Front Bridge", LEDPattern.SolidYellow)
            .add("Blue Front Bridge", LEDPattern.SolidOrange)
            .add("Red Perimeter 1", LEDPattern.SolidRed)
            .add("Red Perimeter 2", LEDPattern.FixedStrobeRed)
            .add("Front Perimeter 1", LEDPattern.SolidGold)
            .add("Front Perimeter 2", LEDPattern.FixedStrobeGold)
            .add("Blue Perimeter 1", LEDPattern.SolidBlue)
            .add("Blue Perimeter 2", LEDPattern.FixedStrobeBlue)
            .add("Rear Perimeter 1", LEDPattern.SolidWhite)
            .add("Rear Perimeter 2", LEDPattern.FixedStrobeWhite);

    private FtcRevBlinkin blinkin;

    public LEDIndicator()
    {
        blinkin = new FtcRevBlinkin("LEDIndicator");
        blinkin.setPatternState(normalPattern, true);
    }   //LEDIndicator

    public void reset()
    {
        blinkin.resetAllPatternStates();
        blinkin.setPatternState(normalPattern, true);
    }   //reset

    public void setDetectedTarget(String targetName)
    {
        reset();
        blinkin.setPatternState(visionPatternMap.get(targetName), true);
    }   //setDetectedTarget

}   //class LEDIndicator
