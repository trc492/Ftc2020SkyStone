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

public class RobotInfo
{
    //
    // Game element distances: all referenced as red alliance.
    //
    public static final double ROBOT_START_X_WALL               = 32.0;
    public static final double ROBOT_START_Y_WALL               = 9.0;
    public static final double ROBOT_START_X_FAR                = 52.0;
    public static final double ROBOT_START_Y_FAR                = 9.0;
    public static final double LEFT_STONE_WALL_X                = -(ROBOT_START_X_WALL - 20.5);
    public static final double LEFT_STONE_FAR_X                 = -(ROBOT_START_X_FAR - 44.5);
    public static final double SKYSTONE_SCAN_DISTANCE_WALL      = -9.0;
    public static final double SKYSTONE_SCAN_DISTANCE_FAR       = -18.0;

    // Absolute position waypoint coordinates
    public static final double FOUNDATION_DROP_ABS_POS_X_INCHES = 120.0;
    public static final double WALL_ABS_POS_Y_INCHES            = 9.0;
    public static final double ON_LINE_ABS_POS_X_INCHES         = 69.0;
    public static final double CENTER_FIELD_ABS_POS_Y_INCHES    = 35.0;
    public static final double AVOID_PARTNER_ABS_POS_X_INCHES   = 89.0;

}   //class RobotInfo
