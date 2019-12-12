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

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import ftclib.FtcVuforia;
import trclib.TrcPose2D;
import trclib.TrcUtil;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class VuforiaVision
{
    public static final String skystoneTargetName = "Stone Target";
    public static final String blueBridgeBackTargetName = "Blue Rear Bridge";
    public static final String redBridgeBackTargetName = "Red Rear Bridge";
    public static final String redBridgeFrontTargetName = "Red Front Bridge";
    public static final String blueBridgeFrontTargetName = "Blue Front Bridge";
    public static final String red1TargetName = "Red Perimeter 1";
    public static final String red2TargetName = "Red Perimeter 2";
    public static final String front1TargetName = "Front Perimeter 1";
    public static final String front2TargetName = "Front Perimeter 2";
    public static final String blue1TargetName = "Blue Perimeter 1";
    public static final String blue2TargetName = "Blue Perimeter 2";
    public static final String back1TargetName = "Rear Perimeter 1";
    public static final String back2TargetName = "Rear Perimeter 2";
    public static final double HALF_FIELD_INCHES = 72.0f;
    public static final double QUAD_FIELD_INCHES = 36.0f;

    private static final int IMAGE_WIDTH = 1280;    //in pixels
    private static final int IMAGE_HEIGHT = 720;    //in pixels
    private static final int FRAME_QUEUE_CAPACITY = 2;
    //
    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here.
    //

    // Height of the center of the target image above the floor.
    private static final float mmTargetHeight = 6.0f * (float)TrcUtil.MM_PER_INCH;

    // Constant for Stone Target.
    private static final float stoneZ = 2.0f * (float)TrcUtil.MM_PER_INCH;

    // Constants for the center support targets.
    private static final float bridgeZ = 6.42f * (float)TrcUtil.MM_PER_INCH;
    private static final float bridgeY = 23.0f * (float)TrcUtil.MM_PER_INCH;
    private static final float bridgeX = 5.18f * (float)TrcUtil.MM_PER_INCH;
    private static final float bridgeRotY = 59.0f;  // Units are degrees
    private static final float bridgeRotZ = 180.0f;

    // Constants for perimeter targets
    private static final float halfField = (float)(HALF_FIELD_INCHES * TrcUtil.MM_PER_INCH);
    private static final float quadField = (float)(QUAD_FIELD_INCHES * TrcUtil.MM_PER_INCH);

    private Robot robot;
    private FtcVuforia vuforia;
    private VuforiaTrackable[] imageTargets;
    private String lastImageName = null;

    public VuforiaVision(Robot robot, FtcVuforia vuforia, OpenGLMatrix phoneLocation)
    {
        final String TRACKABLE_IMAGES_FILE = "Skystone";

        this.robot = robot;
        this.vuforia = vuforia;
        vuforia.configVideoSource(IMAGE_WIDTH, IMAGE_HEIGHT, FRAME_QUEUE_CAPACITY);

        /*
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        OpenGLMatrix stoneTargetLocation = OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));

        //Set the position of the bridge support targets with relation to origin (center of field)
        OpenGLMatrix blueFrontBridgeLocation = OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ));

        OpenGLMatrix blueRearBridgeLocation = OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ));

        OpenGLMatrix redFrontBridgeLocation = OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0));

        OpenGLMatrix redRearBridgeLocation = OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0));

        //Set the position of the perimeter targets with relation to origin (center of field)
        OpenGLMatrix redPerimeter1Location = OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));

        OpenGLMatrix redPerimeter2Location = OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));

        OpenGLMatrix frontPerimeter1Location = OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));

        OpenGLMatrix frontPerimeter2Location = OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));

        OpenGLMatrix bluePerimeter1Location = OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));

        OpenGLMatrix bluePerimeter2Location = OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));

        OpenGLMatrix rearPerimeter1Location = OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90));

        OpenGLMatrix rearPerimeter2Location = OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));

        FtcVuforia.TargetInfo[] imageTargetsInfo =
        {
                new FtcVuforia.TargetInfo(0, skystoneTargetName, false, stoneTargetLocation),
                new FtcVuforia.TargetInfo(1, blueBridgeBackTargetName, false, blueRearBridgeLocation),
                new FtcVuforia.TargetInfo(2, redBridgeBackTargetName, false, redRearBridgeLocation),
                new FtcVuforia.TargetInfo(3, redBridgeFrontTargetName, false, redFrontBridgeLocation),
                new FtcVuforia.TargetInfo(4, blueBridgeFrontTargetName, false, blueFrontBridgeLocation),
                new FtcVuforia.TargetInfo(5, red1TargetName, false, redPerimeter1Location),
                new FtcVuforia.TargetInfo(6, red2TargetName, false, redPerimeter2Location),
                new FtcVuforia.TargetInfo(7, front1TargetName, false, frontPerimeter1Location),
                new FtcVuforia.TargetInfo(8, front2TargetName, false, frontPerimeter2Location),
                new FtcVuforia.TargetInfo(9, blue1TargetName, false, bluePerimeter1Location),
                new FtcVuforia.TargetInfo(10, blue2TargetName, false, bluePerimeter2Location),
                new FtcVuforia.TargetInfo(11, back1TargetName, false, rearPerimeter1Location),
                new FtcVuforia.TargetInfo(12, back2TargetName, false, rearPerimeter2Location)
        };

        vuforia.addTargetList(TRACKABLE_IMAGES_FILE, imageTargetsInfo, phoneLocation);
        imageTargets = new VuforiaTrackable[imageTargetsInfo.length];
        for (int i = 0; i < imageTargets.length; i++)
        {
            imageTargets[i] = vuforia.getTarget(imageTargetsInfo[i].name);
        }
    }   //VuforiaVision

    public void setEnabled(boolean enabled)
    {
        vuforia.setTrackingEnabled(enabled);
    }   //setEnabled

    public String getLastSeenImageName()
    {
        return lastImageName;
    }   //getLastSeenImageName

    public VectorF getLocationTranslation(OpenGLMatrix location)
    {
        return location.getTranslation();
    }   //getLocationTranslation

    public Orientation getLocationOrientation(OpenGLMatrix location)
    {
        return Orientation.getOrientation(location, EXTRINSIC, XYZ, DEGREES);
    }   //getLocationOrientation

    public OpenGLMatrix getRobotLocation(String targetName)
    {
        OpenGLMatrix robotLocation = null;
        VuforiaTrackable target = vuforia.getTarget(targetName);

        if (target != null)
        {
            robotLocation = vuforia.getRobotLocation(target);

            if (robot.ledIndicator != null && !robot.preferences.useBlinkinFlashLight)
            {
                if (robotLocation != null)
                {
                    robot.ledIndicator.setDetectedTarget(targetName);
                }
                else
                {
                    robot.ledIndicator.reset();
                }
            }
        }

        return robotLocation;
    }   //getRobotLocation

    public OpenGLMatrix getRobotLocation(String targetName, boolean exclude)
    {
        OpenGLMatrix robotLocation = null;

        if (targetName == null || exclude)
        {
            for (VuforiaTrackable target: imageTargets)
            {
                String name = target.getName();
                boolean isMatched = targetName == null || !targetName.equals(name);

                if (isMatched && vuforia.isTargetVisible(target))
                {
                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix location = vuforia.getRobotLocation(target);
                    if (location != null)
                    {
                        robotLocation = location;
                        lastImageName = name;
                    }
                    break;
                }
            }
        }
        else
        {
            robotLocation = getRobotLocation(targetName);
            if (robotLocation != null)
            {
                lastImageName = targetName;
            }
        }

        if (robot.ledIndicator != null && !robot.preferences.useBlinkinFlashLight)
        {
            if (robotLocation != null)
            {
                robot.ledIndicator.setDetectedTarget(lastImageName);
            }
            else
            {
                robot.ledIndicator.reset();
            }
        }

        return robotLocation;
    }   //getRobotLocation

    public TrcPose2D getRobotPose(String targetName, boolean exclude)
    {
        OpenGLMatrix robotLocation = getRobotLocation(targetName, exclude);
        VectorF translation = robotLocation == null? null: getLocationTranslation(robotLocation);
        Orientation orientation = robotLocation == null? null: getLocationOrientation(robotLocation);
        //
        // The returned RobotPose have the X axis pointing from the audience side to the back of the field,
        // the Y axis pointing from the red alliance to the blue alliance and the direction of the Y axis
        // is zero degree and increases in the clockwise direction.
        //
        TrcPose2D robotPose = (translation == null || orientation == null)? null:
                                new TrcPose2D(translation.get(0)/TrcUtil.MM_PER_INCH,
                                              translation.get(1)/TrcUtil.MM_PER_INCH,
                                              -orientation.thirdAngle + 90.0);

        return robotPose;
    }   //getRobotPose

}   //class VuforiaVision
