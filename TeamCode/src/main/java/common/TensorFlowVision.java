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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import ftclib.FtcVuforia;
import trclib.TrcDbgTrace;
import trclib.TrcHomographyMapper;

public class TensorFlowVision
{
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final double TFOD_MIN_CONFIDENCE = 0.5;
    private static final double ASPECT_RATIO_TOLERANCE_LOWER = 1.5;
    private static final double ASPECT_RATIO_TOLERANCE_UPPER = 2.5;
    public static final String LABEL_STONE = "Stone";
    public static final String LABEL_SKYSTONE = "Skystone";

    public class TargetInfo
    {
        String label;
        Rect rect;
        double angle;
        double confidence;
        int imageWidth;
        int imageHeight;
        Point targetBottomCenter;

        TargetInfo(String label, Rect rect, double angle, double confidence, int imageWidth, int imageHeight,
                   Point targetBottomCenter)
        {
            this.label = label;
            this.rect = rect;
            this.angle = angle;
            this.confidence = confidence;
            this.imageWidth = imageWidth;
            this.imageHeight = imageHeight;
            this.targetBottomCenter = targetBottomCenter;
        }   //TargetInfo

        @Override
        public String toString()
        {
            return String.format(Locale.US,
                    "%s: Rect[%d,%d,%d,%d] targetPos[%.1f,%.1f] angle=%.1f, confidence=%.1f, image(%d,%d)",
                    label, rect.x, rect.y, rect.width, rect.height, targetBottomCenter.x,
                    targetBottomCenter.y, angle, confidence, imageWidth, imageHeight);
        }
    }   //class TargetInfo

    private FtcVuforia vuforia;
    private TrcDbgTrace tracer;
    private TFObjectDetector tfod;
    private TrcHomographyMapper homographyMapper;
    private boolean useFlashLight = false;

    public TensorFlowVision(
            FtcVuforia vuforia, int tfodMonitorViewId, TrcHomographyMapper.Rectangle cameraRect,
            TrcHomographyMapper.Rectangle worldRect, TrcDbgTrace tracer)
    {
        this.vuforia = vuforia;
        this.tracer = tracer;
        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            TFObjectDetector.Parameters tfodParameters =
                    tfodMonitorViewId == -1?
                            new TFObjectDetector.Parameters() : new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = TFOD_MIN_CONFIDENCE;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia.getLocalizer());
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
        }
        else
        {
            throw new UnsupportedOperationException("This device is not compatible with TensorFlow Object Detection.");
        }

        homographyMapper = new TrcHomographyMapper(cameraRect, worldRect);
    }   //TensorFlowVision

    public void setEnabled(boolean enabled, boolean useFlashLight)
    {
        this.useFlashLight = useFlashLight;
        if (useFlashLight)
        {
            vuforia.setFlashlightEnabled(enabled);
        }

        if (enabled)
        {
            tfod.activate();
        }
        else
        {
            tfod.deactivate();
        }
    }   //setEnabled

    public void shutdown()
    {
        setEnabled(false, useFlashLight);
        tfod.shutdown();
    }   //shutdown

    public void setLightEnabled(boolean enabled)
    {
        vuforia.setFlashlightEnabled(enabled);
    }   //setLightEnabled

    /**
     * This method is called to sort the targets in ascending X order.
     * Note: Phone is in landscape mode, so top is really left.
     *
     * @param a specifies first target.
     * @param b specifes the second target.
     * @return negative value if first target is on the left of the second target, positive if on the right.
     */
    private int compareTargetX(Recognition a, Recognition b)
    {
        return (int)(a.getTop() - b.getTop());
    }   //compareTargetX

    /**
     * This method is called to sort the targets in descending Y order.
     * Note: Phone is in landscape mode, so right is really top.
     *
     * @param a specifies the first target.
     * @param b specifies the second target
     * @return negative if first target below the second target, positive if above.
     */
    private int compareTargetY(Recognition a, Recognition b)
    {
        return (int)(a.getRight() - b.getRight());
    }   //compareTargetY

    private ArrayList<Recognition> getDetectedTargets(String label)
    {
        final String funcName = "getDetectedTargets";
        ArrayList<Recognition> targets = null;
        //
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        //
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null)
        {
            targets = new ArrayList<>();
            for (int i = 0; i < updatedRecognitions.size(); i++)
            {
                Recognition object = updatedRecognitions.get(i);
                boolean foundIt = label == null || label.equals(object.getLabel());
                double aspectRatio = object.getHeight()/object.getWidth();
                boolean rejected = false;

                if (foundIt)
                {
                    if (aspectRatio >= ASPECT_RATIO_TOLERANCE_LOWER && aspectRatio <= ASPECT_RATIO_TOLERANCE_UPPER)
                    {
                        targets.add(object);
                    }
                    else
                    {
                        rejected = true;
                    }
                }

                if (tracer != null)
                {
                    tracer.traceInfo(funcName, "[%d] TensorFlow.%s: x=%.0f, y=%.0f, w=%.0f, h=%.0f, " +
                            "aspectRatio=%.1f, foundIt=%s, rejected=%s",
                            i, object.getLabel(), object.getTop(), object.getImageWidth() - object.getRight(),
                            object.getHeight(), object.getWidth(), aspectRatio, foundIt, rejected);
                }
            }
            //
            // Sort the list in ascending X order so that the left most target will be first.
            //
            if (targets.size() > 1)
            {
                Collections.sort(targets, this::compareTargetX);
            }
            else if (targets.size() == 0)
            {
                //
                // No target found.
                //
                targets = null;
            }
        }

        return targets;
    }   //getDetectedTargets

    public TargetInfo getTargetInfo(Recognition target)
    {
        final String funcName = "getTargetInfo";
        //
        // The phone is set to portrait mode but mounted in counter-clockwise landscape orientation. This means the
        // camera top in portrait mode is actually left in the actual phone orientation.
        //  phone orientation x = camera top
        //  pbone orientation y = camera imageWidth - camera right
        //  phone orientation width = camera height
        //  phone orientation height = camera width
        //
        double imageWidth = target.getImageWidth();
        double imageHeight = target.getImageHeight();
        Rect targetRect = new Rect(
                (int)target.getTop(), (int)(imageWidth - target.getRight()),
                (int)target.getHeight(), (int)target.getWidth());
//        Point targetBottomCenter = homographyMapper.mapPoint(
//                new Point(targetRect.x + targetRect.width/2, targetRect.y + targetRect.height));
        Point targetBottomCenter = new Point(targetRect.x + targetRect.width/2 - imageHeight/2,
                targetRect.y + targetRect.height - imageWidth/2);
        TargetInfo targetInfo = new TargetInfo(
                target.getLabel(), targetRect, target.estimateAngleToObject(AngleUnit.DEGREES),
                target.getConfidence(), target.getImageHeight(), target.getImageWidth(), targetBottomCenter);

        if (tracer != null)
        {
            tracer.traceInfo(funcName, "Target Point: %.0f, %.0f/%.0f, %.0f",
                    targetBottomCenter.x, targetBottomCenter.y, imageHeight, imageWidth);
            tracer.traceInfo(funcName, "###TargetInfo###: %s", targetInfo);
        }

        return targetInfo;
    }   //getTargetInfo

    public TargetInfo[] getDetectedTargetsInfo(String label)
    {
        ArrayList<Recognition> targets = getDetectedTargets(label);
        TargetInfo[] targetsInfo = targets != null && targets.size() > 0 ? new TargetInfo[targets.size()] : null;

        if (targetsInfo != null)
        {
            for (int i = 0; i < targets.size(); i++)
            {
                targetsInfo[i] = getTargetInfo(targets.get(i));
            }
        }

        return targetsInfo;
    }   //getDetectedTargetsInfo

}   //class TensorFlowVision
