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

import android.graphics.Rect;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import ftclib.FtcVuforia;
import trclib.TrcDbgTrace;
import trclib.TrcHomographyMapper;

import org.opencv.core.Point;

public class TensorFlowVision
{
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final double TFOD_MIN_CONFIDENCE = 0.8;
    public static final String LABEL_STONE = "Stone";
    public static final String LABEL_SKYSTONE = "Skystone";
    public static final String OPENCV_NATIVE_LIBRARY_NAME = "opencv_java3";

    public class TargetInfo
    {
        String label;
        Rect rect;
        double angle;
        double confidence;
        int imageWidth;
        int imageHeight;
        Point worldCoordinates;

        TargetInfo(String label, Rect rect, double angle, double confidence, int imageWidth, int imageHeight)
        {
            this.label = label;
            this.rect = rect;
            this.angle = angle;
            this.confidence = confidence;
            this.imageWidth = imageWidth;
            this.imageHeight = imageHeight;
            this.worldCoordinates = null;
        }   //TargetInfo

        TargetInfo(String label, Rect rect, double angle, double confidence, int imageWidth, int imageHeight, Point worldCoordinates)
        {
            this.label = label;
            this.rect = rect;
            this.angle = angle;
            this.confidence = confidence;
            this.imageWidth = imageWidth;
            this.imageHeight = imageHeight;
            this.worldCoordinates = worldCoordinates;
        }   //TargetInfo

        @Override
        public String toString()
        {
            if (worldCoordinates == null)
            {
                return String.format(
                        Locale.US, "%s: Rect[%d,%d,%d,%d] angle=%.1f, confidence=%.3f, image(%d,%d)",
                        label, rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top, angle, confidence,
                        imageWidth, imageHeight);
            }
            else
            {
                return String.format(
                        Locale.US, "%s: Rect[%d,%d,%d,%d] IrlPos[%.2f,%.2f] angle=%.1f, confidence=%.3f, image(%d,%d)",
                        label, rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top, worldCoordinates.x, worldCoordinates.y, angle, confidence,
                        imageWidth, imageHeight);
            }

        }
    }   //class TargetInfo

    private TrcDbgTrace tracer;
    private FtcVuforia vuforia;
    private TFObjectDetector tfod;
    private TrcHomographyMapper homographyMapper;

    public TensorFlowVision(int tfodMonitorViewId, VuforiaLocalizer.CameraDirection cameraDir, TrcDbgTrace tracer,
                            double cameraWidth,
                            double cameraHeight,
                            double tl_x,
                            double tl_y,
                            double tr_x,
                            double tr_y,
                            double bl_x,
                            double bl_y,
                            double br_x,
                            double br_y
                            )
    {
        final String VUFORIA_LICENSE_KEY =
                "ATu19Kj/////AAAAGcw4SDCVwEBSiKcUtdmQd2aOugrxo/OgeBJUt7XwMSi3e0KSZaylbsTnWp8EBxyA5o/00JFJVDY1OxJ" +
                "XLxQOpz1tbM4ex1sl1EbF25olEZ3w9xXZ1QaqMP+5T63VqTwvkgKbtM+dS+tLi8EHMvJ2viYf6WwOE776e0s3QNfl/XvONM" +
                "XS4ZtEWLNeiSEMTCdO9bdeaxnSb2RfErcmjadAThDWf6PC9HrMRHLmgfcFaZlj5JN+figOjgKhyQZeYYrcDEm0lICN5kAr2" +
                "pdfNKNOii3A80eXyTVDfPGfzTwVa4eNBY/SgmoIdBbMPb3hfZBOz7GVoVHHQWbCNbzm31p1OY+zqPPWMfzzpyiJ4mA9bLTQ";

        this.tracer = tracer;
        vuforia = new FtcVuforia(VUFORIA_LICENSE_KEY, -1, cameraDir);

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

        System.loadLibrary(OPENCV_NATIVE_LIBRARY_NAME);// initialize OpenCV UwU

        homographyMapper = new TrcHomographyMapper(
                // Camera coordinates: top left, top right, bottom left and bottom right
                new Point(0.0, 0.0), new Point(cameraWidth, 0.0),
                new Point(0.0, cameraHeight), new Point(cameraWidth, cameraHeight),
                // World coordinates: top left, top right, bottom left and bottom right.
                new Point(tl_x, tl_y),
                new Point(tr_x, tr_y),
                new Point(bl_x, bl_y),
                new Point(br_x, br_y));

    }   //TensorFlowVision

    public void setEnabled(boolean enabled)
    {
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
        setEnabled(false);
        tfod.shutdown();
    }   //shutdown

    public void setLightEnabled(boolean enabled)
    {
        vuforia.setFlashlightEnabled(enabled);
    }   //setLightEnabled

    private int compareTargetY(Recognition a, Recognition b)
    {
        return (int)(a.getRight() - b.getRight());
    }   //compareTargetY

    private int compareTargetX(Recognition a, Recognition b)
    {
        return (int)(a.getTop() - b.getTop());
    }   //compareTargetX

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

                if (tracer != null)
                {
                    tracer.traceInfo(funcName, "[%d] %s: x=%.0f, y=%.0f, w=%.0f, h=%.0f, foundIt=%s",
                            i, object.getLabel(), object.getTop(), object.getImageWidth() - object.getRight(),
                            object.getHeight(), object.getWidth(), foundIt);
                }

                if (foundIt)
                {
                    targets.add(object);
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
        Rect targetRect = new Rect(
                (int)target.getTop(), (int)(target.getImageWidth() - target.getRight()),
                (int)target.getHeight(), (int)target.getWidth());
        Point homographyMappedPoint = homographyMapper.mapPoint(new Point((int)target.getTop(), (int)(target.getImageWidth() - target.getRight())));
        TargetInfo targetInfo = new TargetInfo(
                target.getLabel(), targetRect, target.estimateAngleToObject(AngleUnit.DEGREES),
                target.getConfidence(), target.getImageHeight(), target.getImageWidth(), homographyMappedPoint);

        if (tracer != null)
        {
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
