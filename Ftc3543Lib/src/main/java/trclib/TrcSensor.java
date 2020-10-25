/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

package trclib;

import java.util.Locale;

/**
 * This class implements a platform independent generic sensor that has one or more axes. Typically, this class is
 * extended by a platform dependent sensor class. This class also allows the caller to specify a number of data
 * processor options, one of which is to register an array of filters, one for each axis. Another one is to adjust
 * the data by its calibration. It also allows the caller to specify scale and offset adjustment to the data as well
 * as the inverting sensor data.
 *
 * @param <S> specifies the sensor data selector enum. A sensor may report multiple data for each axis. Data selector
 *            enum is used to select what data to process.
 */
public class TrcSensor<S>
{
    protected static final String moduleName = "TrcSensor";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This class implements the SensorData object that consists of the sensor data as well as a timestamp when the
     * data sample is taken.
     *
     * @param <T> specifies the sensor data type. It could be integer, double, enum, object or even an array of types.
     */
    public static class SensorData<T> implements Cloneable
    {
        public double timestamp;
        public T data;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param timestamp specifies the timestamp of the sensor data.
         * @param data specifies the sensor data.
         */
        public SensorData(double timestamp, T data)
        {
            this.timestamp = timestamp;
            this.data = data;
        }   //SensorData

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param data specifies the sensor data.
         */
        public SensorData(T data)
        {
            this.timestamp = TrcUtil.getCurrentTime();
            this.data = data;
        }   //SensorData

        /**
         * Constructor: Creates an instance of the object.
         */
        public SensorData()
        {
            this.timestamp = 0.0;
            this.data = null;
        }   //SensorData

        /**
         * This method returns the string format of the object data.
         *
         * @return string form of the object data.
         */
        @Override
        public String toString()
        {
            return String.format(Locale.US, "(timestamp=%.3f,data=%s)", timestamp, data);
        }   //toString

        @Override
        public SensorData clone() throws CloneNotSupportedException
        {
            
        }   //clone

    }   //class SensorData

    /**
     * This class stores the parameters for the sensor data processor that control how the sensor data is processed.
     * Some parameters is an array for which each element is for each axis of the sensor.
     */
    private class ProcessorParams
    {
        TrcSensorCalibrator<S> calibrator;
        boolean calibrating;
        TrcFilter[] filters;
        double[] scales;
        double[] offsets;
        int[] signs;
    }   //class ProcessorParams

    private static final int NUM_CAL_SAMPLES = 100;
    private static final long CAL_INTERVAL = 10;    //in msec.

    protected final String instanceName;
    private final int numAxes;
    private final TrcHashMap<S, ProcessorParams> processorParamsMap = new TrcHashMap<>();

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numAxes specifies the number of axes.
     */
    public TrcSensor(String instanceName, int numAxes)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }
        //
        // Make sure we have at least one axis.
        //
        if (numAxes <= 0)
        {
            throw new IllegalArgumentException("Sensor must have at least one axis.");
        }

        this.instanceName = instanceName;
        this.numAxes = numAxes;
    }   //TrcSensor

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method returns the number of axes of the sensor.
     *
     * @return number of axes.
     */
    public int getNumAxes()
    {
        final String funcName = "getNumAxes";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", numAxes);
        }

        return numAxes;
    }   //getNumAxes

    /**
     * This method creates a processor parameters object for the sensor data type.
     *
     * @param dataSelector specifies the data selector enum for selecting the sensor data.
     * @return newly created processor parameters object
     */
    private ProcessorParams createProcessorParams(S dataSelector)
    {
        ProcessorParams processorParams = new ProcessorParams();

        processorParams.calibrator = null;
        processorParams.calibrating = false;
        processorParams.filters = new TrcFilter[numAxes];
        processorParams.scales = new double[numAxes];
        processorParams.offsets = new double[numAxes];
        processorParams.signs = new int[numAxes];
        for (int i = 0; i < numAxes; i++)
        {
            processorParams.scales[i] = 1.0;
            processorParams.offsets[i] = 0.0;
            processorParams.signs[i] = 1;
        }
        processorParamsMap.add(dataSelector, processorParams);

        return processorParams;
    }   //createProcessorParams

    /**
     * This method sets a filter for the data of the specified axis. This is useful if the data is susceptible to noise,
     * for example.
     *
     * @param index specifies the axis index.
     * @param dataSelector specifies the data selector for selecting the sensor data.
     * @param filter specifies filter to be used on the data.
     */
    public void setFilter(int index, S dataSelector, TrcFilter filter)
    {
        final String funcName = "setFilter";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                funcName, TrcDbgTrace.TraceLevel.API, "index=%d,selector=%s,filter=%s",
                index, dataSelector, filter);
        }

        ProcessorParams processorParams;
        try
        {
            processorParams = processorParamsMap.get(dataSelector);
        }
        catch (IllegalArgumentException e)
        {
            // There is no processor for the specified data selector, so create one.
            processorParams = createProcessorParams(dataSelector);
        }

        synchronized (processorParams)
        {
            processorParams.filters[index] = filter;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setFilter

    /**
     * This method calibrates the sensor by creating a calibrator if none exist yet. It then calls the calibrator
     * to do the calibration. Note: this is a synchronous method so it won't return until the sensor calibration is
     * done.
     *
     * @param dataSource specifies the DataSource object to get sensor data.
     * @param dataSelector specifies the data selector for selecting the sensor data.
     * @param numCalSamples specifies the number of calibration sample to take.
     * @param calInterval specifies the interval between each calibration sample in msec.
     */
    protected void calibrate(
            TrcSensorCalibrator.InputSource<S> dataSource, S dataSelector, int numCalSamples, long calInterval)
    {
        final String funcName = "calibrate";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                funcName, TrcDbgTrace.TraceLevel.API,
                "selector=%s,numSamples=%d,calInterval=%d", dataSelector, numCalSamples, calInterval);
        }

        ProcessorParams processorParams;
        try
        {
            processorParams = processorParamsMap.get(dataSelector);
        }
        catch (IllegalArgumentException e)
        {
            processorParams = createProcessorParams(dataSelector);
        }

        synchronized (processorParams)
        {
            if (processorParams.calibrator == null)
            {
                processorParams.calibrator = new TrcSensorCalibrator<>(instanceName, numAxes);
            }

            processorParams.calibrating = true;
            processorParams.calibrator.calibrate(dataSource, dataSelector, numCalSamples, calInterval);
            processorParams.calibrating = false;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //calibrate

    /**
     * This method calibrates the sensor by creating a calibrator if none exist yet. It then calls the calibrator
     * to do the calibration. Note: this is a synchronous method so it won't return until the sensor calibration is
     * done.
     *
     * @param dataSource specifies the DataSource object to get sensor data.
     * @param dataSelector specifies the data selector for selecting the sensor data.
     */
    protected void calibrate(TrcSensorCalibrator.InputSource<S> dataSource, S dataSelector)
    {
        calibrate(dataSource, dataSelector, NUM_CAL_SAMPLES, CAL_INTERVAL);
    }   //calibrate

    /**
     * This method always returns false because the built-in calibrator is synchronous.
     *
     * @param dataSelector specifies the data selector for selecting the sensor data to check for calibration.
     * @return true if the specified sensor data is being calibrated, false otherwise.
     */
    public boolean isCalibrating(S dataSelector)
    {
        final String funcName = "isCalibrating";
        boolean calibrating;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "selector=%s", dataSelector);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=false");
        }

        ProcessorParams processorParams;
        try
        {
            processorParams = processorParamsMap.get(dataSelector);
            synchronized (processorParams)
            {
                calibrating = processorParams.calibrating;
            }
        }
        catch (IllegalArgumentException e)
        {
            calibrating = false;
        }

        return calibrating;
    }   //isCalibrating

    /**
     * This method sets the scale factor and offset for the data of the specified axis.
     *
     * @param index specifies the axis index.
     * @param dataSelector specifies the data selector for selecting the sensor data.
     * @param scale specifies the scale factor for the axis.
     * @param offset specifies the offset to be subtracted from the scaled data.
     */
    public void setScaleOffset(int index, S dataSelector, double scale, double offset)
    {
        final String funcName = "setScaleOffset";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                funcName, TrcDbgTrace.TraceLevel.API,
                "index=%d,selector=%s,scale=%f,offset=%f", index, dataSelector, scale, offset);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        ProcessorParams processorParams;
        try
        {
            processorParams = processorParamsMap.get(dataSelector);
        }
        catch (IllegalArgumentException e)
        {
            processorParams = createProcessorParams(dataSelector);
        }

        synchronized (processorParams)
        {
            processorParams.scales[index] = scale;
            processorParams.offsets[index] = offset;
        }
    }   //setScaleOffset

    /**
     * This method inverts the data of the specified axis of the sensor. This is useful if the orientation of the
     * sensor axis is such that the data goes the wrong direction, if the sensor is mounted up-side-down, for example.
     *
     * @param index specifies the axis index.
     * @param dataSelector specifies the data selector for selecting the sensor data.
     * @param inverted specifies true to invert the axis, false otherwise.
     */
    public void setInverted(int index, S dataSelector, boolean inverted)
    {
        final String funcName = "setInverted";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API, "index=%d,selector=%s,inverted=%s",
                    index, dataSelector, inverted);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        ProcessorParams processorParams;
        try
        {
            processorParams = processorParamsMap.get(dataSelector);
        }
        catch (IllegalArgumentException e)
        {
            processorParams = createProcessorParams(dataSelector);
        }

        synchronized (processorParams)
        {
            processorParams.signs[index] = inverted? -1: 1;
        }
    }   //setInverted

    /**
     * This method returns the processed sensor data for the specified axis. The data will go through a filter if a
     * filter is supplied for the axis. The calibration data will be applied to the sensor data if applicable. The
     * sign and scale/offset will also be applied.
     *
     * @param data specifies the raw data to be processed.
     * @param index specifies which axis of the data processor to use to process the data.
     * @param dataSelector specifies which data processor to use to proecess the data.
     * @return processed sensor data for the axis.
     */
    protected double processData(double data, int index, S dataSelector)
    {
        final String funcName = "processData";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.API,
                    "data=%f,index=%d,selector=%s", data, index, dataSelector);
            dbgTrace.traceInfo(funcName, "raw=%.3f", data);
        }

        ProcessorParams processorParams;
        try
        {
            processorParams = processorParamsMap.get(dataSelector);
            synchronized (processorParams)
            {
                if (processorParams.filters[index] != null)
                {
                    data = processorParams.filters[index].filterData(data);
                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "filtered=%.3f", data);
                    }
                }

                if (processorParams.calibrator != null)
                {
                    data = processorParams.calibrator.getCalibratedData(index, data);
                    if (debugEnabled)
                    {
                        dbgTrace.traceInfo(funcName, "calibrated=%.3f", data);
                    }
                }

                data *= processorParams.scales[index];
                data += processorParams.offsets[index];
                data *= processorParams.signs[index];

                if (debugEnabled)
                {
                    dbgTrace.traceInfo(funcName, "scaled=%.3f (scale=%f,offset=%f)",
                            data, processorParams.scales[index], processorParams.offsets[index]);
                }
            }
        }
        catch (IllegalArgumentException e)
        {
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%f", data);
        }

        return data;
    }   //processData

}   //class TrcSensor
