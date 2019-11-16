package common;

import trclib.TrcEvent;

public class Grabber extends ServoEndEffector
{
    public Grabber(String instanceName, Parameters params)
    {
        super(instanceName, params);
    }

    public void release()
    {
        super.extend();
    }

    public void release(double time, TrcEvent event)
    {
        super.extend(time, event);
    }

    public void grab()
    {
        super.retract();
    }

    public void grab(double time, TrcEvent event)
    {
        super.retract(time, event);
    }
}
