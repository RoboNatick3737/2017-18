package hankutanku.math;

import dude.makiah.androidlib.threading.TimeMeasure;

public interface LimitedUpdateRateFunction extends Function
{
    TimeMeasure getUpdateRate();
}
