package hankutanku.math;

import dude.makiah.androidlib.threading.TimeMeasure;

public interface LimitedUpdateRateFunction<T> extends Function<T>
{
    TimeMeasure getUpdateRate();
}
