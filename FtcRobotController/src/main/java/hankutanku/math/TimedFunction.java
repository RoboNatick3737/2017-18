package hankutanku.math;

public class TimedFunction
{
    private final Function someFunction;
    private long startTime;

    public TimedFunction(Function someFunction)
    {
        this.someFunction = someFunction;
        startTime = System.currentTimeMillis();
    }

    public double value()
    {
        return someFunction.value((System.currentTimeMillis() - startTime) * .001); // value with current elapsed time in seconds
    }
}
