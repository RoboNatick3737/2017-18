package hankutanku.math;

public class TimedFunction<T>
{
    private final Function<T> someFunction;
    private long startTime;

    public TimedFunction(Function<T> someFunction)
    {
        this.someFunction = someFunction;
        startTime = System.currentTimeMillis();
    }

    public T value()
    {
        return someFunction.value((System.currentTimeMillis() - startTime) * 1e-3); // value with current elapsed time in seconds
    }
}
