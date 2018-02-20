package hankutanku.math;

public interface Function<T>
{
    /**
     * Returns some object in response to a numerical parameter.
     * @param input
     * @return
     */
    T value(double input);
}
