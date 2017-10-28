package hankextensions.phonesensors;

public interface Gyro
{
    /**
     * Should result in complete recalibration of the gyroscope.
     */
    void calibrate() throws InterruptedException;

    /**
     * Should just zero the gyroscope, over however much time it requires.
     */
    void zero() throws InterruptedException;

    /**
     * Accuracy may vary depending on the gyroscope in question.
     * @return the current heading of the robot.
     */
    double x();
    double y();
    double z();
}
