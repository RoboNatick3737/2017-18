package hankextensions.phonesensors;

/**
 * Since we're better off with calculating the rate at which the gyro drifts and trying to counteract it.
 */
public interface Gyro
{
    /**
     * Should result in complete recalibration of the gyroscope.
     */
    void initAntiDrift() throws InterruptedException;

    /**
     * Tells us when the robot starts
     * @throws InterruptedException
     */
    void startAntiDrift() throws InterruptedException;

    /**
     * Tells the gyro to offset their internal count by some degree (used at OpMode start sometimes)
     * @param offset the offset to correct for
     */
    void applyOffset(double offset);

    /**
     * Should just zero the gyroscope, over however much time it requires.
     */
    void zero() throws InterruptedException;

    /**
     * Accuracy may vary depending on the gyroscope in question.
     * @return the current heading of the robot.
     */
    double getHeading();
}
