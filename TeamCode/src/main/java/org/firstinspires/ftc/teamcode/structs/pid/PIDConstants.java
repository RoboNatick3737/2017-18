package org.firstinspires.ftc.teamcode.structs.pid;

/**
 * Wraps P, I, and D constants into a single class.
 */
public class PIDConstants
{
    /**
     * Typically the main drive in a control loop, KP reduces a large part of the overall
     * error.
     *
     * KP x Verror
     */
    public double kP;

    /**
     * Reduces the final error in a system. Summing even a small error over time produces
     * a drive signal large enough to move the system toward a smaller error.
     *
     * KI x  ∫ Verror dt
     *
     * Note that this is very seldom used for FTC robots, and so may be zero for some
     * hardware.
     */
    public double kI;

    /**
     * Counteracts the KP and KI terms when the output changes quickly. This helps reduce
     * overshoot and ringing. It has no effect on final error.
     *
     * KD x dVerror / dt
     */
    public double kD;

    /**
     * Sets a range in which no errors should be corrected for.
     */
    public double errorThreshold;

    /**
     * The unit which controls how quickly it updates (just gets converted in the constructor).
     */
    public enum TimeUnits {SECONDS, MILLISECONDS, NANOSECONDS, HERTZ}

    /**
     * The minimum time required before updating this PID loop (in nanoseconds).
     */
    public long minimumNanosecondGap;

    /**
     * Required to prevent integral windup (in which the integrator proceeds despite having
     * surpassed the maximum output.
     */
    public double minimumOutput, maximumOutput;

    public PIDConstants(double kP, double kI, double kD, double errorThreshold, TimeUnits updateRateUnits, long minimumTimeGap, double minimumOutput, double maximumOutput)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.errorThreshold = errorThreshold;

        this.minimumOutput = minimumOutput;
        this.maximumOutput = maximumOutput;

        // Decide nanoseconds required for update.
        switch (updateRateUnits)
        {
            case SECONDS:
                minimumTimeGap *= 1e9;
                break;

            case MILLISECONDS:
                minimumTimeGap *= 1e3;
                break;

            case NANOSECONDS:
                // no conversion necessary
                break;

            case HERTZ:
                minimumTimeGap = (long)(1e9 / minimumTimeGap);
                break;
        }
        this.minimumNanosecondGap = minimumTimeGap;
    }
}
