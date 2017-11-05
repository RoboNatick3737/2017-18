package org.firstinspires.ftc.teamcode.hardware.pid;

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

    public PIDConstants(double kP, double kI, double kD, double errorThreshold)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.errorThreshold = errorThreshold;
    }
}
