package hankutanku.math;

import com.qualcomm.robotcore.util.Range;

import dude.makiah.androidlib.threading.TimeMeasure;

/**
 * Largely based on http://www.societyofrobots.com/programming_PID.shtml.
 */
public class PIDController implements LimitedUpdateRateFunction<Double>
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
     * Required to prevent integral windup (in which the integrator proceeds despite having
     * surpassed the maximum output.
     */
    public double minimumOutput, maximumOutput;

    /**
     * The value at which kP, kI, and kD become pointless to calculate.
     */
    protected final static double NO_CALCULATION_THRESHOLD = .000000005;

    /**
     * The minimum time gap between controller updates.
     */
    protected TimeMeasure updateRate;

    /**
     * Updates whenever calculatePIDCorrection called.
     */
    protected long lastCorrectionTime = -1;

    /**
     * Required for derivative correction calculation.
     */
    protected double lastError;

    /**
     * Constructor for the controller
     */
    public PIDController(double kP, double kI, double kD, double errorThreshold, TimeMeasure updateRate, double minimumOutput, double maximumOutput)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.errorThreshold = errorThreshold;

        this.minimumOutput = minimumOutput;
        this.maximumOutput = maximumOutput;

        this.updateRate = updateRate;
    }

    /**
     * Whether or not this loop isn't ready to update (would be inaccurate).
     */
    public boolean canUpdate()
    {
        return System.nanoTime() - lastCorrectionTime >= updateRate.durationIn(TimeMeasure.Units.NANOSECONDS);
    }

    /**
     * Don't integrate or derive over MASSIVE lapses of time.
     */
    public void pauseController()
    {
        lastCorrectionTime = -1;
    }

    /**
     * Quick resets
     */
    public void resetController()
    {
        pauseController();
        i = 0;
    }

    /**
     * Has to remember past states.
     */
    protected double i = 0;

    /**
     * If the method-caller already knows the error value, this does the heavy lifting
     * and figures out what to do with it next.
     * @return correction result
     */
    public Double value(double error)
    {
        if (Math.abs(error) < errorThreshold)
            return 0.0;

        if (!canUpdate())
            return 0.0;

        // Calculate proportional correction.
        double p = Math.abs(kP) > NO_CALCULATION_THRESHOLD ? kP * error : 0;

        // Stop here if I and D can't be calculated
        if (Math.abs(kD) < NO_CALCULATION_THRESHOLD && Math.abs(kI) < NO_CALCULATION_THRESHOLD)
            return p;

        // Don't calculate I and D if paused or never started.
        if (lastCorrectionTime == -1)
        {
            lastCorrectionTime = System.nanoTime();
            return p;
        }

        // Calculate time passed since last loop.
        double secondsSinceLoop = (System.nanoTime() - lastCorrectionTime) / 1e9;

        // Calculate derivative correction
        double d = Math.abs(kD) > NO_CALCULATION_THRESHOLD ? kD * (error - lastError) / secondsSinceLoop : 0;

        // Calculate integral correction.
        if (Math.abs(kI) > NO_CALCULATION_THRESHOLD)
        {
            // Keep track of previous error
            i += kI * error * secondsSinceLoop;

            // Prevent windup (don't scale up output excessively).
            i = Range.clip(i, minimumOutput, maximumOutput);
        }
        else
            i = 0;

        // Record the last correction time.
        lastCorrectionTime = System.nanoTime();
        lastError = error;

        return p + i + d;
    }

    public String summary()
    {
        return "kP: " + kP + "kD: " + kD + "kI: " + kI + "I: " + i;
    }

    @Override
    public TimeMeasure getUpdateRate()
    {
        return updateRate;
    }
}
