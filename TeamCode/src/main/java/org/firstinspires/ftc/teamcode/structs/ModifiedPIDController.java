package org.firstinspires.ftc.teamcode.structs;

/**
 * The idea for this is swerve module friction: a steady state error isn't side dependent (friction)
 * so there's no point in keeping a sign on I.  Therefore i is scaled with the sign of the error.
 */
public class ModifiedPIDController extends PIDController
{
    /**
     * Since I is only being scaled up, I is scaled down by some factor over time just to ensure
     * stability.
     */
    private double decayRate;

    public ModifiedPIDController(double kP, double kI, double kD, double errorThreshold, TimeUnits updateRateUnits, long minimumTimeGap, double minimumOutput, double maximumOutput, double decayRate)
    {
        super(kP, kI, kD, errorThreshold, updateRateUnits, minimumTimeGap, minimumOutput, maximumOutput);

        this.decayRate = decayRate;
    }

    /**
     * If the method-caller already knows the error value, this does the heavy lifting
     * and figures out what to do with it next.
     * @return correction result
     */
    @Override
    public double value(double error)
    {
        if (Math.abs(error) < errorThreshold)
            return 0;

        if (!canUpdate())
            return 0;

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

        // Calculate integral correction, supplies extra power to p because friction means steady state error.
        if (Math.abs(kI) > NO_CALCULATION_THRESHOLD)
        {
            // Slowly reduce i over time.
            i *= decayRate;

            // Keep track of previous error, always is positive.
            i += Math.abs(kI * error * secondsSinceLoop) / (Math.abs(p) + 1); // The greater p is, the less we should scale up i.

            // Prevent windup (don't scale up output excessively).
            if (i > maximumOutput)
                i = maximumOutput;
        }
        else
            i = 0;

        // Record the last correction time.
        lastCorrectionTime = System.nanoTime();
        lastError = error;

        return p + Math.signum(error) * i + d;
    }
}
