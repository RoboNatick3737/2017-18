package org.firstinspires.ftc.teamcode.structs.pid;

/**
 * Largely based on http://www.societyofrobots.com/programming_PID.shtml.
 */
public class PIDController
{
    /**
     * The value at which kP, kI, and kD become pointless to calculate.
     */
    private final static double NO_CALCULATION_THRESHOLD = .000000005;

    /**
     * The constants required to evaluate the PID formula.
     */
    public final PIDConstants pidConstants;

    /**
     * Updates whenever calculatePIDCorrection called.
     */
    private long lastCorrectionTime = -1;

    /**
     * Required for derivative correction calculation.
     */
    private double lastError;

    public PIDController(PIDConstants pidConstants)
    {
        this.pidConstants = pidConstants;
    }

    /**
     * Whether or not this loop isn't ready to update (would be inaccurate).
     */
    public boolean canUpdate()
    {
        return System.nanoTime() - lastCorrectionTime > pidConstants.minimumNanosecondGap;
    }

    /**
     * Don't integrate or derive over MASSIVE lapses of time.
     */
    public void pauseController()
    {
        lastCorrectionTime = -1;
    }

    /**
     * Has to remember past states.
     */
    private double i = 0;

    /**
     * If the method-caller already knows the error value, this does the heavy lifting
     * and figures out what to do with it next.
     * @return correction result
     */
    public double calculatePIDCorrection(double error)
    {
        if (Math.abs(error) < pidConstants.errorThreshold)
            return 0;

        if (!canUpdate())
            return 0;

        // Calculate proportional correction.
        double p = Math.abs(pidConstants.kP) > NO_CALCULATION_THRESHOLD ? pidConstants.kP * error : 0;

        // Stop here if I and D can't be calculated
        if (Math.abs(pidConstants.kD) < NO_CALCULATION_THRESHOLD && Math.abs(pidConstants.kI) < NO_CALCULATION_THRESHOLD)
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
        double d = Math.abs(pidConstants.kD) > NO_CALCULATION_THRESHOLD ? pidConstants.kD * (error - lastError) / secondsSinceLoop : 0;

        // Calculate integral correction.
        if (Math.abs(pidConstants.kI) > NO_CALCULATION_THRESHOLD)
        {
            // Keep track of previous error
            i += pidConstants.kI * error * secondsSinceLoop;

            // Prevent windup (don't scale up output excessively).
            if (i > pidConstants.maximumOutput)
                i = pidConstants.maximumOutput;
            else if (i < pidConstants.minimumOutput)
                i = pidConstants.minimumOutput;
        }
        else
            i = 0;

        // Record the last correction time.
        lastCorrectionTime = System.nanoTime();
        lastError = error;

        return p + i + d;
    }
}
