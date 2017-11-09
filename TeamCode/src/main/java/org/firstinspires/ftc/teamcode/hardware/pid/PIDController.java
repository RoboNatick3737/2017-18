package org.firstinspires.ftc.teamcode.hardware.pid;

import hankextensions.structs.Vector2D;

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
     * Stored after each calculation for the next correction.
     */
    private Vector2D lastOrientation = Vector2D.polar(1, 0);

    /**
     * Updates whenever calculatePIDCorrection called.
     */
    private long lastCorrectionTime = -1;


    /**
     * Required to calculate the kI * I factor.
     */
    private double errorAccumulation;

    /**
     * Required for derivative correction calculation.
     */
    private double lastError;

    /**
     * Prevent boxing/unboxing by placing these items before the method.
     */
    private double proportionalCorrection, derivativeCorrection, integralCorrection;

    public PIDController(PIDConstants pidConstants)
    {
        this.pidConstants = pidConstants;

        this.proportionalCorrection = 0;
        this.derivativeCorrection = 0;
        this.integralCorrection = 0;
    }


    private double timeSinceLastCorrection;
    /**
     * If the method-caller already knows the error value, this does the heavy lifting
     * and figures out what to do with it next.
     * @return
     */
    public double calculatePIDCorrection(double error)
    {
        if (Math.abs(error) < pidConstants.errorThreshold)
            return 0;

        // Calculate proportional correction, the "quick" correction factor.
        if (Math.abs(pidConstants.kP) > NO_CALCULATION_THRESHOLD)
            proportionalCorrection = pidConstants.kP * error;
        else
            proportionalCorrection = 0;

        // Only calculate integral and derivative correction if we have to.
        if (lastCorrectionTime != -1 && (Math.abs(pidConstants.kD) > NO_CALCULATION_THRESHOLD || Math.abs(pidConstants.kI) > NO_CALCULATION_THRESHOLD))
        {
            // Calculate time passed since last loop.
            timeSinceLastCorrection = (System.currentTimeMillis() - lastCorrectionTime) / 1000.0;

            if (Math.abs(pidConstants.kD) > NO_CALCULATION_THRESHOLD) {
                // Calculate derivative correction, which reduces the oscillation of the kP function.
                derivativeCorrection = pidConstants.kD * (error - lastError) / timeSinceLastCorrection;
            } else
                derivativeCorrection = 0;

            if (Math.abs(pidConstants.kI) > NO_CALCULATION_THRESHOLD) {
                // Calculate integral correction, which further reduces oscillation.
                errorAccumulation += error * timeSinceLastCorrection;
                integralCorrection = pidConstants.kI * errorAccumulation;
            } else
                integralCorrection = 0;

            // Record the last correction time.
            lastCorrectionTime = System.currentTimeMillis();
            lastError = error;
        }
        else
        {
            derivativeCorrection = 0;
            integralCorrection = 0;
            lastCorrectionTime = -1;
            lastError = 0;
        }

        // Return the total correction (PID)
        return proportionalCorrection + derivativeCorrection + integralCorrection;
    }

    /**
     * Does all of the heavy lifting for the PID calculations (actually doing the formula).
     * @param currentOrientation the current orientation of the wheel, gear, whatever.
     *                           Imagine this as a spoke in the wheel, the vector representation
     *                           makes it easier to work with.
     * @return the new correction factor, negative to slow down, and positive to speed up.
     */
    public double calculatePIDCorrection(Vector2D currentOrientation, Vector2D desiredOrientation)
    {
        // Calculate error based on the current orientation.
        double error = lastOrientation.leastAngleTo(desiredOrientation) - lastOrientation.leastAngleTo(currentOrientation);

        // Record last position and time.
        lastOrientation = Vector2D.clone(currentOrientation);
        lastCorrectionTime = System.currentTimeMillis();

        return calculatePIDCorrection(error);
    }
}
