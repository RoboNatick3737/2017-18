package org.firstinspires.ftc.teamcode.robot.hardware;

import android.support.annotation.NonNull;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.ScheduledTask;
import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;

import hankextensions.EnhancedOpMode;
import hankextensions.input.HTGamepad;
import hankextensions.phonesensors.Gyro;

import org.firstinspires.ftc.teamcode.structs.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.structs.pid.PIDController;
import hankextensions.structs.Vector2D;

/**
 * The SwerveDrive contains 4 SwerveWheel instances to which a number of vectors are specified
 * in order to determine the direction in which movement will occur.
 *
 * TODO slow down wheel turning speeds while moving at high speed.
 */
public class SwerveDrive extends ScheduledTask
{
    //////  Physical drive constants ////////
    private static final double MAX_TELEOP_SPEED = 100;
    private static final double ROBOT_WIDTH = 18, ROBOT_LENGTH = 18;
    private static final double ROBOT_PHI = Math.toDegrees(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH)); // Will be 45 degrees with perfect square dimensions.
    private static final double[] WHEEL_ORIENTATIONS = {ROBOT_PHI - 90, (180 - ROBOT_PHI) - 90, (180 + ROBOT_PHI) - 90, (360 - ROBOT_PHI) - 90};
    private static final PIDConstants TURN_PID_CONSTANTS = new PIDConstants(.005, 0, 0, 5);

    //////  Instance specific components ////////
    public enum ControlMethod { FIELD_CENTRIC, TANK_DRIVE }
    // Control method
    private ControlMethod controlMethod = ControlMethod.FIELD_CENTRIC;
    public void setControlMethod(ControlMethod controlMethod)
    {
        this.controlMethod = controlMethod;
    }
    public ControlMethod getControlMethod()
    {
        return controlMethod;
    }
    // Whether or not the joystick can direct the swerve drive (false during auto)
    private boolean joystickControlEnabled = false;
    // Whether or not this bot will help the driver auto-align to the cryptobox using vision.
    private boolean automaticCryptoboxAlignment = false;
    // Whether or not the robot will go in an arc instead of immediately shifting the wheels to a different direction while moving quickly.
    private boolean avoidAxleDestruction = false;
    public void setAxleDrivingProtectionTo(boolean state)
    {
        avoidAxleDestruction = state;

        if (!state)
            lastMovement = null;
    }

    // The SwerveWheel instances which constitute the swerve drive: frontLeft, backLeft, backRight, frontRight respectively.
    public final SwerveWheel[] swerveWheels = new SwerveWheel[4];
    public final Gyro gyro; // Public because teleop can manually reset.
    private final PIDController pidController;

    // Constantly shifting in autonomous and teleop.
    private Vector2D desiredMovement = Vector2D.ZERO;
    private Vector2D lastMovement = null;
    private double desiredHeading = 0;

    // Required for operation of the driving tasks.
    public final ScheduledTaskPackage swerveUpdatePackage;

    // The logger for when data needs to be displayed to the drivers.
    private final ProcessConsole swerveConsole;

    // The swerve drive constructor, starts all swerve wheel alignment threads.
    public SwerveDrive(Gyro gyro, SwerveWheel frontLeft, SwerveWheel frontRight, SwerveWheel backLeft, SwerveWheel backRight) throws InterruptedException
    {
        // Allows field centric orientation
        this.gyro = gyro;

        // The swerve wheels.
        this.swerveWheels[0] = frontLeft;
        this.swerveWheels[1] = backLeft;
        this.swerveWheels[2] = backRight;
        this.swerveWheels[3] = frontRight;

        // Initialize the task package regardless we need it atm, better to have it and skip the initialization sequence.
        swerveUpdatePackage = new ScheduledTaskPackage(EnhancedOpMode.instance, "Swerve Turn Alignments",
                this, frontLeft, frontRight, backLeft, backRight);

        // For turning the drive.
        pidController = new PIDController(TURN_PID_CONSTANTS);

        swerveConsole = LoggingBase.instance.newProcessConsole("Swerve Console");
    }

    /**
     * Tells the swerve drive whether joystick control is acceptable.
     */
    public void setJoystickControlEnabled(boolean state)
    {
        joystickControlEnabled = state;
    }

    /**
     * Shifts the current swerve control system between synchronous and asynchronous.
     */
    public void setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode updateMode)
    {
        swerveUpdatePackage.setUpdateMode(updateMode);

        if (updateMode == ScheduledTaskPackage.ScheduledUpdateMode.ASYNCHRONOUS)
            swerveUpdatePackage.run();
    }

    /**
     * Synchronous version of the async task thread.
     */
    public void synchronousUpdate() throws InterruptedException
    {
        // Only updates if the control system has been set to synchronous.
        swerveUpdatePackage.synchronousUpdate();
    }

    /**
     * Sets a new desired orientation.
     * @param newlyDesiredHeading the new orientation to align to.
     */
    public void setDesiredHeading(double newlyDesiredHeading)
    {
        this.desiredHeading = newlyDesiredHeading;
    }

    /**
     * Sets new desired translation.
     * @param newlyDesiredMovement the new desired movement vector.
     *
     */
    public void setDesiredMovement(@NonNull Vector2D newlyDesiredMovement)
    {
        this.desiredMovement = newlyDesiredMovement;
    }

    /**
     * This is where pretty much all the work for the swerve DRIVE calculations take place
     * (calculating the vectors to which the wheels should align), but the wheels have their own
     * update methods to actually change their orientation.
     */
    private void updateSwerveDriveFieldCentric() throws InterruptedException
    {
        if (joystickControlEnabled)
        {
            // Rotate by -90 in order to make forward facing zero.
            Vector2D joystickDesiredRotation = HTGamepad.CONTROLLER1.rightJoystick();
            Vector2D joystickDesiredMovement = HTGamepad.CONTROLLER1.leftJoystick();

            // Use the left joystick for rotation unless nothing is supplied, in which case check the DPAD.
            if (joystickDesiredRotation.magnitude > .0005)
                setDesiredHeading(joystickDesiredRotation.angle);

            if (joystickDesiredMovement.magnitude > .0005)
                setDesiredMovement(joystickDesiredMovement);
            else
                setDesiredMovement(Vector2D.ZERO);

            // Upon tapping white, calibrate the gyro
            if (HTGamepad.CONTROLLER1.gamepad.y)
            {
                try
                {
                    gyro.calibrate();
                }
                catch (InterruptedException e)
                {
                    return;
                }
            }

            // Fine tuned adjustments.
            if (HTGamepad.CONTROLLER1.gamepad.left_trigger > 0.1 || HTGamepad.CONTROLLER1.gamepad.right_trigger > 0.1)
            {
                this.desiredHeading += 5 * (HTGamepad.CONTROLLER1.gamepad.left_trigger - HTGamepad.CONTROLLER1.gamepad.right_trigger);
                this.desiredHeading = Vector2D.clampAngle(this.desiredHeading);
            }
        }

        // Get current gyro val.
        double gyroHeading = gyro.z();

        // Find the least heading between the gyro and the current heading.
        double angleOff = (Vector2D.clampAngle(desiredHeading - gyroHeading) + 180) % 360 - 180;
        angleOff = angleOff < -180 ? angleOff + 360 : angleOff;

        // Figure out the actual translation vector for swerve wheels based on gyro value.
        Vector2D fieldCentricTranslation = desiredMovement.rotateBy(-gyroHeading);

        // Don't bother trying to be more accurate than 8 degrees while turning.
        double rotationSpeed = -pidController.calculatePIDCorrection(angleOff);


        // Apply dumb driver handicaps :P
        if (avoidAxleDestruction && desiredMovement.magnitude > .2 && joystickControlEnabled)
        {
            if (lastMovement != null)
            {
                // Sort of move via a gradient to determine the max heading change.
                double changeCoefficient = 1 / (25 * fieldCentricTranslation.magnitude + 1);
                double maxHeadingChange = 90 * changeCoefficient;
                double maxMagnitudeChange = MAX_TELEOP_SPEED * .25 * changeCoefficient;

                double desiredChangeInHeading = lastMovement.leastAngleTo(fieldCentricTranslation);
                if (maxHeadingChange < Math.abs(desiredChangeInHeading)) // if this isn't within the constraint, shift as close as possible.
                    fieldCentricTranslation = lastMovement.rotateBy(Math.signum(desiredChangeInHeading) * maxHeadingChange);

                double desiredChangeInMagnitude = fieldCentricTranslation.magnitude - lastMovement.magnitude;
                if (maxMagnitudeChange < Math.abs(desiredChangeInMagnitude))
                    fieldCentricTranslation = Vector2D.polar(lastMovement.magnitude + Math.signum(desiredChangeInMagnitude) * maxMagnitudeChange, fieldCentricTranslation.angle);
            }

            lastMovement = fieldCentricTranslation;
        } else
        {
            lastMovement = Vector2D.ZERO;
        }

        // Calculate in accordance with http://imjac.in/ta/pdf/frc/A%20Crash%20Course%20in%20Swerve%20Drive.pdf
        for (int i = 0; i < swerveWheels.length; i++)
            swerveWheels[i].setVectorTarget(
                    Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[i]).add(fieldCentricTranslation).multiply(MAX_TELEOP_SPEED));

        // Check to see whether it's okay to start moving by observing the state of all wheels.
        boolean drivingCanStart = true;
        for (SwerveWheel wheel : swerveWheels)
        {
            if (!wheel.atAcceptableSwivelOrientation())
            {
                drivingCanStart = false;
                break;
            }
        }
        for (SwerveWheel wheel : swerveWheels)
            wheel.setDrivingState(drivingCanStart);


        // Write some information to the telemetry console.
        swerveConsole.write(
                "Current Heading: " + gyroHeading,
                "Desired Angle: " + desiredHeading,
                "Rotation Speed: " + rotationSpeed,
                "Translation Vector: " + desiredMovement.toString(Vector2D.VectorCoordinates.POLAR),
                "PID kP: " + TURN_PID_CONSTANTS.kP,
                "PID kD: " + TURN_PID_CONSTANTS.kD,
                "Magnitude: " + fieldCentricTranslation.magnitude,
                "Driving acceptable: " + drivingCanStart
        );
    }

    /**
     * No longer field centric
     */
    private void updateSwerveDriveTankDrive() throws InterruptedException
    {
        Vector2D desiredMovement = HTGamepad.CONTROLLER1.leftJoystick();
        if (desiredMovement.magnitude < .0005)
            desiredMovement = Vector2D.ZERO;

        double rotationSpeed = HTGamepad.CONTROLLER1.gamepad.right_stick_x;

        for (int i = 0; i < swerveWheels.length; i++)
            swerveWheels[i].setVectorTarget(
                    Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[i]).add(desiredMovement).multiply(MAX_TELEOP_SPEED));
    }

    /**
     * A scheduled task to ensure swerve consistency.
     */
    @Override
    protected long onContinueTask() throws InterruptedException
    {
        if (controlMethod == ControlMethod.FIELD_CENTRIC)
            updateSwerveDriveFieldCentric();
        else if (controlMethod == ControlMethod.TANK_DRIVE)
            updateSwerveDriveTankDrive();

        return 40;
    }
}
