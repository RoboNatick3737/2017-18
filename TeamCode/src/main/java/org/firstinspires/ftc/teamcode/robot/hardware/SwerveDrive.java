package org.firstinspires.ftc.teamcode.robot.hardware;

import android.support.annotation.NonNull;

import com.makiah.makiahsandroidlib.logging.LoggingBase;
import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.ScheduledTask;
import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;

import hankextensions.EnhancedOpMode;
import hankextensions.input.HTButton;
import hankextensions.input.HTGamepad;
import hankextensions.phonesensors.Gyro;

import org.firstinspires.ftc.teamcode.components.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.components.pid.PIDController;
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
    private static final double ROBOT_WIDTH = 18, ROBOT_LENGTH = 18;
    private static final double ROBOT_PHI = Math.toDegrees(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH)); // Will be 45 degrees with perfect square dimensions.
    private static final double[] WHEEL_ORIENTATIONS = {ROBOT_PHI - 90, (180 - ROBOT_PHI) - 90, (180 + ROBOT_PHI) - 90, (360 - ROBOT_PHI) - 90};
    private static final PIDConstants TURN_PID_CONSTANTS = new PIDConstants(.005, 0, 0, 12);

    //////  Instance specific components ////////
    private boolean joystickControlEnabled = false; // Whether or not the joystick can direct the swerve drive.
    private final SwerveWheel[] swerveWheels = new SwerveWheel[4];
    public final Gyro gyro; // Public because teleop can manually reset.
    private final PIDController pidController;

    // Constantly shifting in autonomous and teleop.
    private Vector2D desiredMovement = Vector2D.ZERO;
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
     * Only takes effect if the gamepad has been supplied to this class.
     */
    private void acceptControllerInput()
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
        if (HTGamepad.CONTROLLER1.y.currentState == HTButton.ButtonState.JUST_TAPPED)
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

        if (HTGamepad.CONTROLLER1.gamepad.left_trigger > 0.1 || HTGamepad.CONTROLLER1.gamepad.right_trigger > 0.1)
        {
            this.desiredHeading += 5 * (HTGamepad.CONTROLLER1.gamepad.left_trigger - HTGamepad.CONTROLLER1.gamepad.right_trigger);
            this.desiredHeading = Vector2D.clampAngle(this.desiredHeading);
        }
    }


    // Unboxing/boxing slowdown fix.
    private double gyroHeading = 0;
    private double rotationSpeed;
    private Vector2D fieldCentricTranslation;
    private double angleOff;

    /**
     * This is where pretty much all the work for the swerve DRIVE calculations take place
     * (calculating the vectors to which the wheels should align), but the wheels have their own
     * update methods to actually change their orientation.
     */
    private void recalculateSwerveWheelVectors() throws InterruptedException
    {
        if (joystickControlEnabled)
            acceptControllerInput();

        // Get current gyro val.
        gyroHeading = gyro.z();

        // Find the least heading between the gyro and the current heading.
        angleOff = (Vector2D.clampAngle(desiredHeading - gyroHeading) + 180) % 360 - 180;
        angleOff = angleOff < -180 ? angleOff + 360 : angleOff;

        // Figure out the actual translation vector for swerve wheels based on gyro value.
        fieldCentricTranslation = desiredMovement.rotateBy(-gyroHeading);

        // Don't bother trying to be more accurate than 8 degrees while turning.
        rotationSpeed = -pidController.calculatePIDCorrection(angleOff);

        /*
         * Calculate in accordance with http://imjac.in/ta/pdf/frc/A%20Crash%20Course%20in%20Swerve%20Drive.pdf
         * Note that I'm scaling these up by 100 to convert to cm/s from encoder ticks/s
         */
        for (int i = 0; i < swerveWheels.length; i++)
            swerveWheels[i].setVectorTarget(
                    Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[i]).add(fieldCentricTranslation).multiply(100));

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

        // Tell the swerve wheels whether it's okay to start driving.
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
                "Driving acceptable: " + drivingCanStart
        );
    }

    /**
     * A scheduled task to ensure swerve consistency.
     */
    @Override
    protected long onContinueTask() throws InterruptedException
    {
        recalculateSwerveWheelVectors();
        return 40;
    }
}
