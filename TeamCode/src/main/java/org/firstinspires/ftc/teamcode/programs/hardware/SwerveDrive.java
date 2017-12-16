package org.firstinspires.ftc.teamcode.programs.hardware;

import android.support.annotation.NonNull;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.ScheduledTask;
import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.hardware.Gamepad;

import hankextensions.RobotCore;
import hankextensions.input.HankuTankuGamepad;
import hankextensions.phonesensors.Gyro;

import org.firstinspires.ftc.teamcode.hardware.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.hardware.pid.PIDController;
import hankextensions.structs.Vector2D;

public class SwerveDrive extends ScheduledTask
{
    //////  Physical drive constants ////////
    private static final double ROBOT_WIDTH = 18, ROBOT_LENGTH = 18;
    private static final double ROBOT_PHI = Math.toDegrees(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH)); // Will be 45 degrees with perfect square dimensions.
    private static final double[] WHEEL_ORIENTATIONS = {ROBOT_PHI - 90, (180 - ROBOT_PHI) - 90, (180 + ROBOT_PHI) - 90, (360 - ROBOT_PHI) - 90};
    private static final PIDConstants TURN_PID_CONSTANTS = new PIDConstants(.005, 0, 0, 12);

    //////  Instance specific components ////////
    private final SwerveWheel frontLeft, frontRight, backLeft, backRight;
    public final Gyro gyro; // Public because teleop can manually reset.
    private final PIDController pidController;

    // Constantly shifting in autonomous and teleop.
    private Vector2D desiredMovement = Vector2D.ZERO;
    private double desiredHeading = 0;

    // Required for operation of the driving tasks.
    public final ScheduledTaskPackage swerveUpdatePackage;

    // The gamepad which controls the bot.
    private HankuTankuGamepad gamepad;

    // The logger for when data needs to be displayed to the drivers.
    private final ProcessConsole swerveConsole;

    // The swerve drive constructor, starts all swerve wheel alignment threads.
    public SwerveDrive(Gyro gyro, SwerveWheel frontLeft, SwerveWheel frontRight, SwerveWheel backLeft, SwerveWheel backRight) throws InterruptedException
    {
        // Allows field centric orientation
        this.gyro = gyro;

        // The swerve wheels.
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        // Initialize the task package regardless we need it atm, better to have it and skip the initialization sequence.
        swerveUpdatePackage = new ScheduledTaskPackage(RobotCore.instance, "Swerve Turn Alignments",
                this, this.frontLeft, this.frontRight, this.backLeft, this.backRight);

        pidController = new PIDController(TURN_PID_CONSTANTS);

        swerveConsole = RobotCore.instance.log.newProcessConsole("Swerve Console");
    }

    /**
     * Shifts the current swerve control system between synchronous and asynchronous.
     */
    public void setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode updateMode)
    {
        swerveUpdatePackage.setUpdateMode(updateMode);
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
     * Tells the swerve drive how to accomplish teleop.
     * @param gamepad the gamepad which provides instructions.
     */
    public void provideGamepad(@NonNull Gamepad gamepad)
    {
        this.gamepad = new HankuTankuGamepad(gamepad);
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
    private void updateVectorsByGamepadInstructions()
    {
        // Rotate by -90 in order to make forward facing zero.
        Vector2D joystickDesiredRotation = gamepad.leftJoystick();
        Vector2D joystickDesiredMovement = gamepad.rightJoystick();

        // Use the left joystick for rotation unless nothing is supplied, in which case check the DPAD.
        if (joystickDesiredRotation.magnitude > .0005)
            setDesiredHeading(joystickDesiredRotation.angle);
        else
        {
            Vector2D dpadDesiredRotation = gamepad.dpad();

            if (dpadDesiredRotation.magnitude > .0005)
                setDesiredHeading(dpadDesiredRotation.angle);
        }

        if (joystickDesiredMovement.magnitude > .0005)
            setDesiredMovement(joystickDesiredMovement);
        else
            setDesiredMovement(Vector2D.ZERO);

        if (gamepad.gamepad.a)
        {
            try
            {
                gyro.calibrate();
                setDesiredHeading(0);
            }
            catch (InterruptedException e)
            {
                return;
            }
        }

        if (gamepad.gamepad.x)
            TURN_PID_CONSTANTS.kP += .0001;
        else if (gamepad.gamepad.b)
            TURN_PID_CONSTANTS.kP -= .0001;
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
        if (gamepad != null)
            updateVectorsByGamepadInstructions();

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
        frontLeft.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[0]).add(fieldCentricTranslation).multiply(100));
        backLeft.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[1]).add(fieldCentricTranslation).multiply(100));
        backRight.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[2]).add(fieldCentricTranslation).multiply(100));
        frontRight.setVectorTarget(Vector2D.polar(rotationSpeed, WHEEL_ORIENTATIONS[3]).add(fieldCentricTranslation).multiply(100));

        // Write some information to the telemetry console.
        swerveConsole.write(
                "Current Heading: " + gyroHeading,
                "Desired Angle: " + desiredHeading,
                "Rotation Speed: " + rotationSpeed,
                "Translation Vector: " + desiredMovement.toString(Vector2D.VectorCoordinates.POLAR),
                "PID kP: " + TURN_PID_CONSTANTS.kP,
                "PID kD: " + TURN_PID_CONSTANTS.kD
        );

        // Check to see whether it's okay to start moving by observing the state of all wheels.  .
        if (
                frontLeft.atAcceptableSwivelOrientation() &&
                        frontRight.atAcceptableSwivelOrientation() &&
                        backLeft.atAcceptableSwivelOrientation() &&
                        backRight.atAcceptableSwivelOrientation())
        {
            frontLeft.setDrivingState(true);
            frontRight.setDrivingState(true);
            backLeft.setDrivingState(true);
            backRight.setDrivingState(true);
        } else {
            frontLeft.setDrivingState(false);
            frontRight.setDrivingState(false);
            backLeft.setDrivingState(false);
            backRight.setDrivingState(false);
        }
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
