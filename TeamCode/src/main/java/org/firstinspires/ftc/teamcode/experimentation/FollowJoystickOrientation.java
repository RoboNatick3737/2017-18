package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import hankextensions.Core;
import hankextensions.logging.ProcessConsole;
import hankextensions.threading.Flow;

public class FollowJoystickOrientation extends Core
{
    private DcMotor toTurn;
    private ModernRoboticsI2cColorSensor calibrationSensor;

    private void setMotorMode(DcMotor motor, DcMotor.RunMode runMode)
    {
        boolean doneSuccessfully = false;
        long additionalTime = 0;
        while (!doneSuccessfully)
        {
            try
            {
                motor.setMode (runMode);
                Flow.msPause (100 + additionalTime);
                doneSuccessfully = true;
            }
            catch (Exception e)
            {
                log.lines("ERROR");
                if (e instanceof InterruptedException)
                    return;

                additionalTime += 20;
            }
        }
    }

    private int previousVal = 0;

    private int getDpadHeading()
    {
        if (gamepad1.dpad_up)
        {
            if (gamepad1.dpad_left) {
                return 45;
            }
            if (gamepad1.dpad_right) {
                return 315;
            }
            return 0;
        }

        if (gamepad1.dpad_down)
        {
            if (gamepad1.dpad_left) {
                return 135;
            }
            if (gamepad1.dpad_right) {
                return 225;
            }
            return 180;
        }

        if (gamepad1.dpad_left) {
            return 90;
        }
        if (gamepad1.dpad_right) {
            return 270;
        }

        return -1;
    }

    private int getDesiredJoystickHeading()
    {
        int dpadHeading = getDpadHeading();
        if (dpadHeading != -1) {
            previousVal = dpadHeading;
            return dpadHeading;
        }

        if (gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0) {
            return previousVal;
        }

        double desiredHeading = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) * 180.0 / Math.PI - 90;

        if (desiredHeading < 0) {
            desiredHeading = 360 - Math.abs(desiredHeading); // if like -90 or something
        }

        int desiredHeadingInt = (int)(desiredHeading);

        previousVal = desiredHeadingInt;

        return desiredHeadingInt;
    }

    private void resetEncoder() {
        // Used to set the encoder to the appropriate position.
        setMotorMode(toTurn, DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorMode(toTurn, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(toTurn, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    int offsetRevolutions = 0;
    protected final void START() throws InterruptedException {
        final int encoderRevolution = 1680;

        log.lines("Started");

        toTurn = initHardwareDevice(DcMotor.class, "Left Motor");
        toTurn.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoder();

        calibrationSensor = initHardwareDevice(ModernRoboticsI2cColorSensor.class, "Calibration Sensor");

        ProcessConsole processConsole = log.newProcessConsole("Position Console");
        ProcessConsole calibrationConsole = log.newProcessConsole("Motor Calibration");
        while (!gamepad1.x)
        {
            if (gamepad1.start)
            {
                // Recalibrate the motor position.
                toTurn.setPower(-.05);
                calibrationSensor.enableLed(true);
                while (calibrationSensor.alpha() <= 5)
                {
                    calibrationConsole.write("Alpha is " + calibrationSensor.alpha() + " threshold is 5");
                    Flow.yield();
                }
                calibrationSensor.enableLed(true);
                previousVal = 0;
                offsetRevolutions = 0;
                toTurn.setPower(0);
                resetEncoder();
            }

            int currentPosition = toTurn.getCurrentPosition();
            int desiredMotorPosition = (int)(getDesiredJoystickHeading() / 360.0 * encoderRevolution) + offsetRevolutions * encoderRevolution;

            // Figure out whether we should continue the current rotation or backtrack (whichever is fastest).
            if (Math.abs(currentPosition - (desiredMotorPosition + encoderRevolution)) < Math.abs(currentPosition - desiredMotorPosition)) {
                offsetRevolutions += 1;
            }

            if (Math.abs(currentPosition - (desiredMotorPosition - encoderRevolution)) < Math.abs(currentPosition - desiredMotorPosition)) {
                offsetRevolutions -= 1;
            }

            // Update the motor power if we're off of the desired position by some threshold.
            final int offFromDesired = desiredMotorPosition - currentPosition;
            if (Math.abs(offFromDesired) > 10) {
                toTurn.setPower(Math.signum(offFromDesired) * (.05 + Math.abs(offFromDesired) * .0005));
            } else {
                toTurn.setPower(0);
            }

            processConsole.write(
                    "X=" + gamepad1.left_stick_x + " Y=" + -gamepad1.left_stick_y + " so degree is " + getDesiredJoystickHeading(),
                    "Current Pos=" + currentPosition + " and desired=" + desiredMotorPosition + " offset=" + offsetRevolutions);

            Flow.yield();
        }

        toTurn.setPower(0);
        log.lines("Done");
        Flow.msPause(2000);
    }
}
