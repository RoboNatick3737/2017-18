package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.hardware.EncoderMotor;
import org.firstinspires.ftc.teamcode.structs.pid.PIDConstants;

import hankextensions.EnhancedOpMode;

@Autonomous(name="Test Motor PID Stability", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class MotorPIDTesting extends EnhancedOpMode
{
    private EncoderMotor frontLeft, backLeft, frontRight, backRight;

    @Override
    protected void onRun() throws InterruptedException
    {
        // All of the drive motors and their respective PID.
        frontLeft = new EncoderMotor(
                "Front Left",
                hardware.initialize(DcMotor.class, "Front Left"),
                new PIDConstants(.0006, 0, 0, 0, 40000000),
                407, 7.62, DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = new EncoderMotor(
                "Front Right",
                hardware.initialize(DcMotor.class, "Front Right"),
                new PIDConstants(.0006, 0, 0, 0, 40000000),
                202, 7.62, DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft = new EncoderMotor(
                "Back Left",
                hardware.initialize(DcMotor.class, "Back Left"),
                new PIDConstants(.0006, 0, 0, 0, 40000000),
                202, 7.62, DcMotor.ZeroPowerBehavior.BRAKE);

        backRight = new EncoderMotor(
                "Back Right",
                hardware.initialize(DcMotor.class, "Back Right"),
                new PIDConstants(.0006, 0, 0, 0, 40000000),
                475, 7.62, DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (true)
        {
            frontLeft.setVelocity(Math.random() * 100 - 50);
            frontRight.setVelocity(Math.random() * 100 - 50);
            backLeft.setVelocity(Math.random() * 100 - 50);
            backRight.setVelocity(Math.random() * 100 - 50);

            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 5000)
            {
                frontLeft.updatePID();
                frontRight.updatePID();
                backLeft.updatePID();
                backRight.updatePID();
            }

            flow.yield();
        }
    }
}
