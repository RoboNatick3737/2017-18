package org.firstinspires.ftc.teamcode.programs.finalbot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.finalbot.HardwareBase;
import org.firstinspires.ftc.teamcode.structs.Vector2D;

import hankextensions.logging.ProcessConsole;
import hankextensions.threading.Flow;

@TeleOp(name="Swerve Teleop", group= Constants.FINAL_BOT_OPMODES)
public class SwerveTeleop extends HardwareBase
{
    @Override
    protected void START() throws InterruptedException
    {
        Vector2D lastRotation = Vector2D.ZERO;
        Vector2D desiredRotation, desiredMovement;

        ProcessConsole teleopConsole = log.newProcessConsole("Swerve Teleop Console");

        while (true)
        {
            desiredRotation = Vector2D.rectangular(gamepad1.left_stick_x, -gamepad1.left_stick_y).rotateBy(-90);
            desiredMovement = Vector2D.rectangular(gamepad1.right_stick_x, -gamepad1.right_stick_y).rotateBy(-90);

            if (desiredRotation.magnitude < .05)
                desiredRotation = lastRotation;
            else
                lastRotation = desiredRotation;

            if (desiredMovement.magnitude < .05)
                desiredMovement = Vector2D.ZERO;

            // Rotate by -90 in order to make forward facing zero.
            swerveDrive.setDesiredRotation(desiredRotation);
            swerveDrive.setDesiredMovement(desiredMovement);

            Flow.yield();
        }
    }
}
