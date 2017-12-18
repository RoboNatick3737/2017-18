package org.firstinspires.ftc.teamcode.programs.teleop;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.HardwareBase;

@TeleOp(name="Final Teleop", group= Constants.FINAL_BOT_OPMODES)
public class FinalTeleop extends HardwareBase
{
    private double flipServoPos = 0;
    private final double FLIP_INCREMENT = .02, FLIP_MAX = .8, FLIP_MIN = .137;

    @Override
    public void INITIALIZE() throws InterruptedException
    {
        updateConveyor();
        updateFlippers();
    }

    private void updateFlippers()
    {
        flipServoPos += FLIP_INCREMENT * gamepad2.right_trigger;
        flipServoPos -= FLIP_INCREMENT * gamepad2.left_trigger;

        if (gamepad1.a)
            flipServoPos = FLIP_MIN;

        flipServoPos = Range.clip(flipServoPos, FLIP_MIN, FLIP_MAX);

        leftFlipper.setPosition(Range.clip(flipServoPos, 0, 1));
        rightFlipper.setPosition(Range.clip(1.08 - flipServoPos, 0, 1));
    }

    private void updateConveyor()
    {
        if (gamepad2.dpad_up)
            conveyor.setPosition(0);
        else if (gamepad2.dpad_down)
            conveyor.setPosition(1);
        else
            conveyor.setPosition(0.5);
    }

    private void updateHarvester()
    {
        harvester.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
    }

    @Override
    protected void START() throws InterruptedException
    {
        swerveDrive.provideGamepad(gamepad1);
        swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.ASYNCHRONOUS);

        while (true)
        {
            updateConveyor();
            updateFlippers();
            updateHarvester();

            flow.yield();
        }
    }
}
