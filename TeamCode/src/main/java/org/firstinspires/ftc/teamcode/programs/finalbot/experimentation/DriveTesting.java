package org.firstinspires.ftc.teamcode.programs.finalbot.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.finalbot.HardwareBase;
import hankextensions.structs.Vector2D;

import hankextensions.threading.Flow;

@Autonomous(name="Drive Testing", group=Constants.FINAL_BOT_EXPERIMENTATION)
public class DriveTesting extends HardwareBase
{
    @Override
    protected void START() throws InterruptedException
    {
        swerveDrive.setDesiredMovement(Vector2D.polar(1, 0));

        Flow.msPause(3000);

        swerveDrive.setDesiredMovement(Vector2D.polar(1, 180));

        Flow.msPause(3000);

        swerveDrive.setDesiredMovement(Vector2D.polar(1, 90));

        Flow.msPause(3000);

        swerveDrive.setDesiredMovement(Vector2D.polar(1, 270));

        Flow.msPause(3000);
    }
}
