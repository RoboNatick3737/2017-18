package org.firstinspires.ftc.teamcode.programs.prelimbot.programs.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by JordanArnold on 10/3/17.
 *
 *
 * For Autonomous
 *
 *
 * Use gyro for initial position
 * Hit Jewel
 * Read image
 * deposit
 *
 *
 */

@Autonomous(name="BlueNonPicSide")
public class BlueNonPicSide extends BaseAuto
{
    @Override
    protected void START() throws InterruptedException
    {
        drive(DIRECTION.FORWARD, 1000);
        drive(DIRECTION.STOP, 1000);
        drive(DIRECTION.RIGHT, 1000);
        drive(DIRECTION.STOP, 1000);
        drive(DIRECTION.BACKWARDS, 1000);
        drive(DIRECTION.STOP, 1000);
        drive(DIRECTION.LEFT, 1000);
        drive(DIRECTION.STOP, 1000);
    }
}
