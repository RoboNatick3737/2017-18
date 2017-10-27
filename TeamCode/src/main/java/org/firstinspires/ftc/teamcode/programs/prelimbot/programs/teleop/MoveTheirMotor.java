package org.firstinspires.ftc.teamcode.programs.prelimbot.programs.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.programs.prelimbot.HardwareBase;

import hankextensions.threading.Flow;

@TeleOp(name="Move Their Motor", group="Experimentation")
public class MoveTheirMotor extends HardwareBase
{
    private DcMotor liftylift1;
    private DcMotor liftylift2;

    @Override
    protected void INITIALIZE() throws InterruptedException
    {
        liftylift1 = initHardwareDevice(DcMotor.class, "LiftyLift1");
        liftylift2 = initHardwareDevice(DcMotor.class, "LiftyLift2");
    }

    @Override
    protected void START() throws InterruptedException
    {
        while (true)
        {
            if (gamepad1.a)
            {
                liftylift1.setPower(0.1);
            }

            Flow.yield();
        }
    }
}
