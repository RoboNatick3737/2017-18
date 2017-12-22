package org.firstinspires.ftc.teamcode.programs.teleop;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.HardwareBase;

import hankextensions.input.HTGamepad;

// Per-unit tax = vc up so avc up, therefore atc and mc go up
// Per-unit subsidy = vc down so avc down therevfore atc and mc go down
// lump-sum = atc up/down

public abstract class TeleopBase extends HardwareBase
{
    protected HTGamepad htGamepad1, htGamepad2;

    @Override
    protected void HARDWARE() throws InterruptedException
    {
        super.HARDWARE();

        htGamepad1 = new HTGamepad(gamepad1);
        htGamepad2 = new HTGamepad(gamepad2);
    }
}
