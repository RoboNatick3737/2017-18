package org.firstinspires.ftc.teamcode.robot;

public abstract class TeleopBase extends HardwareBase
{
    @Override
    protected final void onRunWithHardware()
    {

    }

    protected abstract void onRunTeleop() throws InterruptedException;
}
