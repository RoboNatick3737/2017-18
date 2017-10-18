package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankextensions.Core;
import hankextensions.threading.Flow;

@Autonomous(name = "Test Op Mode", group = "Experimentation")
public class TestOpMode extends Core
{
    @Override
    protected void START() throws InterruptedException {
        while (true)
            Flow.yield();
    }
}
