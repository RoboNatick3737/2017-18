package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankextensions.RobotCore;

@Autonomous(name = "Test Op Mode", group = "Experimentation")
public class TestOpMode extends RobotCore
{
    @Override
    protected void START() throws InterruptedException {
        while (true)
            flow.yield();
    }
}
