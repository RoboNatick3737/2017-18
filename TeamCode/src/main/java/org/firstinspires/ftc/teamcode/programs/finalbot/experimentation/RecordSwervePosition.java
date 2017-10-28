package org.firstinspires.ftc.teamcode.programs.finalbot.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.finalbot.hardware.AbsoluteEncoder;

import hankextensions.Core;
import hankextensions.logging.ProcessConsole;
import hankextensions.threading.Flow;

@Autonomous(name="Swerve Positions", group=Constants.FINAL_BOT_EXPERIMENTATION)
public class RecordSwervePosition extends Core
{
    @Override
    protected void START() throws InterruptedException
    {
        AbsoluteEncoder backRight = new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Right Vex Encoder"));
        AbsoluteEncoder frontRight = new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Right Vex Encoder"));
        AbsoluteEncoder backLeft = new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Left Vex Encoder"));
        AbsoluteEncoder frontLeft = new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Left Vex Encoder"));

        ProcessConsole swervePositionsConsole = log.newProcessConsole("Swerve positions");
        while (true)
        {
            swervePositionsConsole.write(
                    "Back Right: " + backRight.position(),
                    "Front Right: " + frontRight.position(),
                    "Back Left: " + backLeft.position(),
                    "Front Left: " + frontLeft.position());

            Flow.yield();
        }
    }
}
