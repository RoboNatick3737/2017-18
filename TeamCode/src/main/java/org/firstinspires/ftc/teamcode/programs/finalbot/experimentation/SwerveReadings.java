package org.firstinspires.ftc.teamcode.programs.finalbot.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.finalbot.hardware.AbsoluteEncoder;

import hankextensions.Core;
import hankextensions.logging.ProcessConsole;
import hankextensions.threading.Flow;

@Autonomous(name="Swerve Readings", group=Constants.FINAL_BOT_EXPERIMENTATION)
public class SwerveReadings extends Core
{
    @Override
    protected void START() throws InterruptedException
    {
        AbsoluteEncoder backRight = new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Right Vex Encoder"));
        AbsoluteEncoder frontRight = new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Right Vex Encoder"));
        AbsoluteEncoder backLeft = new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Left Vex Encoder"));
        AbsoluteEncoder frontLeft = new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Left Vex Encoder"));

        DcMotor frontLeftDrive = initHardwareDevice(DcMotor.class, "Front Left");
        DcMotor frontRightDrive = initHardwareDevice(DcMotor.class, "Front Right");
        DcMotor backLeftDrive = initHardwareDevice(DcMotor.class, "Back Left");
        DcMotor backRightDrive = initHardwareDevice(DcMotor.class, "Back Right");


        ProcessConsole swervePositionsConsole = log.newProcessConsole("Swerve positions");
        while (true)
        {
            swervePositionsConsole.write(
                    "Back Right: " + backRight.position(),
                    "Front Right: " + frontRight.position(),
                    "Back Left: " + backLeft.position(),
                    "Front Left: " + frontLeft.position(),
                    "Front Left D: " + frontLeftDrive.getCurrentPosition(),
                    "Front Right D: " + frontRightDrive.getCurrentPosition(),
                    "Back Left D: " + backLeftDrive.getCurrentPosition(),
                    "Back Right D: " + backRightDrive.getCurrentPosition());

            Flow.yield();
        }
    }
}
