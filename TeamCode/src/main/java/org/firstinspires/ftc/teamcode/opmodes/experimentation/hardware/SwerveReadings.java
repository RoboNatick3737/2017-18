package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;

import hankextensions.EnhancedOpMode;

@Autonomous(name="Swerve Readings", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class SwerveReadings extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        waitForStart();

        AbsoluteEncoder backRight = new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Back Right Vex Encoder"));
        AbsoluteEncoder frontRight = new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Front Right Vex Encoder"));
        AbsoluteEncoder backLeft = new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Back Left Vex Encoder"));
        AbsoluteEncoder frontLeft = new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Front Left Vex Encoder"));

        DcMotor frontLeftDrive = hardware.initialize(DcMotor.class, "Front Left");
        DcMotor frontRightDrive = hardware.initialize(DcMotor.class, "Front Right");
        DcMotor backLeftDrive = hardware.initialize(DcMotor.class, "Back Left");
        DcMotor backRightDrive = hardware.initialize(DcMotor.class, "Back Right");

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

            flow.yield();
        }
    }
}
