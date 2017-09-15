package org.firstinspires.ftc.teamcode.programs.experimentation;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.programs.Core;
import org.firstinspires.ftc.teamcode.sdkextensions.logging.ProcessConsole;
import org.firstinspires.ftc.teamcode.sdkextensions.threading.Flow;

@Autonomous(name="Test Vex Motor", group="Experimentation")
public class VexMotorWithEncoder extends Core
{
    private Servo toTurn;
    private DcMotor toTurnEncoder;

    protected final void START() throws InterruptedException {
        toTurn = initHardwareDevice(Servo.class, "vexboi");
        toTurnEncoder = initHardwareDevice(DcMotor.class, "encoder");

        toTurnEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        toTurnEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        toTurnEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        toTurn.setPosition(0.8);

        ProcessConsole processConsole = log.newProcessConsole("Position Log");
        while (true) {
            processConsole.updateWith("Position " + toTurnEncoder.getCurrentPosition());
            Flow.yield();
        }
    }
}
