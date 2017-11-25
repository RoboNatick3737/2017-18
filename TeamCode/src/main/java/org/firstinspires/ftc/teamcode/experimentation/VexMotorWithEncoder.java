package org.firstinspires.ftc.teamcode.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import hankextensions.RobotCore;

public class VexMotorWithEncoder extends RobotCore
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

        ProcessConsole processConsole = log.newProcessConsole("Position TelemetryWrapper");
        while (true) {
            processConsole.write("Position " + toTurnEncoder.getCurrentPosition());
            flow.yield();
        }
    }
}
