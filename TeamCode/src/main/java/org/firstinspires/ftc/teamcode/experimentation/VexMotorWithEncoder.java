package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import hankextensions.Core;
import hankextensions.logging.ProcessConsole;
import hankextensions.threading.Flow;

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
            processConsole.write("Position " + toTurnEncoder.getCurrentPosition());
            Flow.yield();
        }
    }
}
