package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Servos Thing", group="Experimentation")
public class ProgramSomeServos extends OpMode
{
    private Servo flipServo1, flipServo2, conveyorVexMotor;

    private double flipServoPos = 0;
    private final double FLIP_INCREMENT = .02, FLIP_MAX = .8, FLIP_MIN = .137;

    @Override
    public void init() {
        flipServo1 = hardwareMap.servo.get("Flip Servo 1");
        flipServo2 = hardwareMap.servo.get("Flip Servo 2");
        conveyorVexMotor = hardwareMap.servo.get("Conveyor Vex Motor");

        updateConveyor();
        updateFlippers();
    }

    private void updateFlippers()
    {
        flipServoPos += FLIP_INCREMENT * gamepad1.right_trigger;
        flipServoPos -= FLIP_INCREMENT * gamepad1.left_trigger;

        if (gamepad1.a)
            flipServoPos = FLIP_MIN;

        flipServoPos = Range.clip(flipServoPos, FLIP_MIN, FLIP_MAX);

        flipServo1.setPosition(Range.clip(flipServoPos, 0, 1));
        flipServo2.setPosition(Range.clip(1.08 - flipServoPos, 0, 1));

        telemetry.addLine("Flip position is " + flipServoPos);
        telemetry.update();
    }

    private void updateConveyor()
    {
        if (gamepad1.dpad_up)
            conveyorVexMotor.setPosition(0);
        else if (gamepad1.dpad_down)
            conveyorVexMotor.setPosition(1);
        else
            conveyorVexMotor.setPosition(0.5);
    }

    @Override
    public void loop()
    {
        updateFlippers();
        updateConveyor();
    }
}
