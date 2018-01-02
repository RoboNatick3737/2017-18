package org.firstinspires.ftc.teamcode.robot.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;

import hankextensions.RobotCore;

@TeleOp(name="Test Swervos", group= Constants.EXPERIMENTATION)
public class TestSwervos extends RobotCore
{
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private Servo frontLeftVexMotor, frontRightVexMotor, backLeftVexMotor, backRightVexMotor;
    private AbsoluteEncoder frontLeftVexPosition, frontRightVexPosition, backLeftVexPosition, backRightVexPosition;
    private ProcessConsole console;

    @Override
    protected void onRun() throws InterruptedException
    {
        frontLeftDrive = initHardwareDevice(DcMotor.class, "Front Left");
        frontRightDrive = initHardwareDevice(DcMotor.class, "Front Right");
        backLeftDrive = initHardwareDevice(DcMotor.class, "Back Left");
        backRightDrive = initHardwareDevice(DcMotor.class, "Back Right");

        frontLeftVexMotor = initHardwareDevice(Servo.class, "Front Left Vex Motor");
        frontLeftVexMotor.setPosition(0.5);
        frontLeftVexPosition = new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Left Vex Encoder"));

        frontRightVexMotor = initHardwareDevice(Servo.class, "Front Right Vex Motor");
        frontRightVexMotor.setPosition(0.5);
        frontRightVexPosition = new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Front Right Vex Encoder"));

        backLeftVexMotor = initHardwareDevice(Servo.class, "Back Left Vex Motor");
        backLeftVexMotor.setPosition(0.5);
        backLeftVexPosition = new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Left Vex Encoder"));

        backRightVexMotor = initHardwareDevice(Servo.class, "Back Right Vex Motor");
        backRightVexMotor.setPosition(0.5);
        backRightVexPosition = new AbsoluteEncoder(initHardwareDevice(AnalogInput.class, "Back Right Vex Encoder"));

        console = log.newProcessConsole("Stats");

        waitForStart();

        log.lines("Prop the robot wheels up, X to progress.");

        pauseForX();

        runOn("Front Left", frontLeftVexMotor, frontLeftVexPosition, frontLeftDrive);
        runOn("Front Right", frontRightVexMotor, frontRightVexPosition, frontRightDrive);
        runOn("Back Left", backLeftVexMotor, backLeftVexPosition, backLeftDrive);
        runOn("Back Right", backRightVexMotor, backRightVexPosition, backRightDrive);
    }

    private void pauseForX() throws InterruptedException
    {
        while (!gamepad1.x)
            flow.yield();
        flow.msPause(500); // Give user time to take hands off X.
    }

    private void runOn(String name, Servo servo, AbsoluteEncoder position, DcMotor motor) throws InterruptedException
    {
        log.lines("Running " + name);
        motor.setPower(1);
        servo.setPosition(1);
        while (!gamepad1.x)
        {
            console.write("Pos is " + position.position());
            flow.yield();
        }
        flow.msPause(500); // Give user time to take hands off X.
        servo.setPosition(0.5);
        motor.setPower(0);
    }
}