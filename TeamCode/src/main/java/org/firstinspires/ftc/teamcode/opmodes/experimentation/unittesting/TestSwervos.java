package org.firstinspires.ftc.teamcode.opmodes.experimentation.unittesting;

import dude.makiah.androidlib.logging.ProcessConsole;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.hardware.AbsoluteEncoder;

import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;

@Autonomous(name="Ensure Swervos", group= OpModeDisplayGroups.FINAL_BOT_EXPERIMENTATION)
public class TestSwervos extends EnhancedOpMode
{
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private Servo frontLeftVexMotor, frontRightVexMotor, backLeftVexMotor, backRightVexMotor;
    private AbsoluteEncoder frontLeftVexPosition, frontRightVexPosition, backLeftVexPosition, backRightVexPosition;
    private ProcessConsole console;

    @Override
    protected void onRun() throws InterruptedException
    {
        frontLeftDrive = hardware.initialize(DcMotor.class, "Front Left");
        frontRightDrive = hardware.initialize(DcMotor.class, "Front Right");
        backLeftDrive = hardware.initialize(DcMotor.class, "Back Left");
        backRightDrive = hardware.initialize(DcMotor.class, "Back Right");

        frontLeftVexMotor = hardware.initialize(Servo.class, "Front Left Vex Motor");
        frontLeftVexMotor.setPosition(0.5);
        frontLeftVexPosition = new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Front Left Vex Encoder"));

        frontRightVexMotor = hardware.initialize(Servo.class, "Front Right Vex Motor");
        frontRightVexMotor.setPosition(0.5);
        frontRightVexPosition = new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Front Right Vex Encoder"));

        backLeftVexMotor = hardware.initialize(Servo.class, "Back Left Vex Motor");
        backLeftVexMotor.setPosition(0.5);
        backLeftVexPosition = new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Back Left Vex Encoder"));

        backRightVexMotor = hardware.initialize(Servo.class, "Back Right Vex Motor");
        backRightVexMotor.setPosition(0.5);
        backRightVexPosition = new AbsoluteEncoder(hardware.initialize(AnalogInput.class, "Back Right Vex Encoder"));

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
//        while (!gamepad1.x)
//            flow.yield();
//        flow.msPause(500); // Give user time to take hands off X.

        flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, 1.5));
    }

    private void runOn(String name, Servo servo, AbsoluteEncoder position, DcMotor motor) throws InterruptedException
    {
        log.lines("Running " + name);
        motor.setPower(1);
        servo.setPosition(1);
//        while (!gamepad1.x)
//        {
//            console.write("Pos is " + position.position());
//            flow.yield();
//        }
        pauseForX();
        flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, .5)); // Give user time to take hands off X.
        servo.setPosition(0.5);
        motor.setPower(0);
    }
}