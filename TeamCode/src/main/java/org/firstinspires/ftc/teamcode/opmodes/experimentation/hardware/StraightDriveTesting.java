package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.EncoderMotor;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveDrive;
import org.firstinspires.ftc.teamcode.structs.pid.PIDConstants;

import hankextensions.EnhancedOpMode;
import hankextensions.phonesensors.Gyro;
import hankextensions.structs.Vector2D;

@Autonomous(name="Straight Line Drive", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class StraightDriveTesting extends EnhancedOpMode
{
    private EncoderMotor frontLeft, backLeft, frontRight, backRight;

    public class FakeGyro implements Gyro
    {
        @Override
        public void calibrate() throws InterruptedException {

        }

        @Override
        public void zero() throws InterruptedException {

        }

        @Override
        public double x() {
            return 0;
        }

        @Override
        public double y() {
            return 0;
        }

        @Override
        public double z() {
            return 0;
        }
    }

    @Override
    protected void onRun() throws InterruptedException
    {
        Robot robot = new Robot(hardware);
        robot.swerveDrive.gyro = new FakeGyro();
        robot.swerveDrive.setJoystickControlEnabled(false);
        robot.swerveDrive.setControlMethod(SwerveDrive.ControlMethod.FIELD_CENTRIC);
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        waitForStart();

        robot.swerveDrive.setDesiredMovement(Vector2D.polar(0.5, 0));

        while (true)
        {
            robot.swerveDrive.synchronousUpdate();

            flow.yield();
        }
    }
}
