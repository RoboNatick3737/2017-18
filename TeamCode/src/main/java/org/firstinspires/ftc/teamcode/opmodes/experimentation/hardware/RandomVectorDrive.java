package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.EncoderMotor;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveDrive;

import hankextensions.EnhancedOpMode;
import hankextensions.phonesensors.Gyro;
import hankextensions.structs.Vector2D;

@Autonomous(name="Random Vector Drive", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class RandomVectorDrive extends EnhancedOpMode
{
    private EncoderMotor frontLeft, backLeft, frontRight, backRight;

    public class FakeGyro implements Gyro
    {
        @Override
        public void initAntiDrift() throws InterruptedException {

        }

        @Override
        public void startAntiDrift() throws InterruptedException {

        }

        @Override
        public void zero() throws InterruptedException {

        }

        @Override
        public double getHeading() {
            return 0;
        }
    }

    @Override
    protected void onRun() throws InterruptedException
    {
        Robot robot = new Robot(hardware, Robot.InitializationMode.TELEOP);
        robot.swerveDrive.gyro = new FakeGyro();
        robot.swerveDrive.setJoystickControlEnabled(false);
        robot.swerveDrive.setControlMethod(SwerveDrive.ControlMethod.FIELD_CENTRIC);
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        waitForStart();

        Vector2D toDrive = Vector2D.rectangular(Math.random() * 0.5 - 0.25, Math.random() * 0.5 - 0.25);
        robot.swerveDrive.setDesiredMovement(toDrive);
        log.lines("Drive is " + toDrive.toString(Vector2D.VectorCoordinates.RECTANGULAR));

        while (true)
        {
            robot.swerveDrive.synchronousUpdate();

            flow.yield();
        }
    }
}
