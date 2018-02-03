package org.firstinspires.ftc.teamcode.opmodes.experimentation.hardware;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.ScheduledTask;
import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveModule;
import org.firstinspires.ftc.teamcode.structs.PIDController;

import hankextensions.EnhancedOpMode;

@TeleOp(name="Tune Swerve Module PID", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class TuneSwerveModulePID extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        SwerveModule[] modules = Robot.getSwerveModules(hardware, DcMotor.ZeroPowerBehavior.FLOAT);

        // All must be PIDController instances or we can't run this.
        for (SwerveModule module : modules)
        {
            if (!(module.errorResponder instanceof PIDController))
            {
                log.lines("Can't continue");
                flow.msPause(5000);
                return;
            }
        }

        int currentIndex = 0;
        modules[0].setEnableLogging(true);

        ScheduledTaskPackage tasks = new ScheduledTaskPackage(this, "Swerve", modules[0], modules[1], modules[2], modules[3], modules[0].driveMotor, modules[1].driveMotor, modules[2].driveMotor, modules[3].driveMotor);
        tasks.setUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        double adjuster = .00001;

        ProcessConsole adjusterConsole = log.newProcessConsole("Adjuster");

        long lastToggle = System.currentTimeMillis();
        while (true)
        {
            for (SwerveModule module : modules)
            {
                module.setVectorTarget(C1.leftJoystick().multiply(70));
            }

            if (C1.gamepad.dpad_down && (System.currentTimeMillis() - lastToggle) > 500)
            {
                modules[currentIndex].setEnableLogging(false);
                currentIndex--;
                if (currentIndex < 0)
                    currentIndex = 3;
                modules[currentIndex].setEnableLogging(true);

                lastToggle = System.currentTimeMillis();
            }
            else if (C1.gamepad.dpad_up && (System.currentTimeMillis() - lastToggle) > 500)
            {
                modules[currentIndex].setEnableLogging(false);
                currentIndex++;
                if (currentIndex > 3)
                    currentIndex = 0;
                modules[currentIndex].setEnableLogging(true);

                lastToggle = System.currentTimeMillis();
            }

            if (gamepad1.left_bumper) {
                adjuster = .001;
            }

            if (gamepad1.left_trigger > .03)
            {
                adjuster = .00001;
            }

            if (gamepad1.right_trigger > .03)
            {
                adjuster = .000001;
            }

            if (gamepad1.right_bumper)
            {
                adjuster = .0000001;
            }

            PIDController current = (PIDController) modules[currentIndex].errorResponder;
            if (gamepad1.a)
                current.kP -= adjuster;
            else if (gamepad1.y)
                current.kP += adjuster;

            if (gamepad1.x)
                current.kD -= adjuster;
            else if (gamepad1.b)
                current.kD += adjuster;

            if (gamepad1.dpad_left)
                current.kI -= adjuster;
            else if (gamepad1.dpad_right)
                current.kI += adjuster;

            boolean drivingCanStart = true;
            for (SwerveModule wheel : modules)
            {
                if (!wheel.atAcceptableSwivelOrientation())
                {
                    drivingCanStart = false;
                    break;
                }
            }
            for (SwerveModule wheel : modules)
                wheel.setDrivingState(drivingCanStart);

            if (C2.gamepad.a)
                SwerveModule.TORQUE_CORRECTION_FACTOR -= adjuster;
            else if (C2.gamepad.y)
                SwerveModule.TORQUE_CORRECTION_FACTOR += adjuster;

            tasks.synchronousUpdate();

            adjusterConsole.write("Adjuster = " + adjuster, "TCF: " + SwerveModule.TORQUE_CORRECTION_FACTOR);

            flow.yield();
        }
    }
}
