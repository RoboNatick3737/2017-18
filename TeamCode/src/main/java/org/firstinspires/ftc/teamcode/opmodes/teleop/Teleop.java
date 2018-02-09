package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeDisplayGroups;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.hardware.BallKnocker;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveDrive;
import org.firstinspires.ftc.teamcode.robot.hardware.SwerveModule;
import org.firstinspires.ftc.teamcode.structs.PIDController;

import hankextensions.EnhancedOpMode;
import hankextensions.input.HTButton;

@TeleOp(name="Teleop", group= OpModeDisplayGroups.FINAL_BOT_OPMODES)
public class Teleop extends EnhancedOpMode
{
    /**
     * The teleop controls for our swerve drive and such.
     */
    @Override
    protected final void onRun() throws InterruptedException
    {
        Robot robot = new Robot(hardware, Robot.ControlMode.TELEOP);

        // Enable logging TODO remove
//        for (SwerveModule module : robot.swerveDrive.swerveModules)
//            module.setEnableLogging(true);

        // Set enable logging for relic arm.
        robot.relicSystem.setEnableLogging(true);

        // Synchronous teleop
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);
        robot.swerveDrive.setControlMethod(SwerveDrive.ControlMethod.TANK_DRIVE);

        // Init robot hardware.
        robot.flipper.advanceStage(0);
        robot.intake.stop();

        waitForStart();

        robot.gyro.startAntiDrift();

        ProcessConsole teleopConsole = log.newProcessConsole("Teleop Console");

        // Constant driver mode stuff.
        boolean seanInIntakeMode = true;

        // The front left swerve module has a hard time dealing with the torque on it.
        PIDController frontLeftkPController = robot.swerveDrive.swerveModules[0].errorResponder instanceof PIDController ? (PIDController)(robot.swerveDrive.swerveModules[0].errorResponder) : null;

        double originalFrontLeftkP = 0;
        double higherFrontLeftkP = 0;
        boolean atHigherkP = false;

        if (frontLeftkPController != null)
        {
            originalFrontLeftkP = frontLeftkPController.kP;
            higherFrontLeftkP = originalFrontLeftkP * 1.2;
        }

        while (true)
        {
            // Update controllers
            C1.update();
            C2.update();

            if (C1.gamepad.dpad_up)
            {
                seanInIntakeMode = false;
//                robot.lights.lights.setPower(0.5);
            }
            else if (C1.gamepad.dpad_down)
            {
                seanInIntakeMode = true;
//                robot.lights.lights.setPower(0);
            }

            // region Driver 1 Swerve Control
            // Update swerve drive
            if (C1.b.currentState == HTButton.ButtonState.JUST_TAPPED) {
                if (robot.swerveDrive.getSwerveSpeedMode() == SwerveDrive.SwerveSpeedMode.FAST)
                    robot.swerveDrive.setSwerveSpeedMode(SwerveDrive.SwerveSpeedMode.SLOW);
                else
                    robot.swerveDrive.setSwerveSpeedMode(SwerveDrive.SwerveSpeedMode.FAST);
            }
            robot.swerveDrive.synchronousUpdate();
            //endregion

            // region Driver 1: Intake Mode
            if (seanInIntakeMode)
            {
                if (atHigherkP)
                {
                    if (frontLeftkPController != null)
                        frontLeftkPController.kP = originalFrontLeftkP;

                    atHigherkP = false;

                    robot.relicSystem.extensionEstimate = 0;
                }

                // Control flipper
                if (C1.x.currentState == HTButton.ButtonState.JUST_TAPPED)
                    robot.flipper.advanceStage();
                robot.flipper.update(); // in case we're doing the timed lift thing.

                // Control intake
                if (C1.gamepad.left_bumper)
                    robot.intake.expel();
                else if (C1.gamepad.right_bumper)
                    robot.intake.intake();
                else if (C1.gamepad.left_trigger > .03 || C1.gamepad.right_trigger > .03) {
                    robot.intake.leftIntake(C1.gamepad.left_trigger);
                    robot.intake.rightIntake(C1.gamepad.right_trigger);
                    robot.intake.secondaryIntake(1);
                } else
                    robot.intake.stop();
            }

            // endregion

            // region Driver 1: Relic Control
            else
            {
                // Controls the relic arm
                if (C1.x.currentState == HTButton.ButtonState.JUST_TAPPED)
                    robot.relicSystem.toggleGrabber();

                robot.relicSystem.variableExtension(C1.gamepad.right_trigger, C1.gamepad.left_trigger);

                if (!atHigherkP && robot.relicSystem.extensionEstimate > 5000)
                {
                    if (frontLeftkPController != null)
                    {
                        frontLeftkPController.kP = higherFrontLeftkP;
                        atHigherkP = true;
                    }
                }

                if (atHigherkP && robot.relicSystem.extensionEstimate < 5000)
                {
                    if (frontLeftkPController != null)
                    {
                        frontLeftkPController.kP = originalFrontLeftkP;
                        atHigherkP = false;
                    }
                }
            }
            //endregion

            // region Driver 2: Lift Control
            // Control the lift.
            if (C2.gamepad.dpad_up)
                robot.lift.up();
            else if (C2.gamepad.dpad_down)
                robot.lift.down();
            else
                robot.lift.stop();
            // endregion

            teleopConsole.write(
                    "In " + (seanInIntakeMode ? "intake mode" : "relic arm mode"),
                    "At higher kp = " + atHigherkP);

            flow.yield();
        }
    }
}
