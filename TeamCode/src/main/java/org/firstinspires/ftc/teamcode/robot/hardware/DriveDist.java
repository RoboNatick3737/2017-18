package org.firstinspires.ftc.teamcode.robot.hardware;

import com.makiah.makiahsandroidlib.threading.ScheduledTaskPackage;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.opmodes.CompetitionProgram;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.structs.Polynomial;
import org.firstinspires.ftc.teamcode.structs.VariableVector2D;

import hankextensions.EnhancedOpMode;
import hankextensions.structs.Vector2D;

@Autonomous(name="Drive Dists", group= Constants.FINAL_BOT_EXPERIMENTATION)
public class DriveDist extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        Robot robot = new Robot(hardware, Robot.ControlMode.AUTONOMOUS);

        robot.swerveDrive.setControlMethod(SwerveDrive.ControlMethod.FIELD_CENTRIC);
        robot.swerveDrive.setSwerveUpdateMode(ScheduledTaskPackage.ScheduledUpdateMode.SYNCHRONOUS);

        robot.swerveDrive.driveDistance(Vector2D.ZERO, 200, flow);
        robot.swerveDrive.driveDistance(Vector2D.polar(0.5, 180), 200, flow);
        robot.swerveDrive.driveDistance(Vector2D.polar(0.5, 270), 200, flow);
        robot.swerveDrive.driveDistance(Vector2D.polar(0.5, 0), 200, flow);
    }
}
