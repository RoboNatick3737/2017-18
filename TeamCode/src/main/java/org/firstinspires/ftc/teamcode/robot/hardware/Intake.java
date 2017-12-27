package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * The left and right flippers for the glyphs.
 */
public class Intake
{
    private final double IN_SPEED = 1, OUT_SPEED = -1;
    private final double STAGE_UP = 0.1, STAGE_DOWN = 0.75;

    private final DcMotor harvester;
    private final Servo harvesterStager;
    private final Servo conveyor;

    public Intake(DcMotor harvester, Servo harvesterStager, Servo conveyor)
    {
        this.harvester = harvester;
        this.conveyor = conveyor;
        this.harvesterStager = harvesterStager;

        setHarvesterTo(false);
    }

    public void intake()
    {
        this.harvester.setPower(IN_SPEED);
        this.conveyor.setPosition(1);
    }

    public void expel()
    {
        this.harvester.setPower(OUT_SPEED);
        this.conveyor.setPosition(0);
    }

    public void stop()
    {
        this.harvester.setPower(0);
        this.conveyor.setPosition(0.5);
    }

    private boolean harvesterUp = false;
    public void setHarvesterTo(boolean up)
    {
        harvesterUp = up;

        harvesterStager.setPosition(up ? STAGE_UP : STAGE_DOWN);
    }
    public void toggleHarvesterStage()
    {
        setHarvesterTo(!harvesterUp);
    }

    public void variable(double in, double out)
    {
        double speed = IN_SPEED * in + OUT_SPEED * out;

        this.harvester.setPower(Math.abs(speed) > .2 ? speed : 0);
        this.conveyor.setPosition(speed * 0.5 + 0.5);
    }
}
