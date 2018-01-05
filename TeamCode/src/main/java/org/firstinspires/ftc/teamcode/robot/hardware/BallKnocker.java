package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class BallKnocker
{
    private final double UP = 0, DOWN = 0.6;
    private boolean up = true;

    private final Servo ballKnocker;
    public BallKnocker(Servo ballKnocker)
    {
        this.ballKnocker = ballKnocker;

        setKnockerTo(true);
    }

    public void setKnockerTo(boolean up)
    {
        this.up = up;

        ballKnocker.setPosition(up ? UP : DOWN);
    }
    public void toggleKnocker()
    {
        setKnockerTo(!up);
    }
}
