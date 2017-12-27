package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class BallKnocker
{
    private final double UP = 0.1, DOWN = 0.7;
    private boolean up = false;

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
