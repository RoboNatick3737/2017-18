package org.firstinspires.ftc.teamcode.robot.hardware;

import com.makiah.makiahsandroidlib.threading.Flow;
import com.qualcomm.robotcore.hardware.Servo;

public class BallKnocker
{
    private boolean up = true;

    private final Servo descender, knocker;

    public BallKnocker(Servo descender, Servo knocker)
    {
        this.descender = descender;
        this.knocker = knocker;

        setUpwardPosTo(true);
        setKnockerTo(KnockerPosition.LEFT);
    }

    public void setUpwardPosTo(boolean up)
    {
        this.up = up;

        descender.setPosition(up ? -1.2 : 0.7);
    }
    public void toggleDescender()
    {
        setUpwardPosTo(!up);
    }

    public enum KnockerPosition {LEFT, MIDDLE, RIGHT}
    public void setKnockerTo(KnockerPosition pos)
    {
        switch (pos)
        {
            case LEFT:
                knocker.setPosition(0);
                break;

            case MIDDLE:
                knocker.setPosition(0.45);
                break;

            case RIGHT:
                knocker.setPosition(0.9);
                break;
        }
    }

    /**
     * Run during auto, knocks the left ball off the holder.
     */
    public void knockBall(KnockerPosition toKnock, Flow flow) throws InterruptedException
    {
        setKnockerTo(KnockerPosition.MIDDLE);
        setUpwardPosTo(false);

        flow.msPause(1000);

        setKnockerTo(toKnock);
        flow.msPause(1000);

        setUpwardPosTo(true);
    }
}
