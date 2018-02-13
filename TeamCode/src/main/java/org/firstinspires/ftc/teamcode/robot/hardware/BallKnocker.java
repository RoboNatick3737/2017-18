package org.firstinspires.ftc.teamcode.robot.hardware;

import com.makiah.makiahsandroidlib.threading.Flow;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class BallKnocker
{
    private boolean up = true;

    private final Servo descender, knocker;

    public BallKnocker(Servo descender, Servo knocker)
    {
        this.descender = descender;
        this.knocker = knocker;

        setUpwardPosTo(true);
        knocker.setPosition(0);
    }

    private double currentPos = 0;
    public void updatePosition(double increment)
    {
        currentPos += increment;
        currentPos = Range.clip(currentPos, 0, 1);
        knocker.setPosition(currentPos);
    }

    public void setUpwardPosTo(boolean up)
    {
        this.up = up;

        descender.setPosition(up ? -1.2 : 0.6);
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
                knocker.setPosition(.1);
                break;

            case MIDDLE:
                knocker.setPosition(0.6);
                break;

            case RIGHT:
                knocker.setPosition(1);
                break;
        }
    }

    public void setKnockerManual(double pos)
    {
        knocker.setPosition(Range.clip(pos, 0, 1));
    }

    /**
     * Run during auto, knocks the left ball off the holder.
     */
    public void knockBall(KnockerPosition toKnock, Flow flow) throws InterruptedException
    {
        setKnockerTo(KnockerPosition.MIDDLE);
        setUpwardPosTo(false);

        flow.msPause(800);

        setKnockerTo(toKnock);
        flow.msPause(800);

        setUpwardPosTo(true);
    }
}
