package org.firstinspires.ftc.teamcode.robot.hardware;

import dude.makiah.androidlib.threading.Flow;
import dude.makiah.androidlib.threading.TimeMeasure;

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
        setKnockerTo(KnockerPosition.RIGHT);
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

        descender.setPosition(up ? 0.2 : 0.77);
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
                knocker.setPosition(0.5);
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

        flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, .8));

        setKnockerTo(toKnock);
        flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, .8));

        setUpwardPosTo(true);
    }
}
