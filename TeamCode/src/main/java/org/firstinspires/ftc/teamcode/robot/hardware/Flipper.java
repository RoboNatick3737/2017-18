package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import hankutanku.math.Function;
import hankutanku.math.TimedFunction;

/**
 * The left and right flippers for the glyphs.
 */
public class Flipper
{
    // region Flipper Servos
    private final Servo left;
    private int currentStage = -1;
    private enum FlipPosition
    {
        MIN(1),
        MID_START(.65),
        MID_END(.45),
        MAX(.3);

        public final double position;
        FlipPosition(double position)
        {
            this.position = position;
        }
    }
    // endregion

    // region Glyph Clamper
    private final Servo glyphClamp;
    private enum GlyphClamp
    {
        CLAMP(0.8),
        FREE(0.5);

        public final double position;
        GlyphClamp(double position)
        {
            this.position = position;
        }
    }
    // endregion

    // region Glyph Holder
    private final Servo glyphHolder;
    private enum GlyphHolder {
        BLOCKING(1),
        AWAY(0);

        public final double position;

        GlyphHolder(double position) {
            this.position = position;
        }
    }
    // endregion

    // The current servo position (getPosition() doesn't seem to work).
    private double position = FlipPosition.MIN.position;
    private TimedFunction liftFunc = null;

    public Flipper(Servo left, Servo glyphClamp, Servo glyphHolder)
    {
        this.left = left;
        this.glyphClamp = glyphClamp;
        this.glyphHolder = glyphHolder;

        advanceStage(0);
    }

    private void updateFlipperPositions()
    {
        left.setPosition(Range.clip(position, 0, 1));
    }

    public void advanceStage(int stage)
    {
        if (stage == currentStage)
            return;

        currentStage = stage;

        switch(stage)
        {
            case 0:
                position = FlipPosition.MIN.position;
                glyphClamp.setPosition(GlyphClamp.FREE.position);
                glyphHolder.setPosition(GlyphHolder.BLOCKING.position);
                liftFunc = null;
                break;

            case 1:
                position = FlipPosition.MID_START.position;
                glyphClamp.setPosition(GlyphClamp.CLAMP.position);
                liftFunc = new TimedFunction(new Function() {
                    @Override
                    public double value(double input) {
                        return -.25 * input + FlipPosition.MID_START.position; // gradient lift
                    }
                });
                glyphHolder.setPosition(GlyphHolder.BLOCKING.position);
                break;

            case 2:
                position = FlipPosition.MAX.position;
                glyphClamp.setPosition(GlyphClamp.FREE.position);
                glyphHolder.setPosition(GlyphHolder.AWAY.position);
                liftFunc = null;
                break;
        }

        updateFlipperPositions();
    }

    /**
     * Kinda weird, used to slowly advance the flipper instead of going super quick.
     */
    public void update()
    {
        if (liftFunc == null)
            return;

        position = liftFunc.value();
        if (position < FlipPosition.MID_END.position)
        {
            liftFunc = null;
            return;
        }

        updateFlipperPositions();
    }

    public void advanceStage()
    {
        if (currentStage == 2)
            advanceStage(0);
        else
            advanceStage(currentStage + 1);
    }

    // Used for autonomous control
    public void setFlipperPositionManually(double position)
    {
        this.position = position;
        updateFlipperPositions();
    }
}
