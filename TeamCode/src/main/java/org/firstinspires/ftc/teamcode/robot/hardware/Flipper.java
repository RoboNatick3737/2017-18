package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.structs.Function;
import org.firstinspires.ftc.teamcode.structs.TimedFunction;

/**
 * The left and right flippers for the glyphs.
 */
public class Flipper
{
    private static final double FLIP_MIN = 1, FLIP_MID_START = .8, FLIP_MID_END = .65, FLIP_MAX = .35;
    private static final double RIGHT_FLIPPER_OFFSET = .19;
    private static final double GLYPH_HOLDER_UP = 0.5, GLYPH_HOLDER_DOWN = 0;

    private int currentStage = 0;

    private final Servo left, right, glyphHolder;

    // The current servo position (getPosition() doesn't seem to work).
    private double position = FLIP_MIN;
    private TimedFunction liftFunc = null;

    public Flipper(Servo left, Servo right, Servo glyphHolder)
    {
        this.left = left;
        this.right = right;
        this.glyphHolder = glyphHolder;

        advanceStage(0);
    }

    private void updateFlipperPositions()
    {
        left.setPosition(Range.clip(position, 0, 1));
        right.setPosition(Range.clip(1 + RIGHT_FLIPPER_OFFSET - position, 0, 1));
    }

    public void advanceStage(int stage)
    {
        switch(stage)
        {
            case 0:
                position = FLIP_MIN;
                glyphHolder.setPosition(GLYPH_HOLDER_UP);
                liftFunc = null;
                break;

            case 1:
                position = FLIP_MID_START;
                glyphHolder.setPosition(GLYPH_HOLDER_UP);
                liftFunc = new TimedFunction(new Function() {
                    @Override
                    public double value(double input) {
                        return -.25 * input + FLIP_MID_START; // gradient lift
                    }
                });
                break;

            case 2:
                position = FLIP_MAX;
                glyphHolder.setPosition(GLYPH_HOLDER_DOWN);
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
        if (position < FLIP_MID_END)
        {
            liftFunc = null;
            return;
        }

        updateFlipperPositions();
    }

    public void advanceStage()
    {
        currentStage++;
        if (currentStage > 2)
            currentStage = 0;

        advanceStage(currentStage);
    }

    // Used for autonomous control
    public void setFlipperPositionManually(double position)
    {
        this.position = position;
        updateFlipperPositions();
    }

    public void setGlyphHolderUpTo(boolean up)
    {
        glyphHolder.setPosition(up ? GLYPH_HOLDER_UP : GLYPH_HOLDER_DOWN);
    }
}
