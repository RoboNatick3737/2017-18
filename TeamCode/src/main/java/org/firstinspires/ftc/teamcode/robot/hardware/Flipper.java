package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * The left and right flippers for the glyphs.
 */
public class Flipper
{
    private final double FLIP_INCREMENT = .02, FLIP_MAX = .137, FLIP_MID = .675, FLIP_MIN = 1;
    private final double RIGHT_FLIPPER_OFFSET = .08;

    private int currentStage = 0;

    private final Servo left, right;

    // The current servo position (getPosition() doesn't seem to work).
    private double position = FLIP_MIN;

    public Flipper(Servo left, Servo right)
    {
        this.left = left;
        this.right = right;
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
                break;
            case 1:
                position = FLIP_MID;
                break;
            case 2:
                position = FLIP_MAX;
                break;
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

    public void move(double upInput, double downInput)
    {
        position += FLIP_INCREMENT * (upInput - downInput);
        position = Range.clip(position, FLIP_MIN, FLIP_MAX);
        updateFlipperPositions();
    }
}
