package org.firstinspires.ftc.teamcode;

import hankextensions.EnhancedOpMode;

/**
 * For when you need the context of the alliance and the balance plate upon which you start.
 */
public abstract class CompetitionOpMode extends EnhancedOpMode
{
    public enum Alliance { RED, BLUE }
    public enum BalancePlateLocation { TOP, BOTTOM }

    // Child opmodes must override.
    protected abstract Alliance getAlliance();
    protected abstract BalancePlateLocation getBalancePlateLocation();
}
