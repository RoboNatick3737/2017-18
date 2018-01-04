package org.firstinspires.ftc.teamcode.opmodes;

public interface CompetitionProgram
{
    enum Alliance {RED, BLUE}
    enum BalancePlate {TOP, BOTTOM}

    Alliance getAlliance();
    BalancePlate getBalancePlate();
}
