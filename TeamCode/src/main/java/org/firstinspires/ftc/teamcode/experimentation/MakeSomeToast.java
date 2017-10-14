package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;

import hankstanks.hankextensions.Core;

@Autonomous(name="Thing", group="blep")
public class MakeSomeToast extends Core
{
    protected void START() throws InterruptedException
    {
        // MAKATTACK
        AppUtil.getInstance().showToast(UILocation.BOTH, "It's ya boi... MAKATTACK");
        // MAKATTACK
    }
}
