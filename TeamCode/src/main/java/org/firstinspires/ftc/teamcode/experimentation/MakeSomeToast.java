package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;

import hankextensions.EnhancedOpMode;

@Autonomous(name="Make Some Toast", group="Experimentation")
public class MakeSomeToast extends EnhancedOpMode
{
    protected void onRun() throws InterruptedException
    {
        AppUtil.getInstance().showToast(UILocation.BOTH, "It's ya boi... MAKATTACK");
    }
}
