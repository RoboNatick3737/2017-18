package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LightingSystem
{
    private static final double LIGHTS_ON = 1.0, LIGHTS_OFF = 0;

    public final DcMotor lights;
    private boolean on = false;
    public LightingSystem(DcMotor lights)
    {
        this.lights = lights;
        setLightsTo(false);
    }

    public void setLightsTo(boolean on)
    {
        this.lights.setPower(on ? LIGHTS_ON : LIGHTS_OFF);
        this.on = on;
    }

    public void toggleLights()
    {
        setLightsTo(!on);
    }
}
