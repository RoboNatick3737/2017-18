package org.firstinspires.ftc.teamcode.experimentation;

import com.makiah.makiahsandroidlib.logging.ProcessConsole;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.HankuTankuIMU;

import hankextensions.RobotCore;

@Autonomous(name="Test IMU", group= Constants.EXPERIMENTATION)
public class TestIMU extends RobotCore
{
    private HankuTankuIMU imu;

    @Override
    protected void HARDWARE()
    {
        // Have to use hardwareMap.get because BNO055 isn't a HardwareDevice instance apparently :/
        imu = new HankuTankuIMU(hardwareMap.get(BNO055IMU.class, "IMU"));
    }

    @Override
    protected void START() throws InterruptedException
    {
        ProcessConsole imuProcessConsole = log.newProcessConsole("IMU Output");
        while (true)
        {
            imuProcessConsole.write("Orientations: " + imu.imu.getAngularOrientation().toString());

            flow.yield();
        }
    }
}
