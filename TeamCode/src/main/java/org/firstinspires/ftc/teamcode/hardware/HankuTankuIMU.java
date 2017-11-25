package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import hankextensions.RobotCore;

public class HankuTankuIMU
{
    public final BNO055IMU imu;

    public HankuTankuIMU(BNO055IMU imu)
    {
        this.imu = imu;

        initializeIMU();
    }

    private void initializeIMU()
    {
        RobotCore.instance.log.lines("Initializing IMU...");

        // Set up the required parameters for the IMU initialization.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        RobotCore.instance.log.lines("IMU initialized!");
    }
}
