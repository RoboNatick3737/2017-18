package org.firstinspires.ftc.teamcode.robot.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import hankextensions.RobotCore;
import hankextensions.phonesensors.Gyro;
import hankextensions.structs.Vector2D;

public class HankuTankuIMU implements Gyro
{
    public final BNO055IMU imu;

    private double resetOffset = 0;

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

    @Override
    public void calibrate() throws InterruptedException
    {
    }

    @Override
    public void zero() throws InterruptedException
    {
        // resetOffset = Vector2D.clampAngle(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }

    @Override
    public double x() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
    }

    @Override
    public double y() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
    }

    @Override
    public double z() {
        return Vector2D.clampAngle(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - resetOffset);
    }
}
