package org.firstinspires.ftc.teamcode.robot.hardware;

import dude.makiah.androidlib.logging.LoggingBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import hankutanku.phonesensors.Gyro;
import hankutanku.math.Angle;

public class HankuTankuIMU implements Gyro
{
    public final BNO055IMU imu;

    private Angle resetOffset = Angle.ZERO;

    public HankuTankuIMU(BNO055IMU imu)
    {
        this.imu = imu;

        initializeIMU();
    }

    private void initializeIMU()
    {
        LoggingBase.instance.lines("Initializing IMU...");

        // Set up the required parameters for the IMU initialization.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        LoggingBase.instance.lines("IMU initialized!");
    }

    @Override
    public void initAntiDrift() throws InterruptedException
    {
    }

    @Override
    public void startAntiDrift() throws InterruptedException
    {
    }

    @Override
    public void applyOffset(double offset) {
    }

    @Override
    public void zero() throws InterruptedException
    {
         resetOffset = Angle.degrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }

    @Override
    public Angle getHeading() {
        return z();
    }

    public Angle x() {
        return Angle.degrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
    }

    public Angle y() {
        return Angle.degrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
    }

    public Angle z() {
        return Angle.degrees(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle).subtract(resetOffset);
    }
}
