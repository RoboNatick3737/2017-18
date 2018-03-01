package hankutanku.phonesensors;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import hankutanku.math.Vector2D;

public class AndroidGyro implements Gyro
{
    // Singleton.
    public static AndroidGyro instance;

    public boolean active = false;

    // Required for calculations
    private SensorManager sensorManager;
    private Sensor sensor;
    private static final float NS2S = 1.0f / 1000000000.0f;
    private final float[] deltaRotationVector = new float[4];
    private float timestamp;
    private static final double EPSILON = 0.05f;

    // Tells us the rate of drift to counteract.
    private static long startCalibrateTime = 0;
    private static double driftRate = 0;

    // Tells the gyro when to reset the readings (set temporarily).
    private static boolean reset = false;

    // The final readings of the gyro.
    public static double x = 0, y = 0, z = 0;

    // Create a listener
    private SensorEventListener gyroscopeSensorListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent event) {
            // This timestep's delta rotation to be multiplied by the current rotation
            // after computing it from the gyro sample data.
            if (timestamp != 0) {
                final float dT = (event.timestamp - timestamp) * NS2S;
                // Axis of the rotation sample, not normalized yet.
                float axisX = event.values[0];
                float axisY = event.values[1];
                float axisZ = event.values[2];

                // Calculate the angular speed of the sample
                float omegaMagnitude = (float) Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);

                // Normalize the rotation vector if it's big enough to get the axis
                // (that is, EPSILON should represent your maximum allowable margin of error)
                if (omegaMagnitude > EPSILON) {
                    axisX /= omegaMagnitude;
                    axisY /= omegaMagnitude;
                    axisZ /= omegaMagnitude;
                }

                // Integrate around this axis with the angular speed by the timestep
                // in order to get a delta rotation from this sample over the timestep
                // We will convert this axis-angle representation of the delta rotation
                // into a quaternion before turning it into the rotation matrix.
                float thetaOverTwo = omegaMagnitude * dT / 2.0f;
                float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
                float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
                deltaRotationVector[0] = sinThetaOverTwo * axisX;
                deltaRotationVector[1] = sinThetaOverTwo * axisY;
                deltaRotationVector[2] = sinThetaOverTwo * axisZ;
                deltaRotationVector[3] = cosThetaOverTwo;
                double delta_x = 2 * Math.toDegrees(deltaRotationVector[0]);
                double delta_y = 2 * Math.toDegrees(deltaRotationVector[1]);
                double delta_z = 2 * Math.toDegrees(deltaRotationVector[2]);
                x += delta_x;
                y += delta_y;
                z += delta_z;
            }
            timestamp = event.timestamp;

            if (reset) {
                x = 0;
                y = 0;
                z = 0;
                reset = false;
            }
        }


        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {
        }
    };

    // Initialize all of the hardware variables
    public AndroidGyro()
    {
        sensorManager = (SensorManager) FtcRobotControllerActivity.instance.getSystemService(Context.SENSOR_SERVICE);
        sensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        instance = this;
    }

    /**
     * Zeroes the gyro.
     */
    public void initAntiDrift()
    {
        startCalibrateTime = System.currentTimeMillis();
        zero();
    }

    /**
     * Calculates the rate at which the gyro is drifting.
     */
    public void startAntiDrift()
    {
        double diff = Vector2D.clampAngle(360 - y);
        diff = diff > 180 ? diff - 360 : diff;

        driftRate = diff / ((System.currentTimeMillis() - startCalibrateTime) / 1000.0);
        zero();
    }

    /**
     * Zeroes the gyro.
     */
    public void zero()
    {
        reset = true;
    }

    private double offset = 0;
    public void applyOffset(double offset)
    {
        this.offset = offset;
    }

    /**
     * Returns the current heading of the bot.
     * @return the heading of the bot.
     */
    public double getHeading()
    {
        return Vector2D.clampAngle(Vector2D.clampAngle(360 - y)
                - driftRate * ((System.currentTimeMillis() - startCalibrateTime) / 1000.0)
                - offset);
    }

    /**
     * Causes the listener to start receiving information.
     */
    public void start()
    {
        if (active)
            return;

        sensorManager.registerListener(gyroscopeSensorListener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
        active = true;
    }

    /**
     * The idea here is the gyro always quits upon being asked to stop, so it doesn't take up
     * excessive CPU.
     */
    public void quit()
    {
        if (!active)
            return;

        sensorManager.unregisterListener(gyroscopeSensorListener, sensor);
        active = false;
    }
}