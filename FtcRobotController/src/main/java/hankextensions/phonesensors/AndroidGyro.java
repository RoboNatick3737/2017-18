package hankextensions.phonesensors;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

public class AndroidGyro implements Gyro
{
    public static AndroidGyro instance;

    public boolean active = false;

    private SensorManager sensorManager;
    private Sensor sensor;
    private static final float NS2S = 1.0f / 1000000000.0f;
    private final float[] deltaRotationVector = new float[4];
    private float timestamp;
    private static final double EPSILON = 0.05f;
    private float omegaMagnitude = 0;

    static boolean reset = false;
    static double x = 0;
    static double y = 0;
    static double z = 0;
    double delta_x = 0;
    double delta_y = 0;
    double delta_z = 0;

    // Create a listener
    SensorEventListener gyroscopeSensorListener = new SensorEventListener() {
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
                omegaMagnitude = (float) Math.sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);

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
                delta_x = 2 * Math.toDegrees(deltaRotationVector[0]);
                delta_y = 2 * Math.toDegrees(deltaRotationVector[1]);
                delta_z = 2 * Math.toDegrees(deltaRotationVector[2]);
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
    public AndroidGyro() {
        sensorManager = (SensorManager) FtcRobotControllerActivity.instance.getSystemService(Context.SENSOR_SERVICE);
        sensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        instance = this;
    }

    public static double wrapAngle(double angle) {
        angle %= 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    public void calibrate() {
        zero();
    }

    public void zero() {
        reset = true;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public double z() {
        return z;
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