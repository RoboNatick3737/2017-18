package hankutanku.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareInitializer
{
    public final HardwareMap map;

    public HardwareInitializer(HardwareMap map)
    {
        this.map = map;
    }

    /**
     * Use by calling DcMotor dcMotor = NiFTInitializer.initialize(DcMotor.class, "name in config");
     */
    public <T extends HardwareDevice> T initialize(Class<T> hardwareDevice, String name)
    {
        try
        {
            //Returns the last subclass (if this were a DcMotor it would pass back a Dc Motor.
            return hardwareDevice.cast (map.get (name));
        }
        catch (Exception e) //There might be other exceptions that this throws, not entirely sure about which so I am general here.
        {
            throw new NullPointerException ("Couldn't find " + name + " in the configuration file!");
        }
    }
}
