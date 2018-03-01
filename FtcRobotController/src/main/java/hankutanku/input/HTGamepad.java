package hankutanku.input;

import com.qualcomm.robotcore.hardware.Gamepad;

import hankutanku.math.Angle;
import hankutanku.math.Vector2D;

/**
 * A gamepad class which keeps track of intelligent robot control.
 */
public class HTGamepad
{
    // Singletons for when we need to init the gamepads.
    public static HTGamepad CONTROLLER1, CONTROLLER2;
    public enum ControllerID { CONTROLLER_1, CONTROLLER_2 }

    // The gamepad buttons.
    public final HTButton
            a = new HTButton(),
            b = new HTButton(),
            x = new HTButton(),
            y = new HTButton(),
            left_bumper = new HTButton(),
            right_bumper = new HTButton();

    /**
     * The gamepad reference to which this corresponds.
     */
    public final Gamepad gamepad;
    public HTGamepad(Gamepad gamepad, ControllerID controllerID)
    {
        this.gamepad = gamepad;

        if (controllerID == ControllerID.CONTROLLER_1)
            CONTROLLER1 = this;
        else if (controllerID == ControllerID.CONTROLLER_2)
            CONTROLLER2 = this;
    }

    /**
     * Needs to be called during every loop cycle.
     */
    public void update()
    {
        a.state(gamepad.a);
        b.state(gamepad.b);
        x.state(gamepad.x);
        y.state(gamepad.y);
        left_bumper.state(gamepad.left_bumper);
        right_bumper.state(gamepad.right_bumper);
    }

    public Vector2D rightJoystick()
    {
        return Vector2D.rectangular(gamepad.right_stick_x, -gamepad.right_stick_y).rotateBy(Angle.degrees(-90));
    }

    public Vector2D leftJoystick()
    {
        return Vector2D.rectangular(gamepad.left_stick_x, -gamepad.left_stick_y).rotateBy(Angle.degrees(-90));
    }

    public Vector2D dpad()
    {
        return Vector2D.rectangular((gamepad.dpad_left ? 1 : 0) + (gamepad.dpad_right ? -1 : 0), (gamepad.dpad_up ? 1 : 0) + (gamepad.dpad_down ? -1 : 0)).unit();
    }
}
