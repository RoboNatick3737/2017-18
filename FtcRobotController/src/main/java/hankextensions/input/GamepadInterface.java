package hankextensions.input;

import com.qualcomm.robotcore.hardware.Gamepad;
import hankextensions.structs.Vector2D;

public class GamepadInterface
{
    /**
     * The wait time before the controller registers that a distinct button tap has occurred.
     */
    private static long BUTTON_TAP_LATENCY = 200;

    /**
     * The gamepad reference to which this corresponds.
     */
    public final Gamepad gamepad;

    public GamepadInterface(Gamepad gamepad)
    {
        this.gamepad = gamepad;
    }

    public Vector2D rightJoystick()
    {
        return Vector2D.rectangular(gamepad.right_stick_x, -gamepad.right_stick_y).rotateBy(-90);
    }

    public Vector2D leftJoystick()
    {
        return Vector2D.rectangular(gamepad.left_stick_x, -gamepad.left_stick_y).rotateBy(-90);
    }

    public Vector2D dpad()
    {
        return Vector2D.rectangular((gamepad.dpad_left ? 1 : 0) + (gamepad.dpad_right ? -1 : 0), (gamepad.dpad_up ? 1 : 0) + (gamepad.dpad_down ? -1 : 0)).unit();
    }
}
