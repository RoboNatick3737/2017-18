package hankextensions.input;

public class HTButton
{
    // The gamepad buttons.
    public enum ButtonState {JUST_TAPPED, HELD, NOT_PRESSED}
    public ButtonState currentState = ButtonState.NOT_PRESSED;

    public void state(boolean pressed)
    {
        if (pressed)
        {
            switch (currentState)
            {
                case JUST_TAPPED:
                    currentState = ButtonState.HELD;
                    break;

                case NOT_PRESSED:
                    currentState = ButtonState.JUST_TAPPED;
                    break;
            }
        }
        else
        {
            currentState = ButtonState.NOT_PRESSED;
        }
    }
}
