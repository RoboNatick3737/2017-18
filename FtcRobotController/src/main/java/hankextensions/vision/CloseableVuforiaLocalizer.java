package hankextensions.vision;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class CloseableVuforiaLocalizer extends VuforiaLocalizerImpl
{
    private boolean closed = false;

    public CloseableVuforiaLocalizer(Parameters parameters) {
        super(parameters);

        isCameraRunning = true;
    }

    @Override
    public void close() {
        if (!closed) super.close();
        closed = true;
    }
}