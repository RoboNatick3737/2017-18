package org.firstinspires.ftc.teamcode.programs.prelimbot.programs.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.programs.prelimbot.HardwareBase;

import hankextensions.threading.Flow;

@TeleOp(name = "Vroom VROOM", group = "Default")
public class DriveBot extends HardwareBase
{
    private boolean clampsClosed = false;

    @Override
    public void START() throws InterruptedException
    {
        while (true) // Will exit because of Flow.yield() if stop requested, don't worry.
        {

            /* <Movement> */
            if (!(gamepad1.left_bumper || gamepad1.right_bumper)) {
                left.setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
                right.setPower(Range.clip(gamepad1.right_stick_y, -1, 1));
            } else {
                if (gamepad1.left_bumper) {
                    left.setPower(0.25);
                    right.setPower(0.25);
                } else {
                    left.setPower(-0.25);
                    right.setPower(-0.25);
                }
            }

            middle.setPower(Range.clip(1.0 * gamepad1.right_trigger + -1.0 * gamepad1.left_trigger, -1, 1));
            /* </Movement> */

            /* <Grippers> */
            if (gamepad1.a && !clampsClosed)
            {
                servo1.setPosition(1);
                servo2.setPosition(-1);
                clampsClosed = true;
            } else if(gamepad1.a && clampsClosed)
            {
                servo1.setPosition(-1);
                servo2.setPosition(1);
                clampsClosed = false;
            }
            /* </Grippers> */

            /* <Slide> */
            if (gamepad1.x)
            {
                liftylift1.setPower(0.1);
            } else if(gamepad1.b)
            {
                liftylift1.setPower(-0.1);
            } else if(!gamepad1.x && !gamepad1.b)
            {
                liftylift1.setPower(0);
            }

            /* </Slide> */

            Flow.yield();
        }
    }
}
