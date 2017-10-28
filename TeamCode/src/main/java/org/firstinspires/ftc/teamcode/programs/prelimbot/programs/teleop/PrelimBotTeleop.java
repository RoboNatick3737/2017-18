package org.firstinspires.ftc.teamcode.programs.prelimbot.programs.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.programs.prelimbot.HardwareBase;

import hankextensions.logging.ProcessConsole;
import hankextensions.threading.Flow;

@TeleOp(name = "Prelim Bot Teleop", group = Constants.PRELIM_BOT_OPMODES)
public class PrelimBotTeleop extends HardwareBase
{
    private long lastAPress = 0;
    private boolean clampsClosed = false;

    private long lastBPress = 0;
    private boolean swingDown = false;

    @Override
    public void START() throws InterruptedException
    {
        ProcessConsole processConsole = log.newProcessConsole("Teleop");

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
            if (gamepad1.x && !clampsClosed && System.currentTimeMillis() - lastAPress > 400)
            {
                topLeftGrabber.setPosition(.2);
                bottomLeftGrabber.setPosition(.2);
                topRightGrabber.setPosition(.7);
                bottomRightGrabber.setPosition(.7);

                clampsClosed = true;
                lastAPress = System.currentTimeMillis();
            } else if(gamepad1.x && clampsClosed && System.currentTimeMillis() - lastAPress > 400)
            {
                topLeftGrabber.setPosition(.7);
                bottomLeftGrabber.setPosition(.7);
                topRightGrabber.setPosition(.2);
                bottomRightGrabber.setPosition(.2);

                clampsClosed = false;
                lastAPress = System.currentTimeMillis();
            }

            if (gamepad1.b && !swingDown && System.currentTimeMillis() - lastBPress > 400)
            {
                swingServo.setPosition(0);

                swingDown = true;
                lastBPress = System.currentTimeMillis();
            } else if(gamepad1.b && swingDown && System.currentTimeMillis() - lastBPress > 400)
            {
                swingServo.setPosition(1);

                swingDown = false;
                lastBPress = System.currentTimeMillis();
            }
            /* </Grippers> */

            /* <Slide> */
            if (gamepad1.y)
            {
                miniLift.setPower(0.6);
            } else if(gamepad1.a)
            {
                miniLift.setPower(-0.3);
            } else if(!gamepad1.y && !gamepad1.a)
            {
                miniLift.setPower(0);
            }

            if (gamepad1.dpad_up)
            {
                primaryLift.setPower(0.5);
            } else if(gamepad1.dpad_down)
            {
                primaryLift.setPower(-0.5);
            } else if(!gamepad1.dpad_up && !gamepad1.dpad_down)
            {
                primaryLift.setPower(0);
            }

            /* </Slide> */

            Flow.yield();
        }
    }
}
