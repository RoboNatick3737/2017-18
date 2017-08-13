package org.firstinspires.ftc.teamcode.programs.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.programs.Core;
import org.firstinspires.ftc.teamcode.sdkextensions.threading.ComplexTask;
import org.firstinspires.ftc.teamcode.sdkextensions.threading.Flow;

@Autonomous(name="Ensure Threading Works", group="Experimentation")
public class EnsureThreadingOperational extends Core
{
    protected void START() throws InterruptedException
    {
        ComplexTask task1 = new ComplexTask("Task 1") {
            @Override
            protected void onDoTask() throws InterruptedException {
                int i = 0;
                while (true) {
                    processConsole.updateWith("Task 1 Update" + i + "!");
                    i++;
                    Flow.msPause(200);
                }
            }
        };

        ComplexTask task2 = new ComplexTask("Task 2") {
            @Override
            protected void onDoTask() throws InterruptedException {
                int i = 0;
                while (true) {
                    processConsole.updateWith("Task 1 Update" + i + "!");
                    i++;
                    Flow.msPause(500);
                }
            }
        };

        task1.run();
        task2.run();

        log.lines("Task 1 updating per 200 ms, Task 2 every 500");

        while (true)
        {
            Flow.msPause(2000);
        }
    }
}
