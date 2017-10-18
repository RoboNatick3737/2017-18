package org.firstinspires.ftc.teamcode.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankextensions.Core;
import hankextensions.threading.ComplexTask;
import hankextensions.threading.Flow;

@Autonomous(name="Test Threading", group="Experimentation")
public class EnsureThreadingOperational extends Core
{
    protected void START() throws InterruptedException
    {
        ComplexTask task1 = new ComplexTask("Task 1") {
            @Override
            protected void onDoTask() throws InterruptedException {
                int i = 0;
                while (true) {
                    processConsole.write("Task 1 Update" + i + "!");
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
                    processConsole.write("Task 1 Update" + i + "!");
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
