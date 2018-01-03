package org.firstinspires.ftc.teamcode.experimentation;

import com.makiah.makiahsandroidlib.threading.ParallelTask;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import hankextensions.EnhancedOpMode;

@Autonomous(name="Test Threading", group="Experimentation")
public class EnsureThreadingOperational extends EnhancedOpMode
{
    @Override
    protected void onRun() throws InterruptedException
    {
        ParallelTask task1 = new ParallelTask(this, "Task 1", log) {
            @Override
            protected void onDoTask() throws InterruptedException {
                int i = 0;
                while (true) {
                    logLinesToProcessConsole("Task 1 Update" + i + "!");
                    i++;
                    flow.msPause(200);
                }
            }
        };

        ParallelTask task2 = new ParallelTask(this, "Task 2", log) {
            @Override
            protected void onDoTask() throws InterruptedException {
                int i = 0;
                while (true) {
                    logLinesToProcessConsole("Task 2 Update" + i + "!");
                    i++;
                    flow.msPause(500);
                }
            }
        };

        task1.run();
        task2.run();

        log.lines("Task 1 updating per 200 ms, Task 2 every 500");

        while (true)
            flow.msPause(2000);
    }
}
