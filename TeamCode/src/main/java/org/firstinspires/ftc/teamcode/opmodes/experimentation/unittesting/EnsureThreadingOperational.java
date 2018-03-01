package org.firstinspires.ftc.teamcode.opmodes.experimentation.unittesting;

import dude.makiah.androidlib.threading.ParallelTask;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dude.makiah.androidlib.threading.TimeMeasure;
import hankutanku.EnhancedOpMode;

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
                    flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, .2));
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
                    flow.pause(new TimeMeasure(TimeMeasure.Units.SECONDS, .5));
                }
            }
        };

        task1.run();
        task2.run();

        log.lines("Task 1 updating per 200 ms, Task 2 every 500");

        while (true)
            flow.yield();
    }
}
