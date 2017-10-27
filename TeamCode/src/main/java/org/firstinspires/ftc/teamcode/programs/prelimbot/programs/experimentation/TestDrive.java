package org.firstinspires.ftc.teamcode.programs.prelimbot.programs.experimentation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.programs.prelimbot.HardwareBase;

/**
 * Created by JordanArnold on 10/18/17.
 *
 * Time to test drive
 */
@Autonomous(name="Test Drive", group="Experimentation")
public class TestDrive extends HardwareBase {


  public void START() throws InterruptedException
  {

     final double DRIVE_POWER = 1.0;

      //Begin movement
      driveForward(DRIVE_POWER, 4000);
      turnLeft(DRIVE_POWER, 500);
      driveForward(DRIVE_POWER, 2000);
      turnRight(DRIVE_POWER, 1000);
      driveBackward(DRIVE_POWER, 2000);
      pauseDriving(4000);


  }

  public void driveForward(double power, long time) throws InterruptedException
  {
      left.setPower(power);
      right.setPower(power);
      middle.setPower(0);
      Thread.sleep(time);
  }

  public void driveBackward(double power, long time) throws InterruptedException
  {
      driveForward(-power, time);
  }

  public void turnLeft(double power, long time) throws InterruptedException
  {
      left.setPower(0);
      right.setPower(0);
      middle.setPower(power);
      Thread.sleep(time);
  }

  public void turnRight(double power, long time) throws InterruptedException
  {
      turnLeft(-power, time);
  }

  public void pauseDriving(long time) throws InterruptedException
  {
      driveForward(0, time);
  }



}
