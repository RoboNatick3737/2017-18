package hankextensions.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by vandejd1 on 9/7/16.
 * FTC Team EV 7393
 */
public class BeaconDetectionOp extends LinearOpMode
{

  @Override
  public void runOpMode() throws InterruptedException {
    FrameGrabber frameGrabber = null;//FtcRobotControllerActivity.cameraController.openCVCam.frameGrabber; //Get the frameGrabber

    frameGrabber.grabSingleFrame(); //Tell it to grab a frame
    while (!frameGrabber.isResultReady()) { //Wait for the result
      Thread.sleep(5); //sleep for 5 milliseconds
    }

    //Get the result
    ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
    BeaconColorResult result = (BeaconColorResult) imageProcessorResult.getResult();

    BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
    BeaconColorResult.BeaconColor rightColor = result.getRightColor();

    telemetry.addData("Result", result); //Display it on telemetry
    telemetry.update();
    //wait before quitting (quitting clears telemetry)
    Thread.sleep(1000);
  }
}
