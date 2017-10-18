package hankextensions.vision;


import org.opencv.core.Scalar;

/**
 * Storage class for the position and color of the beacon
 * This file was made by the electronVolts, FTC team 7393
 * Date Created: 8/17/16.
 */
public class BeaconColorResult{
  
  public enum BeaconColor{
    RED     (ImageUtil.RED),
    GREEN   (ImageUtil.GREEN),
    BLUE    (ImageUtil.BLUE),
    UNKNOWN (ImageUtil.BLACK);
    
    public final Scalar color;
    
    BeaconColor(Scalar scalar){
      this.color = scalar;
    }
  }
  private final BeaconColor leftColor, rightColor;
  
  public BeaconColorResult() {
    this.leftColor = BeaconColor.UNKNOWN;
    this.rightColor = BeaconColor.UNKNOWN;
  }
  
  public BeaconColorResult(BeaconColor leftColor, BeaconColor rightColor) {
    this.leftColor = leftColor;
    this.rightColor = rightColor;
  }
  
  public String toString(){
    return leftColor + ", " + rightColor;
  }
  
  public BeaconColor getLeftColor() {
    return leftColor;
  }
  
  public BeaconColor getRightColor() {
    return rightColor;
  }
}
