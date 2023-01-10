package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Limelight camera, running PhotonVision.
 * 
 * 
 */

public class Photonvision extends SubsystemBase {
  private final PhotonCamera limelight; //FIXME


  
  

  /** Creates a new LimelightS. */
  public Photonvision() {
    limelight = new PhotonCamera("gloworm"); //FIXME    
    limelight.setPipelineIndex(1);
    limelight.setDriverMode(false);
  }



  /**
   * Adds the latest PhotonVision result to the filters.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
 

    

  }
}