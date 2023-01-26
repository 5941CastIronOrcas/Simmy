
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;


public class Vision extends SubsystemBase {
  
  PhotonCamera camera = new PhotonCamera("photonvision");

  public Vision() {}
  
  public PhotonTrackedTarget obtainTargets() {
    var result = camera.getLatestResult();

    if  (result.hasTargets()) {
      //Sends back the most clear target and its data
      return result.getBestTarget();
    } else {
      return null;
    }
  }

  @Override
  public void periodic() {
    
  }

}