package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO {
  @AutoLog
  public static class VisionInputs {
    public PhotonPipelineResult result;
    public boolean hasTargets;
    public int numOfTargets;
    public Pose3d[] targetLocation;
    public PhotonTrackedTarget currentBestTarget;
    public Transform3d cameraToTargetPose;
    public Transform3d alternateCameraToTarget;
  }

  // Updates loggable inputs
  public default void updateInputs(VisionInputs inputs) {}

  // Updates information
  public default void updateVision() {}

  // Contains all information about currently detected targets
  public PhotonPipelineResult getLatestResult();

  // method to inform the user as to whether the result contains any targets
  public boolean hasTargets();

  // Get a list of tracked targets
  public Pose3d[] getTargetLocations();

  // Get the best tracked target
  public PhotonTrackedTarget getBestTarget();

  public PhotonCamera getCam();
}
