package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.utils.AprilTagLocation;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class VisionIOPhotonvision implements VisionIO {
  // private final CANSparkMax spark;
  private final PhotonCamera vision;
  private boolean hasTargets;
  private PhotonPipelineResult result;
  private PhotonTrackedTarget target;

  public VisionIOPhotonvision() {
    // this.vision = new PhotonCamera("Integrated_Camera");
    this.vision = new PhotonCamera("Camera_Module_v1");
    // this.vision = new PhotonCamera("USB2.0_Camera");
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    inputs.result = this.result;
    inputs.hasTargets = hasTargets();
    inputs.numOfTargets = result.getTargets().size();
    inputs.targetLocation = getTargetLocations();
    inputs.currentBestTarget = getBestTarget();
    inputs.cameraToTargetPose = target.getBestCameraToTarget();
    inputs.alternateCameraToTarget = target.getAlternateCameraToTarget();
  }

  @Override
  public PhotonCamera getCam() {
    return vision;
  }

  @Override
  public PhotonPipelineResult getLatestResult() {
    return this.result;
  }

  private void setLatestResult() {
    try {
      result = vision.getLatestResult();
    } catch (NullPointerException e) {
      result =
          new PhotonPipelineResult(
              0,
              new ArrayList<PhotonTrackedTarget>(0),
              new MultiTargetPNPResult(new PNPResult(), new ArrayList<>(0)));
    }
  }

  @Override
  public boolean hasTargets() {
    // System.out.println(hasTargets ? "Have target" : "Dont have target");
    return hasTargets;
  }

  private void setHasTargets() {
    try {
      hasTargets = result.hasTargets();
    } catch (NullPointerException e) {
      hasTargets = false;
    }
  }

  @Override
  public Pose3d[] getTargetLocations() {
    return setListOfTargets();
  }

  private Pose3d[] setListOfTargets() {
    try {
      var targets = result.getTargets();
      Pose3d[] pose3ds = new Pose3d[targets.size()];

      for (int i = 1; i < targets.size() + 1; i++) {
        var tag = AprilTagLocation.getPositionById(i);
        pose3ds[i - 1] =
            new Pose3d(tag.xPos, tag.yPos, tag.zPos, new Rotation3d(0, 0, tag.rotation));
      }
      return pose3ds;
    } catch (NullPointerException e) {
      return new Pose3d[0];
    }
  }

  @Override
  public PhotonTrackedTarget getBestTarget() {
    return target;
  }

  private void setBestTarget() {
    if (hasTargets() != false) {
      try {
        target = result.getBestTarget();
      } catch (NullPointerException e) {
        target =
            new PhotonTrackedTarget(
                0,
                0,
                0,
                0,
                0,
                new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)),
                new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)),
                0,
                new ArrayList<TargetCorner>(0),
                new ArrayList<TargetCorner>(0));
      }
    } else {
      target =
          new PhotonTrackedTarget(
              0,
              0,
              0,
              0,
              0,
              new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)),
              new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)),
              0,
              new ArrayList<TargetCorner>(0),
              new ArrayList<TargetCorner>(0));
    }
  }

  @Override
  public void updateVision() {
    setLatestResult();
    setHasTargets();
    setListOfTargets();
    setBestTarget();
  }

  // @Override
  // public void runMotor() {
  // if (hasTargets()) {
  // spark.set(.5);
  // } else {
  // spark.set(0);
  // }
  // }
}
