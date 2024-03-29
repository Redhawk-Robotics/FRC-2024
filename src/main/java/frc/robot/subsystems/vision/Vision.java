// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOInputsAutoLogged;
import frc.robot.utils.AprilTagLocation;
import java.util.NoSuchElementException;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * At first only God and we knew how this code worked, now its just God. Good luck trying to figure
 * it out hours wasted here = 15
 */
public class Vision extends SubsystemBase {
  private final VisionIO visionIO;
  private final GyroIO gyroIO;
  private final VisionInputsAutoLogged visionInputs = new VisionInputsAutoLogged();

  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private PhotonTrackedTarget target;
  private Transform2d cameraToRobot2D;
  private Transform3d cameraToRobot3D;
  private AprilTagFieldLayout layout;
  private Pose2d robotPose;
  private Pose3d robotPose3D;
  private double range;
  private PhotonPoseEstimator photonPoseEstimator;
  private PhotonCamera cam;
  private double distanceToTarget;
  private Transform3d pose;
  private static Pose2d visionOffset;

  /** Creates a new Photonvision. */
  public Vision(VisionIO io, GyroIO gyroIO) {
    System.out.println("[INIT] VISION INIT");
    this.visionIO = io;
    this.gyroIO = gyroIO;
    gyroIO.reset();
    this.robotPose = new Pose2d();
    this.robotPose3D = new Pose3d();
    this.visionOffset = new Pose2d();
    this.range = 0;
    this.cam = visionIO.getCam();
    this.distanceToTarget = 0;
    this.layout = new AprilTagFieldLayout(AprilTagLocation.getTagIdList(), 16.45, 8.21);
    this.cameraToRobot2D =
        new Transform2d(
            new Translation2d(Units.inchesToMeters(-14), Units.inchesToMeters(5.5)),
            new Rotation2d(0));
    // this.cameraToRobot2D = new Transform2d(new Translation2d(0, 0), new
    // Rotation2d(0));
    this.cameraToRobot3D =
        new Transform3d(
            new Translation3d(Units.inchesToMeters(0), 0, Units.inchesToMeters(0)),
            new Rotation3d());
    // this.photonPoseEstimator = new PhotonPoseEstimator(layout, null,
    // cameraToRobot3D);
    // this.cameraToRobot3D =
    // new Transform3d(
    // new Pose3d(new Pose2d(0, 0, new Rotation2d(0))),
    // new Pose3d(new Pose2d(Units.inchesToMeters(0), 0, new Rotation2d(0))));
    this.visionIO.updateVision();
    this.gyroIO.updateInputs(gyroInputs);
    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            layout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, cameraToRobot3D);
  }

  @Override
  public void periodic() {
    visionIO.updateVision();
    gyroIO.updateInputs(gyroInputs);
    // This method will be called once per scheduler run
    // visionIO.updateInputs(visionInputs);
    // Logger.processInputs("Photonvision", visionInputs);
    Logger.processInputs("Gyro", gyroInputs);

    Logger.recordOutput("Vision_ Nav-X Yaw", gyroInputs.yaw);
    Logger.recordOutput("Vision_ EstimateFieldToRobot - getRobotPoseTraditional", robotPose);
    Logger.recordOutput("Vision_ Robotpose3d - estimateFieldToRobotAprilTag", robotPose3D);
    Logger.recordOutput("Vision_ Distance to target", Units.metersToInches(range));
    Logger.recordOutput(
        "Vision_ Distance to Pose - from robotPose", distanceToTarget); // maybe more accurate
    Logger.recordOutput(
        "Vision_ Robot to camera transform", photonPoseEstimator.getRobotToCameraTransform());
    Logger.recordOutput(
        "Vision_ Estimated global Pose", getEstimatedGlobalPose(robotPose).estimatedPose);
    Logger.recordOutput("Vision_Test pose value", getBestCamToTag());

    target = visionIO.getBestTarget();
    updateRobotPoseTraditional();
    updateRobotPose();
    updateRangeToTarget();
    updatePhotonPoseEstimator();
  }

  public void updateRobotPoseTraditional() {
    if (visionIO.hasTargets()) {
      double gyroAngleDegrees =
          (gyroInputs.yaw + 90)
              - (2 * Units.radiansToDegrees(getAprilTagLocationMeters().rotation)) % 360;

      Logger.recordOutput("Vision_ Created Gyro Angle", gyroAngleDegrees);
      // Now use the adjusted gyro angle in the estimateFieldToRobot method
      robotPose =
          PhotonUtils.estimateFieldToRobot(
              Units.inchesToMeters(26.25), // 46 when tag 16
              getAprilTagLocationMeters().zPos,
              Units.degreesToRadians(22),
              Units.degreesToRadians(target.getPitch()),
              Rotation2d.fromDegrees(-target.getYaw()),
              Rotation2d.fromDegrees(gyroAngleDegrees),
              getAprilTag2D(),
              cameraToRobot2D);
      visionOffset = robotPose;
    }
  }

  public static Pose2d getOffset() {
    return visionOffset;
  }

  public void updateRobotPose() {
    if (visionIO.hasTargets()) {
      robotPose3D =
          PhotonUtils.estimateFieldToRobotAprilTag(
              target.getBestCameraToTarget(),
              new Pose3d(
                  getAprilTagLocationMeters().xPos,
                  getAprilTagLocationMeters().yPos,
                  0,
                  new Rotation3d(0, 0, getAprilTagLocationMeters().rotation)),
              cameraToRobot3D);
    }
  }

  public AprilTagLocation getAprilTagLocationMeters() {
    if (visionIO.hasTargets() && target != null) {
      return AprilTagLocation.getPositionById(target.getFiducialId());
    }
    return AprilTagLocation.getPositionById(17);
  }

  public Pose2d getAprilTag2D() {
    if (getAprilTagLocationMeters() != null) {
      return new Pose2d(
          getAprilTagLocationMeters().xPos,
          getAprilTagLocationMeters().yPos,
          Rotation2d.fromRadians(getAprilTagLocationMeters().rotation));
    }
    return new Pose2d();
  }

  public void updateRangeToTarget() {
    if (target != null && AprilTagLocation.getPositionById(target.getFiducialId()) != null) {
      range =
          PhotonUtils.calculateDistanceToTargetMeters(
              Units.inchesToMeters(26.25),
              getAprilTagLocationMeters().zPos,
              Units.degreesToRadians(22),
              Units.degreesToRadians(target.getPitch()));

      distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, getAprilTag2D());
    }
  }

  public void updatePhotonPoseEstimator() {
    if (visionIO.hasTargets() && target != null && cam != null) {
      photonPoseEstimator =
          new PhotonPoseEstimator(
              layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, cameraToRobot3D);
    }
  }

  public EstimatedRobotPose getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (cam != null && target != null) {
      photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      try {
        pose = target.getBestCameraToTarget();
        return photonPoseEstimator.update().get();
      } catch (NoSuchElementException e) {
        return new EstimatedRobotPose(robotPose3D, range, null, null);
      }
    }
    return new EstimatedRobotPose(robotPose3D, range, null, null);
  }

  public Pose2d test() {
    if (target != null && robotPose3D != null) {
      // System.out.println("thing " + PhotonUtils.getYawToPose(robotPose,
      // getAprilTag2D()));
      return new Pose2d(
          robotPose3D.getX(),
          robotPose3D.getY(),
          new Rotation2d(robotPose3D.getRotation().getX(), robotPose3D.getRotation().getY()));
    }
    return new Pose2d();
  }

  public AprilTagLocation getResult() {
    try {
      Logger.recordOutput(
          "try this",
          AprilTagLocation.getPositionById(
              visionIO.getLatestResult().getBestTarget().getFiducialId()));
      return AprilTagLocation.getPositionById(
          visionIO.getLatestResult().getBestTarget().getFiducialId());
    } catch (NullPointerException e) {
      return AprilTagLocation.kTagNULL;
    }
  }

  public boolean getHasTargets() {
    return visionIO.hasTargets();
  }

  public double getDistance() {
    return distanceToTarget;
  }

  public Transform3d getBestCamToTag() {
    if (target == null) {
      System.out.println("got called");
      return new Transform3d();
    }
    return target.getBestCameraToTarget();
  }

  // public Supplier<Transform3d> getBestCamera() {
  // return target.getBestCameraToTarget();
  // }
}
