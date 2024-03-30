// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Settings.AutoConstants;
import frc.constants.Settings.SwerveConfig;
import frc.lib.math.GeometryUtils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Swerve extends SubsystemBase {
  private GyroIO mGyroIO;
  private GyroIOInputsAutoLogged mGyroIOInputs;

  private Module[] mSwerveMods;
  private SwerveDriveOdometry swerveOdometry;

  private final Translation2d[] m_swerveTranslation2d = SwerveConfig.m_swerveTranslation2d;
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_swerveTranslation2d);
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private Pose2d pathPlannerPose;
  private Field2d field;

  private LoggedDashboardNumber PPtranslationP =
      new LoggedDashboardNumber("Pathplanner_ TranslationConstants P");
  private LoggedDashboardNumber PProtationP =
      new LoggedDashboardNumber("Pathplanner_ RotationConstants P");
  private LoggedDashboardNumber PPtranslationI =
      new LoggedDashboardNumber("Pathplanner_ TranslationConstants I");
  private LoggedDashboardNumber PProtationI =
      new LoggedDashboardNumber("Pathplanner_ RotationConstants I");
  private LoggedDashboardNumber PPtranslationD =
      new LoggedDashboardNumber("Pathplanner_ TranslationConstants D");
  private LoggedDashboardNumber PProtationD =
      new LoggedDashboardNumber("Pathplanner_ RotationConstants D");

  /** Creates a new Swerve. */
  public Swerve(GyroIO gyroIO, ModuleIO FLIO, ModuleIO FRIO, ModuleIO RLIO, ModuleIO RRIO) {
    this.mGyroIO = gyroIO;
    this.mGyroIOInputs = new GyroIOInputsAutoLogged();

    mSwerveMods =
        new Module[] {
          new Module(FLIO, 0), new Module(FRIO, 1), new Module(RLIO, 2), new Module(RRIO, 3)
        };

    swerveOdometry =
        new SwerveDriveOdometry(SwerveConfig.swerveKinematics, getYaw(), getModulePositions());
    zeroGyro();

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        AutoConstants.kPathFollowerConfig,
        () -> {
          // var alliance = DriverStation.getAlliance();
          // if (alliance.isPresent()) {
          // return alliance.get() == DriverStation.Alliance.Red;
          // }
          // return false;

          return true;
        },
        this // Reference to this subsystem to set requirements
        );

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    PProtationP.setDefault(2);
    PProtationI.setDefault(0);
    PProtationD.setDefault(0);
    PPtranslationP.setDefault(1.9);
    PPtranslationI.set(0);
    PPtranslationD.setDefault(0);

    swerveDrivePoseEstimator =
        new SwerveDrivePoseEstimator(
            m_kinematics, gyroIO.getRotation2d(), getModulePositions(), getPose());

    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          // Do whatever you want with the pose here
          pathPlannerPose = pose;
          field.getObject("target pose").setPose(pose);
        });
  }

  // ~ periodic
  @Override
  public void periodic() {
    for (Module mod : mSwerveMods) {
      mod.periodic();
    }

    mGyroIO.updateInputs(mGyroIOInputs);
    Logger.processInputs("Gyro", mGyroIOInputs);

    swerveDrivePoseEstimator.update(mGyroIO.getRotation2d(), getModulePositions());
    swerveOdometry.update(mGyroIO.getRotation2d(), getModulePositions());

    Logger.recordOutput("Swerve/RobotPose", getPose());
  }

  // ~ IMPORTANT DRIVE FUNCTIONS
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    ChassisSpeeds desiredChassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                Rotation2d.fromDegrees(-mGyroIOInputs.angle))
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

    SwerveModuleState[] swerveModuleStates =
        SwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
    setModuleStates(swerveModuleStates);
    Logger.recordOutput("tewts states", swerveModuleStates);
    Logger.recordOutput("speeds", desiredChassisSpeeds);
  }

  // ! START OF METHODS FOR PATHPLANNER

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return SwerveConfig.swerveKinematics.toChassisSpeeds(
        mSwerveMods[0].getState(),
        mSwerveMods[1].getState(),
        mSwerveMods[2].getState(),
        mSwerveMods[3].getState());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, .02);

    SwerveModuleState[] setPointState = m_kinematics.toSwerveModuleStates(discreteSpeeds);
    setModuleStates(setPointState);

    Logger.recordOutput("Pathplanner/Pathplanner ChassisSpeeds", speeds);
    Logger.recordOutput("Pathplanner/Pathplanner discrete ChassisSpeeds", discreteSpeeds);
    Logger.recordOutput("Pathplanner/Pathplanner setPointState", setPointState);
  }

  // ! END OF METHODS FOR PATHPLANNER

  /**
   * Returns the position of the robot on the field.
   *
   * @return the robots estimated Pose2d on the field based off SwerveDrivePoseEstimator, in meters
   */
  public Pose2d getRobotPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * Returns the pathplanners starting position.
   *
   * @param path - the path that will be followed
   * @return the Pose2d of the starting position of said path, in meters
   */
  public Pose2d getPathplannerStartPosition(PathPlannerPath path) {
    return path.getPreviewStartingHolonomicPose();
  }

  public Pose2d getPathPlannerPose() {
    return pathPlannerPose;
  }

  // ! Pathplanner follow path
  /**
   * Returns the path the robot will follow.
   *
   * @param pathName - the name of the .path file
   * @return the path following command that will use to drive the robot
   */
  public Command followPathCommand(String pathName) {
    var path = PathPlannerPath.fromPathFile(pathName);

    mGyroIO.reset();

    swerveOdometry.resetPosition(
        mGyroIO.getRotation2d(), getModulePositions(), getPathplannerStartPosition(path));

    Logger.recordOutput(
        "Pathplanner/Pathplanner starting position", path.getPreviewStartingHolonomicPose());

    return new FollowPathHolonomic(
        path,
        this::getRobotPose, // Robot pose supplier
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig(
            new PIDConstants(
                PPtranslationP.get(),
                PPtranslationI.get(),
                PPtranslationD.get()), // Translation PID
            // constants, around
            // 2.25
            new PIDConstants(
                PProtationP.get(), PProtationI.get(), PProtationD.get()), // Rotation PID constants
            SwerveConfig.maxSpeed, // Max module speed, in m/s
            AutoConstants
                .kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to
            // furthest module.
            new ReplanningConfig(false, false)),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          // var alliance = DriverStation.getAlliance();
          // if (alliance.isPresent()) {
          // return alliance.get() == DriverStation.Alliance.Red;
          // }
          // return false;

          return true;
        },
        this // Reference to this subsystem to set requirements
        );
  }

  /*
   *
   * Class methods
   *
   */

  public void zeroGyro(double deg) {
    if (SwerveConfig.invertGyro) {
      deg = -deg;
    }
    // gyro.setYaw(deg);
    // m_gyro.setAngleAdjustment(deg);
    mGyroIO.reset();

    swerveOdometry.update(getYaw(), getModulePositions());
  }

  public void zeroGyro() {
    zeroGyro(0);
    // m_gyro.reset();
  }

  public Rotation2d getYaw() {
    return (SwerveConfig.invertGyro)
        ? Rotation2d.fromDegrees(360 - mGyroIOInputs.yaw)
        : Rotation2d.fromDegrees(mGyroIOInputs.yaw);
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (Module mod : mSwerveMods) {
      positions[mod.getModuleNumber()] = mod.getPosition();
    }
    return positions;
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConfig.maxSpeed);

    for (Module mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
    }
  }

  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME_S = 0.02;
    Pose2d futureRobotPose =
        new Pose2d(
            originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
            originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
    Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
    ChassisSpeeds updatedSpeeds =
        new ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S);
    return updatedSpeeds;
  }

  @AutoLogOutput
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (Module mod : mSwerveMods) {
      states[mod.getModuleNumber()] = mod.getState();
    }
    return states;
  }
}
