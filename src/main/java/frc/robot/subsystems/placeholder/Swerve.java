package frc.robot.subsystems.placeholder;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.GeometryUtils;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Swerve extends SubsystemBase {
  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private final Translation2d[] m_swerveTranslation2d = {
      SwerveConfig.m_frontLeftTranslation,
      SwerveConfig.m_frontRightTranslation,
      SwerveConfig.m_backLeftTranslation,
      SwerveConfig.m_backRightTranslation
  };
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_swerveTranslation2d);
  // public Pigeon2 gyro;
  private AHRS m_gyro;

  private LoggedDashboardNumber PPtranslationP = new LoggedDashboardNumber("Pathplanner_ TranslationConstants P");
  private LoggedDashboardNumber PProtationP = new LoggedDashboardNumber("Pathplanner_ RotationConstants P");
  private LoggedDashboardNumber PPtranslationI = new LoggedDashboardNumber("Pathplanner_ TranslationConstants I");
  private LoggedDashboardNumber PProtationI = new LoggedDashboardNumber("Pathplanner_ RotationConstants I");
  private LoggedDashboardNumber PPtranslationD = new LoggedDashboardNumber("Pathplanner_ TranslationConstants D");
  private LoggedDashboardNumber PProtationD = new LoggedDashboardNumber("Pathplanner_ RotationConstants D");

  SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  public Swerve() {

    // gyro = new Pigeon2(Constants.Swerve.REV.pigeonID);
    // gyro.configFactoryDefault();
    m_gyro = new AHRS();

    mSwerveMods = new SwerveModule[] {
        new SwerveMod(0, SwerveConfig.REV.frontLeftModule.constants),
        new SwerveMod(1, SwerveConfig.REV.frontRightModule.constants),
        new SwerveMod(2, SwerveConfig.REV.backLeftModule.constants),
        new SwerveMod(3, SwerveConfig.REV.backRightModule.constants)
    };

    swerveOdometry = new SwerveDriveOdometry(SwerveConfig.swerveKinematics, getYaw(), getModulePositions());
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

          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    PProtationP.setDefault(2);
    PProtationI.setDefault(0);
    PProtationD.setDefault(0);
    PPtranslationP.setDefault(1.9);
    PPtranslationI.set(0);
    PPtranslationD.setDefault(0);

    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
        m_kinematics, m_gyro.getRotation2d(), getModulePositions(), getPose());
  }

  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME_S = 0.02;
    Pose2d futureRobotPose = new Pose2d(
        originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
        originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
        Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
    Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
    ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
        twistForPose.dx / LOOP_TIME_S,
        twistForPose.dy / LOOP_TIME_S,
        twistForPose.dtheta / LOOP_TIME_S);
    return updatedSpeeds;
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    ChassisSpeeds desiredChassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            Rotation2d.fromDegrees(-m_gyro.getAngle()))
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

    SwerveModuleState[] swerveModuleStates = SwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConfig.maxSpeed);
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
    }
  }

  @AutoLogOutput
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
    Logger.recordOutput("Pathplanner_ Pathplanner ChassisSpeeds", discreteSpeeds);
    Logger.recordOutput("Pathplanner_ Pathplanner targetStates", setPointState);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConfig.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
    }
  }

  @AutoLogOutput
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw()), getModulePositions(), pose);
  }

  @AutoLogOutput
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.getModuleNumber()] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.getModuleNumber()] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro(double deg) {
    if (SwerveConfig.invertGyro) {
      deg = -deg;
    }
    // gyro.setYaw(deg);
    // m_gyro.setAngleAdjustment(deg);
    m_gyro.reset();

    swerveOdometry.update(getYaw(), getModulePositions());
  }

  public void zeroGyro() {
    zeroGyro(0);
    // m_gyro.reset();
  }

  public Rotation2d getYaw() {
    return (SwerveConfig.invertGyro)
        ? Rotation2d.fromDegrees(360 - m_gyro.getYaw())
        : Rotation2d.fromDegrees(m_gyro.getYaw());
  }

  @Override
  public void periodic() {
    swerveDrivePoseEstimator.update(m_gyro.getRotation2d(), getModulePositions());
    swerveOdometry.update(m_gyro.getRotation2d(), getModulePositions());

    SmartDashboard.putNumber("yaw", Units.degreesToRadians(m_gyro.getYaw()));
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "REV Mod " + mod.getModuleNumber() + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "REV Mod " + mod.getModuleNumber() + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "REV Mod " + mod.getModuleNumber() + " Velocity", mod.getState().speedMetersPerSecond);
      // Logger.recordOutput("thingy" + mod.getModuleNumber(), mod.resetToAbsolute());
    }
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return the robots estimated Pose2d on the field based off
   *         SwerveDrivePoseEstimator, in meters
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

  /**
   * Returns the path the robot will follow.
   *
   * @param pathName - the name of the .path file
   * @return the path following command that will use to drive the robot
   */
  public Command followPathCommand(String pathName) {
    var path = PathPlannerPath.fromPathFile(pathName);

    m_gyro.reset();

    swerveOdometry.resetPosition(
        m_gyro.getRotation2d(), getModulePositions(), getPathplannerStartPosition(path));

    Logger.recordOutput(
        "Pathplanner_ Pathplanner starting position", path.getPreviewStartingHolonomicPose());

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
            AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            AutoConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to
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

          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }
}
