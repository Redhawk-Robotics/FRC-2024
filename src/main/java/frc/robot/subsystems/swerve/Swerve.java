package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.lib.math.GeometryUtils;
import org.littletonrobotics.junction.AutoLogOutput;

public class Swerve extends SubsystemBase {

  @AutoLogOutput
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  // public Pigeon2 gyro;
  public AHRS m_gyro;

  public Swerve() {

    // gyro = new Pigeon2(Constants.Swerve.REV.pigeonID);
    // gyro.configFactoryDefault();
    m_gyro = new AHRS();

    mSwerveMods = new SwerveModule[] {
        new SwerveMod(0, Constants.Swerve.REV.frontLeftModule.constants),
        new SwerveMod(1, Constants.Swerve.REV.frontRightModule.constants),
        new SwerveMod(2, Constants.Swerve.REV.backLeftModule.constants),
        new SwerveMod(3, Constants.Swerve.REV.backRightModule.constants)
    };

    swerveOdometry = new SwerveDriveOdometry(SwerveConfig.swerveKinematics, getYaw(), getModulePositions());
    zeroGyro();
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
            translation.getX(), translation.getY(), rotation, getYaw())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

    SwerveModuleState[] swerveModuleStates = SwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConfig.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    // System.out.println("setting module states: "+desiredStates[0]);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConfig.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
    }
  }

  public Pose2d getPose() {
    Pose2d p = swerveOdometry.getPoseMeters();
    return new Pose2d(-p.getX(), -p.getY(), p.getRotation());
  }

  public void resetOdometry(Pose2d pose) {

    swerveOdometry.resetPosition(new Rotation2d(), getModulePositions(), pose);
    zeroGyro(pose.getRotation().getDegrees());
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
    m_gyro.setAngleAdjustment(deg);
    swerveOdometry.update(getYaw(), getModulePositions());
  }

  public void zeroGyro() {
    zeroGyro(0);
  }

  public Rotation2d getYaw() {
    return (SwerveConfig.invertGyro)
        ? Rotation2d.fromDegrees(360 - m_gyro.getYaw())
        : Rotation2d.fromDegrees(m_gyro.getYaw());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("yaw", m_gyro.getYaw());
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "REV Mod " + mod.getModuleNumber() + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "REV Mod " + mod.getModuleNumber() + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "REV Mod " + mod.getModuleNumber() + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
