package frc.robot.subsystems.placeholder;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import frc.robot.Constants.SwerveConfig;
import org.littletonrobotics.junction.AutoLogOutput;

/** a Swerve Modules using REV Robotics motor controllers and CTRE CANcoder absolute encoders. */
public class SwerveMod implements SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;

  private CANSparkMax mAngleMotor;
  private CANSparkMax mDriveMotor;

  private CANcoder angleEncoder;
  private RelativeEncoder relAngleEncoder;
  private RelativeEncoder relDriveEncoder;

  public SwerveMod(int moduleNumber, RevSwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /* Angle Motor Config */
    mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    configAngleMotor();

    /* Drive MotorModuleIO Config */
    mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    configDriveMotor();

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    configEncoders();

    // lastAngle = getState().angle;
  }

  private void configEncoders() {
    // absolute encoder

    // angleEncoder.configFactoryDefault();
    // angleEncoder.configAllSettings(new SwerveConfig().canCoderConfig);

    relDriveEncoder = mDriveMotor.getEncoder();
    relDriveEncoder.setPosition(0);

    relDriveEncoder.setPositionConversionFactor(SwerveConfig.driveRevToMeters);
    relDriveEncoder.setVelocityConversionFactor(SwerveConfig.driveRpmToMetersPerSecond);

    relAngleEncoder = mAngleMotor.getEncoder();
    relAngleEncoder.setPositionConversionFactor(SwerveConfig.DegreesPerTurnRotation);
    // in degrees/sec
    relAngleEncoder.setVelocityConversionFactor(SwerveConfig.DegreesPerTurnRotation / 60);

    resetToAbsolute();
    mDriveMotor.burnFlash();
    mAngleMotor.burnFlash();
  }

  private void configAngleMotor() {
    mAngleMotor.restoreFactoryDefaults();
    SparkPIDController controller = mAngleMotor.getPIDController();
    controller.setP(SwerveConfig.angleKP, 0);
    controller.setI(SwerveConfig.angleKI, 0);
    controller.setD(SwerveConfig.angleKD, 0);
    controller.setFF(SwerveConfig.angleKF, 0);
    controller.setOutputRange(-SwerveConfig.anglePower, SwerveConfig.anglePower);
    mAngleMotor.setSmartCurrentLimit(SwerveConfig.angleContinuousCurrentLimit);

    mAngleMotor.setInverted(SwerveConfig.angleMotorInvert);
    mAngleMotor.setIdleMode(SwerveConfig.angleIdleMode);
  }

  private void configDriveMotor() {
    mDriveMotor.restoreFactoryDefaults();
    SparkPIDController controller = mDriveMotor.getPIDController();
    controller.setP(SwerveConfig.driveKP, 0);
    controller.setI(SwerveConfig.driveKI, 0);
    controller.setD(SwerveConfig.driveKD, 0);
    controller.setFF(SwerveConfig.driveKF, 0);
    controller.setOutputRange(-SwerveConfig.drivePower, SwerveConfig.drivePower);
    mDriveMotor.setSmartCurrentLimit(SwerveConfig.driveContinuousCurrentLimit);
    mDriveMotor.setInverted(SwerveConfig.driveMotorInvert);
    mDriveMotor.setIdleMode(SwerveConfig.driveIdleMode);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

    // CTREModuleState functions for any motor type.
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);

    if (mDriveMotor.getFault(FaultID.kSensorFault)) {
      DriverStation.reportWarning(
          "Sensor Fault on Drive Motor ID:" + mDriveMotor.getDeviceId(), false);
    }

    if (mAngleMotor.getFault(FaultID.kSensorFault)) {
      DriverStation.reportWarning(
          "Sensor Fault on Angle Motor ID:" + mAngleMotor.getDeviceId(), false);
    }
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / SwerveConfig.maxSpeed;
      mDriveMotor.set(percentOutput);
      return;
    }

    double velocity = desiredState.speedMetersPerSecond;

    SparkPIDController controller = mDriveMotor.getPIDController();
    controller.setReference(velocity, ControlType.kVelocity, 0);
  }

  private void setAngle(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConfig.maxSpeed * 0.01)) {
      mAngleMotor.stopMotor();
      return;
    }
    Rotation2d angle = desiredState.angle;
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.

    SparkPIDController controller = mAngleMotor.getPIDController();

    double degReference = angle.getDegrees();

    controller.setReference(degReference, ControlType.kPosition, 0);
  }

  @AutoLogOutput(key = "mod{moduleNumber}")
  private double angle = 0;

  private Rotation2d getAngle() {
    angle = getCanCoder().getDegrees();
    return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue());
    // return getAngle();
  }

  public int getModuleNumber() {
    return moduleNumber;
  }

  public void setModuleNumber(int moduleNumber) {
    this.moduleNumber = moduleNumber;
  }

  @AutoLogOutput(key = "thing{moduleNumber}")
  double absolutePosition;

  @AutoLogOutput(key = "thing{moduleNumber} part1")
  double part1;

  @AutoLogOutput(key = "thing{moduleNumber} offset")
  double offset;

  private void resetToAbsolute() {
    absolutePosition = (360 * getCanCoder().getDegrees()) - (angleOffset.getDegrees() + 180);
    part1 = getCanCoder().getDegrees();
    offset = angleOffset.getDegrees();
    relAngleEncoder.setPosition(absolutePosition);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(relDriveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(relDriveEncoder.getPosition(), getAngle());
  }
}
