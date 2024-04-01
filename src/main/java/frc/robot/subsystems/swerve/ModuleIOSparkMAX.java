package frc.robot.subsystems.swerve;

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
import frc.constants.Settings.SwerveConfig;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import org.littletonrobotics.junction.Logger;

public class ModuleIOSparkMAX implements ModuleIO {
  public int moduleNumber;
  private Rotation2d angleOffset;

  private CANSparkMax mAngleMotor;
  private CANSparkMax mDriveMotor;

  private CANcoder angleEncoder;
  private RelativeEncoder relAngleEncoder;
  private RelativeEncoder relDriveEncoder;

  private SparkPIDController driveController;
  private SparkPIDController angleController;

  public ModuleIOSparkMAX(int moduleNumber, RevSwerveModuleConstants moduleConstants) {
    System.out.println("[Init] Creating ModuleIOSparkMax: " + moduleNumber);

    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /*
     * Angle Motor Config
     */
    mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    configAngleMotor();

    /*
     * Drive Motor Config
     */
    mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    configDriveMotor();

    /*
     * Angle Encoder Config
     */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    configEncoders();

    // lastAngle = getState().angle;
  }

  /*
   *
   * Overriden Interface methods
   *
   */

  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveVoltage = mDriveMotor.getAppliedOutput();
    inputs.driveEncoderPose = relDriveEncoder.getPosition();
    inputs.driveVelocity = relDriveEncoder.getVelocity();
    inputs.driveTemp = mDriveMotor.getMotorTemperature();

    inputs.angleVoltage = mAngleMotor.getAppliedOutput();
    inputs.angleEncoderPose = relAngleEncoder.getPosition();
    inputs.angleVelocity = relAngleEncoder.getVelocity();
    inputs.angleTemp = mAngleMotor.getMotorTemperature();
  }

  @Override
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

  @Override
  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue());
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(relDriveEncoder.getVelocity(), getAngle());
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(relDriveEncoder.getPosition(), getAngle());
  }

  /*
   *
   * Class methods
   *
   */

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
    angleController = mAngleMotor.getPIDController();
    angleController.setP(
        SwerveConfig
            .angleKP); // FIXME IDK if you would need the second paramater for the slotID, I just
    // remove it since it was 0
    angleController.setI(SwerveConfig.angleKI);
    angleController.setD(SwerveConfig.angleKD);
    angleController.setFF(SwerveConfig.angleKF);
    angleController.setOutputRange(-SwerveConfig.anglePower, SwerveConfig.anglePower);
    mAngleMotor.setSmartCurrentLimit(SwerveConfig.angleContinuousCurrentLimit);

    mAngleMotor.setInverted(SwerveConfig.angleMotorInvert);
    mAngleMotor.setIdleMode(SwerveConfig.angleIdleMode);
  }

  private void configDriveMotor() {
    mDriveMotor.restoreFactoryDefaults();
    driveController = mDriveMotor.getPIDController();
    driveController.setP(SwerveConfig.driveKP);
    driveController.setI(SwerveConfig.driveKI);
    driveController.setD(SwerveConfig.driveKD);
    driveController.setFF(SwerveConfig.driveKF);
    driveController.setOutputRange(-SwerveConfig.drivePower, SwerveConfig.drivePower);
    mDriveMotor.setSmartCurrentLimit(SwerveConfig.driveContinuousCurrentLimit);
    mDriveMotor.setInverted(SwerveConfig.driveMotorInvert);
    mDriveMotor.setIdleMode(SwerveConfig.driveIdleMode);
  }

  private void resetToAbsolute() {
    relAngleEncoder.setPosition((360 * getCanCoder().getDegrees()) - angleOffset.getDegrees());
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(relAngleEncoder.getPosition());
  }

  private void setAngle(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConfig.maxSpeed * 0.01)) {
      mAngleMotor.stopMotor();
      return;
    }
    Rotation2d angle = desiredState.angle;
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.

    SparkPIDController controller = angleController;

    double degReference = angle.getDegrees();
    Logger.recordOutput("anglethingy", degReference);

    controller.setReference(degReference, ControlType.kPosition, 0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / SwerveConfig.maxSpeed;
      mDriveMotor.set(percentOutput);
      return;
    }

    double velocity = desiredState.speedMetersPerSecond * SwerveConfig.maxSpeed;
    Logger.recordOutput("velo", velocity);
    SparkPIDController controller = driveController;
    controller.setReference(velocity, ControlType.kVelocity, 0);
  }
}
