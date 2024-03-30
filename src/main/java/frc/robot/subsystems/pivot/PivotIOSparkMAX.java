package frc.robot.subsystems.pivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import frc.constants.Ports;
import frc.constants.Settings;
import org.littletonrobotics.junction.Logger;

public class PivotIOSparkMAX implements PivotIO {
  private final CANSparkMax leftPivot, rightPivot;
  private SparkPIDController pivotController;
  private AbsoluteEncoder pivotEncoder;
  private double targetPosition;

  public PivotIOSparkMAX() {
    /*
     * Sparkmax
     */
    this.leftPivot = new CANSparkMax(Ports.pivotID.leftPivot, MotorType.kBrushless);
    this.rightPivot = new CANSparkMax(Ports.pivotID.rightPivot, MotorType.kBrushless);

    leftPivot.restoreFactoryDefaults();
    rightPivot.restoreFactoryDefaults();

    this.leftPivot.setInverted(Settings.PivotConstants.leftPivotInvert);
    this.rightPivot.setInverted(Settings.PivotConstants.rightPivotInvert);

    this.leftPivot.follow(rightPivot);

    this.leftPivot.setIdleMode(Settings.PivotConstants.pivotNeutralMode);
    this.rightPivot.setIdleMode(Settings.PivotConstants.pivotNeutralMode);

    this.leftPivot.setSmartCurrentLimit(Settings.PivotConstants.armContinousCurrentLimit);
    this.rightPivot.setSmartCurrentLimit(Settings.PivotConstants.armContinousCurrentLimit);

    this.leftPivot.enableVoltageCompensation(Settings.PivotConstants.maxVoltage);
    this.rightPivot.enableVoltageCompensation(Settings.PivotConstants.maxVoltage);

    this.pivotEncoder = rightPivot.getAbsoluteEncoder(Type.kDutyCycle);
    this.pivotEncoder.setInverted(Settings.PivotConstants.pivotInvert);
    this.pivotEncoder.setZeroOffset(
        Settings.PivotConstants.ZERO_OFFSET); // FIXME need to find the value

    this.pivotController = rightPivot.getPIDController();
    this.pivotController.setFeedbackDevice(pivotEncoder);

    this.pivotController.setP(Settings.PivotConstants.pivotKP);
    this.pivotController.setI(Settings.PivotConstants.pivotKI);
    this.pivotController.setD(Settings.PivotConstants.pivotKD);
    this.pivotController.setFF(Settings.PivotConstants.pivotKFF);

    this.pivotController.setOutputRange(
        Settings.PivotConstants.MIN_INPUT, Settings.PivotConstants.MAX_INPUT);

    this.rightPivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    this.rightPivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    this.rightPivot.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward,
        Settings.PivotConstants.forwardSoftLimit); // TODO check the value for both forward and
    // // TODO reverse
    this.rightPivot.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse, Settings.PivotConstants.reverseSoftLimit);

    this.rightPivot.burnFlash();
    this.leftPivot.burnFlash();

    Logger.recordOutput(
        "Pivot/Forward Soft Limit",
        rightPivot.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kForward));

    Logger.recordOutput(
        "Pivot/Reverse Soft Limit",
        rightPivot.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kReverse));
  }

  /*
   *
   *
   * Overriden Interface methods
   */
  @Override
  public void updateInputs(PivotInputs inputs) {}

  @Override
  public void pivotStop() {
    setMotorSpeeds(0);
  }

  @Override
  public void pivotUp() {
    setMotorSpeeds(1);
  }

  @Override
  public void pivotDown() {
    setMotorSpeeds(-1);
  }

  @Override
  public void pivotApplySpeed(double speed) {
    setMotorSpeeds(speed);
  }

  @Override
  public void setReference(double targetPosition) {
    this.targetPosition = targetPosition;
    pivotController.setReference(targetPosition, ControlType.kSmartMotion);
  }

  @Override
  public boolean atReference() {
    if (Math.abs(pivotEncoder.getPosition() - targetPosition) < 5) {
      return true;
    }
    return false;
  }

  /*
   *
   * Class methods
   *
   */
  public void setMotorSpeeds(double speed) {
    leftPivot.set(speed);
    rightPivot.set(-speed);
  }

  public double getPosition() {
    return pivotEncoder.getPosition();
  }
}
