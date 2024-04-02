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
    System.out.println("[Init] Creating PivotIOSparkMAX!");

    /*
     * Sparkmax
     */
    this.leftPivot = new CANSparkMax(Ports.pivotID.leftPivot, MotorType.kBrushless);
    this.rightPivot = new CANSparkMax(Ports.pivotID.rightPivot, MotorType.kBrushless);

    pivotEncoder = rightPivot.getAbsoluteEncoder(Type.kDutyCycle);
    pivotController = rightPivot.getPIDController();
    pivotController.setFeedbackDevice(pivotEncoder);

    pivotController.setP(Settings.PivotConstants.pivotKP);
    pivotController.setI(Settings.PivotConstants.pivotKI);
    pivotController.setD(Settings.PivotConstants.pivotKD);
    pivotController.setFF(Settings.PivotConstants.pivotKFF);

    pivotController.setOutputRange(
        Settings.PivotConstants.MIN_INPUT, Settings.PivotConstants.MAX_INPUT);

    leftPivot.setInverted(Settings.PivotConstants.leftPivotInvert);
    leftPivot.setIdleMode(Settings.PivotConstants.pivotNeutralMode);

    rightPivot.setInverted(Settings.PivotConstants.rightPivotInvert);
    rightPivot.setIdleMode(Settings.PivotConstants.pivotNeutralMode);

    leftPivot.follow(rightPivot, true);

    pivotEncoder.setInverted(false);
    rightPivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    rightPivot.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    rightPivot.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward, (float) .21); // TODO check the value for both

    // // TODO
    rightPivot.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) -0.02); // .006

    // leftPivot.setSmartCurrentLimit(Settings.pi.armContinousCurrentLimit)
    // rightPivot.setSmartCurrentLimit(Settings.armSetting.armContinousCurrentLimit);

    rightPivot.burnFlash();
    leftPivot.burnFlash();

    Logger.recordOutput(
        "SoftLimits/Pivot/Forward Soft Limit",
        rightPivot.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kForward));

    Logger.recordOutput(
        "SoftLimits/Pivot/Reverse Soft Limit",
        rightPivot.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kReverse));
  }

  /*
   *
   *
   * Overriden Interface methods
   */
  @Override
  public void updateInputs(PivotInputs inputs) {
    inputs.leftPivotVoltage = leftPivot.getAppliedOutput();
    inputs.leftPivotEncoderPose = leftPivot.getEncoder().getPosition();
    inputs.leftPivotVelocity = leftPivot.getEncoder().getVelocity();
    inputs.leftPivotTemp = leftPivot.getMotorTemperature();
    inputs.leftPivotCurrentSetSpeed = leftPivot.get();
    inputs.leftPivotBusVoltage = leftPivot.getBusVoltage();
    inputs.leftPivotOutputCurrent = leftPivot.getOutputCurrent();
    inputs.leftPivotVoltageCompensation = leftPivot.getVoltageCompensationNominalVoltage();

    inputs.rightPivotVoltage = rightPivot.getAppliedOutput();
    inputs.rightPivotEncoderPose = rightPivot.getEncoder().getPosition();
    inputs.rightPivotVelocity = rightPivot.getEncoder().getVelocity();
    inputs.rightPivotTemp = rightPivot.getMotorTemperature();
    inputs.rightPivotCurrentSetSpeed = rightPivot.get();
    inputs.rightPivotBusVoltage = rightPivot.getBusVoltage();
    inputs.rightPivotOutputCurrent = rightPivot.getOutputCurrent();
    inputs.rightPivotVoltageCompensation = rightPivot.getVoltageCompensationNominalVoltage();

    inputs.isPivotAtRef = atReference();
    inputs.isPivotAtRefCalculation = Math.abs(pivotEncoder.getPosition() - targetPosition);
  }

  @Override
  public void pivotApplySpeed(double speed) {
    setMotorSpeeds(speed);
  }

  @Override
  public void setReference(double targetPosition) {
    this.targetPosition = targetPosition;
    pivotController.setReference(targetPosition, ControlType.kPosition);
  }

  @Override
  public boolean atReference() {
    if (Math.abs(pivotEncoder.getPosition() - targetPosition) < .1) {
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
