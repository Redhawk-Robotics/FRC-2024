package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.constants.Ports;
import frc.constants.Settings;

public class ClimberIOSparkFLEX implements ClimberIO {
  private final CANSparkFlex leftClimber, rightClimber;
  private final RelativeEncoder leftClimberEncoder, rightClimberEncoder;

  public ClimberIOSparkFLEX() {
    System.out.println("[Init] Creating ClimberIOSparkFLEX!");

    /*
     * Sparkmax
     */
    this.leftClimber = new CANSparkFlex(Ports.climberID.leftClimber, MotorType.kBrushless);
    this.rightClimber = new CANSparkFlex(Ports.climberID.rightClimber, MotorType.kBrushless);

    leftClimber.setInverted(Settings.ClimberConstants.leftClimberInvert);
    leftClimber.setIdleMode(Settings.ClimberConstants.climberNeutralMode);

    rightClimber.setInverted(Settings.ClimberConstants.rightClimberInvert);
    rightClimber.setIdleMode(Settings.ClimberConstants.climberNeutralMode);

    leftClimber.enableSoftLimit(CANSparkFlex.SoftLimitDirection.kForward, false);
    leftClimber.enableSoftLimit(CANSparkFlex.SoftLimitDirection.kReverse, false);

    rightClimber.enableSoftLimit(CANSparkFlex.SoftLimitDirection.kForward, false);
    rightClimber.enableSoftLimit(CANSparkFlex.SoftLimitDirection.kReverse, false);

    leftClimber.setSmartCurrentLimit(Settings.ClimberConstants.climberCurrent);
    rightClimber.setSmartCurrentLimit(Settings.ClimberConstants.climberCurrent);

    leftClimber.enableVoltageCompensation(Settings.ClimberConstants.maxVoltage);
    rightClimber.enableVoltageCompensation(Settings.ClimberConstants.maxVoltage);

    // ^ RightClimber
    // rightClimber.setSoftLimit(
    //     CANSparkFlex.SoftLimitDirection.kForward, (float) 0); // TODO check the value for both

    // rightClimber.setSoftLimit(CANSparkFlex.SoftLimitDirection.kReverse, (float) 0);

    // ~ LeftClimber
    // leftClimber.setSoftLimit(
    //     CANSparkFlex.SoftLimitDirection.kForward, (float) 0); // TODO check the value for both

    // leftClimber.setSoftLimit(CANSparkFlex.SoftLimitDirection.kReverse, (float) 0);

    rightClimber.burnFlash();
    leftClimber.burnFlash();

    rightClimberEncoder = rightClimber.getEncoder();
    leftClimberEncoder = leftClimber.getEncoder();

    // ^ RightClimber
    // Logger.recordOutput(
    //     "SoftLimits/rightClimber/Forward Soft Limit",
    //     rightClimber.isSoftLimitEnabled(CANSparkFlex.SoftLimitDirection.kForward));

    // Logger.recordOutput(
    //     "SoftLimits/rightClimber/Reverse Soft Limit",
    //     rightClimber.isSoftLimitEnabled(CANSparkFlex.SoftLimitDirection.kReverse));

    // // ~ LeftClimber
    // Logger.recordOutput(
    //     "SoftLimits/leftClimber/Forward Soft Limit",
    //     leftClimber.isSoftLimitEnabled(CANSparkFlex.SoftLimitDirection.kForward));

    // Logger.recordOutput(
    //     "SoftLimits/leftClimber/Reverse Soft Limit",
    //     leftClimber.isSoftLimitEnabled(CANSparkFlex.SoftLimitDirection.kReverse));
  }

  /*
   *
   *
   * Overriden Interface methods - need to INITIALIZE THIS, but it keeps breaking
   * FIXME PLS
   */
  @Override
  public void updateInputs(ClimberInputs inputs) {
    inputs.leftClimberVoltage = leftClimber.getAppliedOutput();
    inputs.leftClimberVelocity = leftClimber.getEncoder().getVelocity();
    inputs.leftClimberTemp = leftClimber.getMotorTemperature();
    inputs.leftClimberCurrentSetSpeed = leftClimber.get();
    inputs.leftClimberBusVoltage = leftClimber.getBusVoltage();
    inputs.leftClimberOutputCurrent = leftClimber.getOutputCurrent();
    inputs.leftClimberVoltageCompensation = leftClimber.getVoltageCompensationNominalVoltage();

    inputs.rightClimberVoltage = rightClimber.getAppliedOutput();
    inputs.rightClimberVelocity = rightClimber.getEncoder().getVelocity();
    inputs.rightClimberTemp = rightClimber.getMotorTemperature();
    inputs.rightClimberCurrentSetSpeed = rightClimber.get();
    inputs.rightClimberBusVoltage = rightClimber.getBusVoltage();
    inputs.rightClimberOutputCurrent = rightClimber.getOutputCurrent();
    inputs.rightClimberVoltageCompensation = rightClimber.getVoltageCompensationNominalVoltage();
  }

  @Override
  public void climberApplySpeed(double leftPower, double rightPower) {
    leftClimber.set(leftPower);
    rightClimber.set(rightPower);
  }

  @Override
  public void setReference(double targetPosition) {
    // this.targetPosition = targetPosition;
    // pivotController.setReference(targetPosition, ControlType.kPosition);
  }

  @Override
  public boolean atReference() {
    // Logger.recordOutput("enocder pose", pivotEncoder.getPosition());
    // Logger.recordOutput("we there", Math.abs(pivotEncoder.getPosition() -
    // targetPosition));
    // if (Math.abs(pivotEncoder.getPosition() - targetPosition) < .1) {
    // return true;
    // }
    // return false;
    return false;
  }

  /*
   *
   * Class methods
   *
   */
  public void setMotorSpeeds(double speed) {
    leftClimber.set(speed);
    rightClimber.set(-speed);
  }

  public double getLeftClimberPosition() {
    return leftClimberEncoder.getPosition();
  }

  public double getRightClimberPosition() {
    return rightClimberEncoder.getPosition();
  }
}
