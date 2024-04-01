package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.constants.Ports;
import frc.constants.Settings;
import org.littletonrobotics.junction.Logger;

public class ClimberIOSparkFLEX {
  private final CANSparkFlex leftClimber, rightClimber;
  private final RelativeEncoder leftClimberEncoder, rightClimberEncoder;

  public ClimberIOSparkFLEX() {
    /*
     * Sparkmax
     */
    this.leftClimber = new CANSparkFlex(Ports.climberID.leftClimber, MotorType.kBrushless);
    this.rightClimber = new CANSparkFlex(Ports.climberID.rightClimber, MotorType.kBrushless);

    leftClimber.setInverted(Settings.ClimberConstants.leftClimberInvert);
    leftClimber.setIdleMode(Settings.ClimberConstants.climberNeutralMode);

    rightClimber.setInverted(Settings.ClimberConstants.rightClimberInvert);
    rightClimber.setIdleMode(Settings.ClimberConstants.climberNeutralMode);

    leftClimber.enableSoftLimit(CANSparkFlex.SoftLimitDirection.kForward, true);
    leftClimber.enableSoftLimit(CANSparkFlex.SoftLimitDirection.kReverse, true);

    rightClimber.enableSoftLimit(CANSparkFlex.SoftLimitDirection.kForward, true);
    rightClimber.enableSoftLimit(CANSparkFlex.SoftLimitDirection.kReverse, true);

    // ^ RightClimber
    rightClimber.setSoftLimit(
        CANSparkFlex.SoftLimitDirection.kForward, (float) 0); // TODO check the value for both

    rightClimber.setSoftLimit(CANSparkFlex.SoftLimitDirection.kReverse, (float) 0);

    // ~ LeftClimber
    leftClimber.setSoftLimit(
        CANSparkFlex.SoftLimitDirection.kForward, (float) 0); // TODO check the value for both

    leftClimber.setSoftLimit(CANSparkFlex.SoftLimitDirection.kReverse, (float) 0);

    rightClimber.burnFlash();
    leftClimber.burnFlash();

    rightClimberEncoder = rightClimber.getEncoder();
    leftClimberEncoder = leftClimber.getEncoder();

    // ^ RightClimber
    Logger.recordOutput(
        "rightClimber/Forward Soft Limit",
        rightClimber.isSoftLimitEnabled(CANSparkFlex.SoftLimitDirection.kForward));

    Logger.recordOutput(
        "rightClimber/Reverse Soft Limit",
        rightClimber.isSoftLimitEnabled(CANSparkFlex.SoftLimitDirection.kReverse));

    // ~ LeftClimber
    Logger.recordOutput(
        "leftClimber/Forward Soft Limit",
        leftClimber.isSoftLimitEnabled(CANSparkFlex.SoftLimitDirection.kForward));

    Logger.recordOutput(
        "leftClimber/Reverse Soft Limit",
        leftClimber.isSoftLimitEnabled(CANSparkFlex.SoftLimitDirection.kReverse));
  }

  /*
   *
   *
   * Overriden Interface methods - need to INITIALIZE THIS, but it keeps breaking FIXME PLS
   */
//   @Override
//   public void updateInputs(ClimberInputs inputs) {}

//   @Override
//   public void climberApplySpeed(double speed) {
//     setMotorSpeeds(speed);
//   }

  // @Override
  // public void setReference(double targetPosition) {
  //     this.targetPosition = targetPosition;
  //     pivotController.setReference(targetPosition, ControlType.kPosition);
  // }

  // @Override
  // public boolean atReference() {
  //     Logger.recordOutput("enocder pose", pivotEncoder.getPosition());
  //     Logger.recordOutput("we there", Math.abs(pivotEncoder.getPosition() - targetPosition));
  //     if (Math.abs(pivotEncoder.getPosition() - targetPosition) < .1) {
  //         return true;
  //     }
  //     return false;
  // }

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
