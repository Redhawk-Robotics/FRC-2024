package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.constants.Ports;
import frc.constants.Settings;
import frc.constants.Constants.IntakeConstants;

public class IntakeIOSparkMAX implements IntakeIO {
  private final CANSparkMax leftIntakeMotor, rightIntakeMotor;
  private final DigitalInput entranceSensor, exitSensor;

  public IntakeIOSparkMAX() {
    /*
     * Sparkmax
     */
    this.leftIntakeMotor = new CANSparkMax(Ports.intakeID.leftIntake, MotorType.kBrushless);
    this.rightIntakeMotor = new CANSparkMax(Ports.intakeID.rightIntake, MotorType.kBrushless);

    this.leftIntakeMotor.setInverted(Settings.Indexer.topIndexerInert);
    this.rightIntakeMotor.setInverted(IntakeConstants.rightIntakeMotorInvert);

    /*
     * IR Sensors
     */
    this.entranceSensor = new DigitalInput(0);
    this.exitSensor = new DigitalInput(0);
  }

  /*
   *
   *
   * Overriden Interface methods
   */

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
  }

  @Override
  public void intakeFloorNote() {
    setMotorSpeeds(1);
  }

  @Override
  public void intakeStop() {
    setMotorSpeeds(0);
  }

  @Override
  public void intakeApplySpeed(double speed) {
    setMotorSpeeds(speed);
  }

  @Override
  public boolean intakeEntranceSensorsEnabled() {
    return false;
  }

  @Override
  public boolean intakeExitSensorsEnabled() {
    return false;
  }

  /*
   *
   * Class methods
   *
   */

  public void setMotorSpeeds(double speed) {
    leftIntakeMotor.set(speed);
    rightIntakeMotor.set(-speed);
  }
}
