package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.constants.Ports;
import frc.constants.Settings;

public class IntakeIOSparkMAX implements IntakeIO {
  private final CANSparkMax leftIntakeMotor, rightIntakeMotor;
  private final DigitalInput entranceSensor;

  public IntakeIOSparkMAX() {
    /*
     * Sparkmax
     */
    this.leftIntakeMotor = new CANSparkMax(Ports.intakeID.leftIntake, MotorType.kBrushless);
    this.rightIntakeMotor = new CANSparkMax(Ports.intakeID.rightIntake, MotorType.kBrushless);

    this.leftIntakeMotor.restoreFactoryDefaults();
    this.rightIntakeMotor.restoreFactoryDefaults();

    this.leftIntakeMotor.setInverted(Settings.IntakeConstants.leftIntakeInvert);
    this.rightIntakeMotor.setInverted(Settings.IntakeConstants.rightIntakeInvert);

    this.leftIntakeMotor.setIdleMode(Settings.IntakeConstants.intakeNeutralMode);
    this.rightIntakeMotor.setIdleMode(Settings.IntakeConstants.intakeNeutralMode);

    this.leftIntakeMotor.setSmartCurrentLimit(Settings.IntakeConstants.intakeCurrentLimit);
    this.rightIntakeMotor.setSmartCurrentLimit(Settings.IntakeConstants.intakeCurrentLimit);

    this.leftIntakeMotor.enableVoltageCompensation(Settings.IntakeConstants.maxVoltage);
    this.rightIntakeMotor.enableVoltageCompensation(Settings.IntakeConstants.maxVoltage);

    this.leftIntakeMotor.burnFlash();
    this.rightIntakeMotor.burnFlash();

    /*
     * IR Sensors
     */
    this.entranceSensor = new DigitalInput(Ports.irSensorsID.intakeSensor);
  }

  /*
   *
   *
   * Overriden Interface methods
   */

  @Override
  public void updateInputs(IntakeInputs inputs) {}

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
