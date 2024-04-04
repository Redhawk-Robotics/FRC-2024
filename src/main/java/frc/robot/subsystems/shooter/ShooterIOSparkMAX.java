package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.constants.Ports;
import frc.constants.Settings;

public class ShooterIOSparkMAX implements ShooterIO {
  private final CANSparkMax topShooter, bottomShooter, uptake, guard;
  private SparkPIDController velocityController;
  private final DigitalInput shooterSensor;

  public ShooterIOSparkMAX() {
    System.out.println("[Init] Creating ShooterIOSparkMAX!");

    this.topShooter = new CANSparkMax(Ports.shooterID.topShooter, MotorType.kBrushless);
    this.bottomShooter = new CANSparkMax(Ports.shooterID.bottomShooter, MotorType.kBrushless);
    this.uptake = new CANSparkMax(Ports.shooterID.uptakeMotor, MotorType.kBrushless);
    this.guard = new CANSparkMax(Ports.shooterID.guardMotor, MotorType.kBrushless);

    this.topShooter.restoreFactoryDefaults();
    this.bottomShooter.restoreFactoryDefaults();
    this.uptake.restoreFactoryDefaults();
    this.guard.restoreFactoryDefaults();

    this.topShooter.setInverted(Settings.ShooterConstants.topShooterInvert);
    this.bottomShooter.setInverted(Settings.ShooterConstants.bottomShooterInvert);
    this.uptake.setInverted(Settings.ShooterConstants.uptakeInvert);
    this.guard.setInverted(Settings.ShooterConstants.guardInvert);

    this.topShooter.setIdleMode(Settings.ShooterConstants.topShooterNeutralMode);
    this.bottomShooter.setIdleMode(Settings.ShooterConstants.bottomShooterNeutralMode);
    this.uptake.setIdleMode(Settings.ShooterConstants.topShooterNeutralMode);
    this.guard.setIdleMode(Settings.ShooterConstants.guardNeutralMode);

    this.topShooter.setSmartCurrentLimit(Settings.ShooterConstants.shooterCurrentLimit);
    this.bottomShooter.setSmartCurrentLimit(Settings.ShooterConstants.shooterCurrentLimit);
    this.uptake.setSmartCurrentLimit(Settings.ShooterConstants.indexerCurrentLimit);
    this.guard.setSmartCurrentLimit(Settings.ShooterConstants.guardCurrentLimit);

    this.topShooter.enableVoltageCompensation(
        Settings.ShooterConstants.maxVoltage); // TODO IDK if we need this
    this.bottomShooter.enableVoltageCompensation(Settings.ShooterConstants.maxVoltage);
    this.uptake.enableVoltageCompensation(Settings.ShooterConstants.maxVoltage);
    this.guard.enableVoltageCompensation(Settings.ShooterConstants.maxVoltage);

    this.velocityController = topShooter.getPIDController();
    this.velocityController = bottomShooter.getPIDController(); // * IDK IF WE NEED THIS

    this.velocityController.setP(0);
    this.velocityController.setI(0);
    this.velocityController.setD(0);
    this.velocityController.setFF(0);

    this.topShooter.burnFlash();
    this.bottomShooter.burnFlash();
    this.uptake.burnFlash();
    this.guard.burnFlash();

    this.shooterSensor = new DigitalInput(Ports.irSensorsID.shooterSensor);
  }

  /*
   *
   *
   * Overriden Interface methods
   */

  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.topShooterVoltage = topShooter.getAppliedOutput();
    inputs.topShooterVelocity = topShooter.getEncoder().getVelocity();
    inputs.topShooterTemp = topShooter.getMotorTemperature();
    inputs.topShooterCurrentSetSpeed = topShooter.get();
    inputs.topShooterBusVoltage = topShooter.getBusVoltage();
    inputs.topShooterOutputCurrent = topShooter.getOutputCurrent();
    inputs.topShooterVoltageCompensation = topShooter.getVoltageCompensationNominalVoltage();

    inputs.bottomShooterVoltage = bottomShooter.getAppliedOutput();
    inputs.bottomShooterVelocity = bottomShooter.getEncoder().getVelocity();
    inputs.bottomShooterTemp = bottomShooter.getMotorTemperature();
    inputs.bottomShooterCurrentSetSpeed = bottomShooter.get();
    inputs.bottomShooterBusVoltage = bottomShooter.getBusVoltage();
    inputs.bottomShooterOutputCurrent = bottomShooter.getOutputCurrent();
    inputs.bottomShooterVoltageCompensation = bottomShooter.getVoltageCompensationNominalVoltage();

    inputs.guardVoltage = guard.getAppliedOutput();
    inputs.guardVelocity = guard.getEncoder().getVelocity();
    inputs.guardTemp = guard.getMotorTemperature();
    inputs.guardCurrentSetSpeed = guard.get();
    inputs.guardBusVoltage = guard.getBusVoltage();
    inputs.guardOutputCurrent = guard.getOutputCurrent();
    inputs.guardVoltageCompensation = guard.getVoltageCompensationNominalVoltage();

    inputs.topShooterVoltage = uptake.getAppliedOutput();
    inputs.topShooterVelocity = uptake.getEncoder().getVelocity();
    inputs.topShooterTemp = uptake.getMotorTemperature();
    inputs.topShooterCurrentSetSpeed = uptake.get();
    inputs.topShooterBusVoltage = uptake.getBusVoltage();
    inputs.topShooterOutputCurrent = uptake.getOutputCurrent();
    inputs.topShooterVoltageCompensation = uptake.getVoltageCompensationNominalVoltage();

    inputs.isShooterBeamBroken = isShooterSensorBeamBroken();
  }

  @Override
  public boolean isShooterSensorBeamBroken() {
    // return false;
    return !shooterSensor.get(); // TODO remove for real life testing
  }

  @Override
  public void applySupportWheelSpeeds(double guardPower, double uptakePower) {
    guard.set(guardPower);
    uptake.set(uptakePower);
  }

  @Override
  public void applyShooterSpeed(double topShooterPower, double bottomShooterPower) {
    setMotorSpeeds(topShooterPower, bottomShooterPower);
  }

  /*
   *
   * Class methods
   *
   */
  public void setMotorSpeeds(double topShooterPower, double bottomShooterPower) {
    topShooter.set(topShooterPower);
    bottomShooter.set(-bottomShooterPower);
  }
}
