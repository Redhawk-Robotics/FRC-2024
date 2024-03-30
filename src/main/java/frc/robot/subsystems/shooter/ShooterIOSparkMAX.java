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
    this.topShooter = new CANSparkMax(Ports.shooterID.topShooter, MotorType.kBrushless);
    this.bottomShooter = new CANSparkMax(Ports.shooterID.bottomShooter, MotorType.kBrushless);
    this.uptake = new CANSparkMax(Ports.shooterID.uptakeMotor, MotorType.kBrushless);
    this.guard = new CANSparkMax(0, MotorType.kBrushless);

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

    this.topShooter.enableVoltageCompensation(Settings.ShooterConstants.maxVoltage);
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

    this.shooterSensor = new DigitalInput(2);
  }

  /*
   *
   *
   * Overriden Interface methods
   */

  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.on = shooterSensorsEnabled();
  }

  @Override
  public void shooterStop() {
    setMotorSpeeds(0);
  }

  @Override
  public boolean shooterSensorsEnabled() {
    return !shooterSensor.get();
  }

  @Override
  public void applySupportWheelSpeeds(double guardPower, double uptakePower) {
    guard.set(guardPower);
    uptake.set(uptakePower);
  }

  /*
   *
   * Class methods
   *
   */
  public void setMotorSpeeds(double speed) {
    topShooter.set(speed);
    bottomShooter.set(-speed);
  }
}
