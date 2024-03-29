package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import frc.constants.Ports;
import frc.constants.Settings;

public class ShooterIOSparkMAX implements ShooterIO {
  private final CANSparkMax topShooter, bottomShooter, indexer;
  private SparkPIDController velocityController;

  public ShooterIOSparkMAX() {
    this.topShooter = new CANSparkMax(Ports.shooterID.topShooter, MotorType.kBrushless);
    this.bottomShooter = new CANSparkMax(Ports.shooterID.bottomShooter, MotorType.kBrushless);
    this.indexer = new CANSparkMax(Ports.shooterID.indexerMotor, MotorType.kBrushless);

    this.topShooter.restoreFactoryDefaults();
    this.bottomShooter.restoreFactoryDefaults();
    this.indexer.restoreFactoryDefaults();

    this.topShooter.setInverted(Settings.Shooter.topShooterInvert);
    this.bottomShooter.setInverted(Settings.Shooter.bottomShooterInvert);
    this.indexer.setInverted(Settings.Shooter.indexerInvert);

    this.topShooter.setIdleMode(Settings.Shooter.topShooterNeutralMode);
    this.bottomShooter.setIdleMode(Settings.Shooter.bottomShooterNeutralMode);
    this.indexer.setIdleMode(Settings.Shooter.topShooterNeutralMode);

    this.topShooter.setSmartCurrentLimit(Settings.Shooter.shooterCurrentLimit);
    this.bottomShooter.setSmartCurrentLimit(Settings.Shooter.shooterCurrentLimit);
    this.indexer.setSmartCurrentLimit(Settings.Shooter.indexerCurrentLimit);

    this.topShooter.enableVoltageCompensation(Settings.Shooter.maxVoltage);
    this.bottomShooter.enableVoltageCompensation(Settings.Shooter.maxVoltage);
    this.indexer.enableVoltageCompensation(Settings.Shooter.maxVoltage);

    this.velocityController = topShooter.getPIDController();
    this.velocityController = bottomShooter.getPIDController(); // * IDK IF WE NEED THIS

    this.velocityController.setP(0);
    this.velocityController.setI(0);
    this.velocityController.setD(0);
    this.velocityController.setFF(0);

    this.topShooter.burnFlash();
    this.bottomShooter.burnFlash();
    this.indexer.burnFlash();
  }
  /*
   *
   *
   * Overriden Interface methods
   */
  @Override
  public void updateInputs(ShooterIOInputs inputs) {}

  @Override
  public void shooterStop() {
    setMotorSpeeds(0);
  }

  @Override
  public void fullShot() {
    setMotorSpeeds(1);
    System.out.println("fullShot enabled");
  }

  @Override
  public void halfShot() {
    setMotorSpeeds(.5);
    System.out.println("halfShot enabled");
  }

  @Override
  public void quarterShot() {
    setMotorSpeeds(1);
    System.out.println("quarterShot enabled");
  }

  @Override
  public void threeQuarterShot() {
    setMotorSpeeds(.75);
    System.out.println("threeQuarterShot enabled");
  }

  @Override
  public void sourceIntake() {
    setMotorSpeeds(-.5);
    System.out.println("sourceIntake enabled");
  }

  @Override
  public void chamferShot() {
    topShooter.set(1);
    bottomShooter.set(-1);
    System.out.println("chamfer enabled");
  }

  // Indexer
  @Override
  public void indexerStop() {
    indexer.set(0);
  }

  @Override
  public void indexerForward() {
    indexer.set(1);
  }

  @Override
  public void indexerInverse() {
    indexer.set(-1);
  }

  @Override
  public void shooterApplySpeed(double speed) {
    setMotorSpeeds(speed);
  }

  @Override
  public void indexerApplySpeed(double speed) {
    indexer.set(speed);
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
