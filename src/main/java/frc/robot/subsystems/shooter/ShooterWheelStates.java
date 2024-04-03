package frc.robot.subsystems.shooter;

public enum ShooterWheelStates {
  kShooterStop(0),
  kShooterFullShot(1),
  kShooterHalfShot(.5),
  kShooterIdle(.35),
  kShooterQuarterShot(.25),
  kShooterThreeQuarterShot(.75),
  kShooterSourceIntake(-.25),
  kShooterChamferShot(-1);

  public double power;

  private ShooterWheelStates(double power) {
    this.power = power;
  }
}
