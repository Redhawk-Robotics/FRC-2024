package frc.robot.subsystems.shooter;

public enum ShooterWheelStates {
  kStop(0),
  kFullShot(1),
  kHalfShot(.5),
  kIdle(.35),
  kQuarterShot(.25),
  kThreeQuarterShot(.75),
  kSourceIntake(-.5),
  kChamferShot(-1);

  public double power;

  private ShooterWheelStates(double power) {
    this.power = power;
  }
}
