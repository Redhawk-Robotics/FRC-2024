package frc.robot.subsystems.shooter;

public enum ShooterWheelStates {
  kShooterStop(0, 0),
  kShooterFullShot(1, 1),
  kShooterHalfShot(.5, .5),
  kShooterIdle(.35, .35),
  kShooterQuarterShot(.25, .25),
  kShooterThreeQuarterShot(.75, .75),
  kShooterSourceIntake(-.25, -.25),
  kAmpShot(.1, 0);

  public double topShooterPower;
  public double bottomShooterPower;

  private ShooterWheelStates(double topShooterPower, double bottomShooterPower) {
    this.topShooterPower = topShooterPower;
    this.bottomShooterPower = bottomShooterPower;
  }
}
