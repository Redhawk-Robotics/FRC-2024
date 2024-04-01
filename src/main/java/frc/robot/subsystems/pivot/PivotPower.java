package frc.robot.subsystems.pivot;

public enum PivotPower {
  kStop(0),
  kUp(1),
  kDown(-1),
  kHalfUp(.5),
  kHalfDown(-0.5),
  KquarterUp(.25),
  kquartDown(-.25);

  public double power;

  private PivotPower(double power) {
    this.power = power;
  }
}
