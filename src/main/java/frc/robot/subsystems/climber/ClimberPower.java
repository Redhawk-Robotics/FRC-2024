package frc.robot.subsystems.climber;

public enum ClimberPower {
  kStop(0),
  kUp(1),
  kDown(-1),
  kHalfUp(.5),
  kHalfDown(-0.5),
  KquarterUp(.25),
  kquartDown(-.25);

  public double power;

  private ClimberPower(double power) {
    this.power = power;
  }
}
