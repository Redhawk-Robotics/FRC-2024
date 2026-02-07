package org.littletonrobotics.junction.networktables;

/** Minimal LoggedDashboardNumber stub to allow compilation when AdvantageKit is not available. */
@SuppressWarnings("unused")
public class LoggedDashboardNumber {
  private final String key;
  private double value = 0.0;

  public LoggedDashboardNumber(String key) {
    this.key = key;
  }

  public void setDefault(double d) {
    this.value = d;
  }

  public void set(double d) {
    this.value = d;
  }

  public double get() {
    return value;
  }
}
