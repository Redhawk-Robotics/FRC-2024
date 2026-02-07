package org.littletonrobotics.junction.networktables;

/** Minimal LoggedDashboardBoolean stub to allow compilation when AdvantageKit is not available. */
@SuppressWarnings("unused")
public class LoggedDashboardBoolean {
  private final String key;
  private boolean value = false;

  public LoggedDashboardBoolean(String key) {
    this.key = key;
  }

  public void setDefault(boolean v) {
    this.value = v;
  }

  public void set(boolean v) {
    this.value = v;
  }

  public boolean get() {
    return value;
  }
}
