package org.littletonrobotics.junction;

/** Minimal logger stub to allow compilation when AdvantageKit is unavailable. */
public final class Logger {
  private Logger() {}

  public static void processInputs(String name, Object inputs) {
    // No-op stub for compilation. Teams can enable AdvantageKit for real logging.
  }

  public static void recordOutput(String key, Object value) {
    // No-op stub for compilation.
  }

  public static void recordOutput(String key, double value) {
    // No-op stub for compilation.
  }

  // Additional no-op methods used by the codebase to interact with AdvantageKit.
  public static void recordMetadata(String key, String value) {
    // No-op
  }

  public static void addDataReceiver(Object receiver) {
    // No-op
  }

  public static void setReplaySource(Object reader) {
    // No-op
  }

  public static void registerURCL(Object supplier) {
    // No-op
  }

  public static void start() {
    // No-op
  }
}
