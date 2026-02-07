package org.littletonrobotics.junction;

/** Minimal LogFileUtil stub with helper methods used by Robot.java */
public final class LogFileUtil {
  private LogFileUtil() {}

  public static String findReplayLog() {
    return ""; // Return empty path by default
  }

  public static String addPathSuffix(String path, String suffix) {
    return path + suffix;
  }
}
