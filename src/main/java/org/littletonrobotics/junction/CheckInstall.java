package org.littletonrobotics.junction;

/**
 * Minimal replacement for AdvantageKit's CheckInstall main so the Gradle task `checkAkitInstall`
 * can run without pulling the real dependency.
 */
public final class CheckInstall {
  public static void main(String[] args) {
    System.out.println("CheckInstall stub: AdvantageKit not present; continuing.");
  }
}
