package frc.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

public interface Ports {
  // * Ports and IDS for mechanisms
  public static final class Gamepad {
    public static final int DRIVER = 0;
    public static final int OPERATOR = 1;
    public static final int DEBUGGER = 2;
  }

  public static final class shooterID {
    public static final int topShooter = 11;
    public static final int bottomShooter = 10;
    public static final int uptakeMotor = 14;
    public static final int guardMotor = 9;
  }

  public static final class intakeID {
    public static final int leftIntake = 15; // 5
    public static final int rightIntake = 16; // 5
  }

  public static final class pivotID {
    public static final int rightPivot = 12;
    public static final int leftPivot = 13;
  }

  public static final class climberID {
    public static final int leftClimber = 17;
    public static final int rightClimber = 18;
  }

  public static final class irSensorsID {
    public static final int intakeSensor = 0;
    public static final int shooterSensor = 2;
  }

  public static final class REV {
    public static final int pigeonID = 39; // IDK

    /* Module Specific Constants */
    /* Front Left Module */
    public static final class frontLeftModule {

      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 44;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(.263 * 360); // Rotation2d.fromDegrees(37.7);
      // 0.540283*360 .334 * 360
      public static final RevSwerveModuleConstants constants =
          new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module */
    public static final class frontRightModule {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 33;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees((.196 * 360) + 180); // .394 * 360
      public static final RevSwerveModuleConstants constants =
          new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module */
    public static final class backLeftModule {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(.228 * 360); // .449 * 360
      public static final RevSwerveModuleConstants constants =
          new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module */
    public static final class backRightModule {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 22;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees((.014 * 360) + 180); // .586 * 360
      public static final RevSwerveModuleConstants constants =
          new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    public static final RevSwerveModuleConstants[] revSwerveModuleConstants = {
      frontLeftModule.constants,
      frontRightModule.constants,
      backLeftModule.constants,
      backRightModule.constants
    };
  }
}
