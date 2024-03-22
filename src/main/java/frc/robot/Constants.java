// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final class REV {
      public static final int pigeonID = 39; // IDK

      /* Module Specific Constants */
      /* Front Left Module */
      public static final class frontLeftModule {

        public static final int driveMotorID = 7;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 44;
        public static final Rotation2d angleOffset =
            Rotation2d.fromDegrees(0); // Rotation2d.fromDegrees(37.7);
        // 0.540283*360
        public static final RevSwerveModuleConstants constants =
            new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Front Right Module */
      public static final class frontRightModule {
        public static final int driveMotorID = 5;
        public static final int angleMotorID = 6;
        public static final int canCoderID = 33;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
        public static final RevSwerveModuleConstants constants =
            new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Left Module */
      public static final class backLeftModule {
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 11;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
        public static final RevSwerveModuleConstants constants =
            new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Right Module */
      public static final class backRightModule {
        public static final int driveMotorID = 3;
        public static final int angleMotorID = 4;
        public static final int canCoderID = 22;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
        public static final RevSwerveModuleConstants constants =
            new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
    }
  }

  public static final class PoseEstimator {
    public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Matrix<N3, N1> VisionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
  }

  public static final class FieldConstants {
    public static final double kBlueSpeakerX = 0.225;
    public static final double kBlueSpeakerY = 5.55;
    public static final double kBlueSpeakerZ = 2.1;

    public static final double kRedSpeakerX = 16.317;
    public static final double kRedSpeakerY = 5.55;
    public static final double kRedSpeakerZ = 2.1;
  }

  public static final class IntakeConstants {
    public static final boolean leftIntakeMotorInvert = false;
    public static final boolean rightIntakeMotorInvert = true;
  }

  public static final class PivotConstants {}

  public static final class ShooterConstants {}

  public static final class VisionConstants {}
}
