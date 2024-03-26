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

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSFalconSwerveConstants;
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

  public static final class SwerveConfig {
    public static final double stickDeadband = 0.1;

    public CANcoderConfiguration canCoderConfig;

    //
    public static final IdleMode driveIdleMode = IdleMode.kBrake;
    public static final IdleMode angleIdleMode = IdleMode.kBrake;
    public static final double drivePower = 1;
    public static final double anglePower = .9;

    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule =
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(28.5);
    public static final double wheelBase = Units.inchesToMeters(28.5);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    public static final Translation2d m_frontLeftTranslation =
        new Translation2d(wheelBase / 2, trackWidth / 2);
    public static final Translation2d m_frontRightTranslation =
        new Translation2d(wheelBase / 2, -trackWidth / 2);
    public static final Translation2d m_backLeftTranslation =
        new Translation2d(-wheelBase / 2, trackWidth / 2);
    public static final Translation2d m_backRightTranslation =
        new Translation2d(-wheelBase / 2, -trackWidth / 2);

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            m_frontLeftTranslation,
            m_frontRightTranslation,
            m_backLeftTranslation,
            m_backRightTranslation);

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    // encoder setup
    // meters per rotation
    public static final double driveRevToMeters = wheelCircumference / (driveGearRatio);
    public static final double driveRpmToMetersPerSecond = driveRevToMeters / 60;
    // the number of degrees that a single rotation of the turn motor turns the
    // wheel.
    public static final double DegreesPerTurnRotation = 360 / angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.05;
    public static final double angleKI = 0;
    public static final double angleKD = 0;
    public static final double angleKF = 0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /*
     * Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE
     */
    public static final double driveKS = (0.32);
    public static final double driveKV = (1.51);
    public static final double driveKA = (0.27);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.0;
    /** Radians per Second */
    public static final double maxAngularVelocity = 5.0; // max 10 or.....

    // public void SwerveConfig() {
    // canCoderConfig = new CANcoderConfiguration();
    // canCoderConfig.MagnetSensor = new MagnetSensorConfigs()
    // .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
    // // canCoderConfig.absoluteSensorRange =
    // AbsoluteSensorRange.Unsigned_0_to_360;
    // canCoderConfig.MagnetSensor = new MagnetSensorConfigs()
    // .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
    // // canCoderConfig.sensorDirection = canCoderInvert;
    // // canCoderConfig.initializationStrategy =
    // // SensorInitializationStrategy.BootToAbsolutePosition;
    // // canCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    // }

    public static final class REV {
      public static final int pigeonID = 39; // IDK

      /* Module Specific Constants */
      /* Front Left Module */
      public static final class frontLeftModule {

        public static final int driveMotorID = 7;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 44;
        public static final Rotation2d angleOffset =
            Rotation2d.fromDegrees(.334 * 360); // Rotation2d.fromDegrees(37.7);
        // 0.540283*360
        public static final RevSwerveModuleConstants constants =
            new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Front Right Module */
      public static final class frontRightModule {
        public static final int driveMotorID = 5;
        public static final int angleMotorID = 6;
        public static final int canCoderID = 33;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(.394 * 360);
        public static final RevSwerveModuleConstants constants =
            new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Left Module */
      public static final class backLeftModule {
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 11;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(.449 * 360);
        public static final RevSwerveModuleConstants constants =
            new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }

      /* Back Right Module */
      public static final class backRightModule {
        public static final int driveMotorID = 3;
        public static final int angleMotorID = 4;
        public static final int canCoderID = 22;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(.586 * 360);
        public static final RevSwerveModuleConstants constants =
            new RevSwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      }
    }
  }

  public static final class PoseEstimator {
    public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Matrix<N3, N1> VisionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    public static final double kDriveBaseRadius = .4572;

    public static final double kPXController = 0;
    public static final double kPYController = 0;
    public static final double kPThetaController = 0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final HolonomicPathFollowerConfig kPathFollowerConfig =
        new HolonomicPathFollowerConfig(
            new PIDConstants(1.9, 0, 0), // Translation PID constants, around 2.25
            new PIDConstants(2, 0, 0), // Rotation PID constants
            kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest
            // module.
            new ReplanningConfig(false, false));
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
