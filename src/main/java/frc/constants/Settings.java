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

package frc.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSFalconSwerveConstants;

/**
 * The Settings class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

// !Settings for the mechanisms
public interface Settings {

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

    public static final IdleMode driveIdleMode = IdleMode.kBrake;
    public static final IdleMode angleIdleMode = IdleMode.kBrake;
    public static final double drivePower = 1;
    public static final double anglePower = .9;

    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule =
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L3);

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

    public static final Translation2d[] m_swerveTranslation2d = {
      m_frontLeftTranslation, m_frontRightTranslation, m_backLeftTranslation, m_backRightTranslation
    };

    /*
     * Swerve Kinematics No need to ever change this unless you are not doing a
     * traditional rectangular/square 4 module swerve
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
    public static final int angleContinuousCurrentLimit = 40;
    public static final int anglePeakCurrentLimit = 60;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 40;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving. We found a small open loop ramp (0.25) helps with tread wear,
     * tipping, etc
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
     * Drive Motor Characterization Values Divide SYSID values by 12 to convert from
     * volts to percent output for CTRE
     */
    public static final double driveKS = (0.32);
    public static final double driveKV = (1.51);
    public static final double driveKA = (0.27);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.469892;
    /** Radians per Second */
    public static final double maxAngularVelocity = 5; // max 10 or.....

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
            SwerveConfig.maxSpeed, // Max module speed, in m/s
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
  
//!Mechanisms
  public static final class Shooter {

    public static boolean topShooterInvert = false;
    public static boolean bottomShooterInvert = false;

    public static int shooterCurrentLimit = 40;

    public static int maxVoltage = 12;

    public static final IdleMode topShooterNeutralMode = IdleMode.kCoast;
    public static final IdleMode bottomShooterNeutralMode = IdleMode.kCoast;

    public static final double shooterKP = 0.0; // FIXME //try 1.0
    public static final double shooterKI = 0.0; // FIXME
    public static final double shooterKD = 0.0; // FIXME //try 0.1
    public static final double shooterKFF = 0.0; // FIXME

    public static boolean indexerInvert = false;
    public static int indexerCurrentLimit = 40;
    public static final IdleMode indexerNeutralMode = IdleMode.kCoast;
  }

  public static final class Intake {
    public static boolean leftIntakeInvert = true;
    public static boolean rightIntakeInvert = false;

    public static int intakeCurrentLimit = 40;

    public static int maxVoltage = 12;

    public static final IdleMode intakeNeutralMode = IdleMode.kBrake;

    public static final double intakeKP = 0.0; // FIXME //try 1.0
    public static final double intakeKI = 0.0; // FIXME
    public static final double intakeKD = 0.0; // FIXME //try 0.1
    public static final double intakeKFF = 0.0; // FIXME
  }

  public static final class Pivot {
    public static boolean leftPivotInvert = true;
    public static boolean rightPivotInvert = false;

    public static boolean pivotInvert = false;

    public static int pivotCurrent = 40;
    public static int armContinousCurrentLimit = 40;

    public static int maxVoltage = 12;

    public static int forwardSoftLimit = 0;
    public static int reverseSoftLimit = 0;

    public static final IdleMode pivotNeutralMode = IdleMode.kBrake;

    public static final double pivotKP = 0.0; // FIXME //try 1.0
    public static final double pivotKI = 0.0; // FIXME
    public static final double pivotKD = 0.0; // FIXME //try 0.1
    public static final double pivotKFF = 0.0; // FIXME
    public static final double kTolerance = 0.02;

    public static final double ZERO_OFFSET = 0;

    public static final double MIN_INPUT = -1.0;
    public static final double MAX_INPUT = 1.0;
    public static final double ARM_DOWN_THRESHOLD = 0;
    public static final double ARM_MIN_DOWN = 0;
    public static final double ARM_MAX_UP = 0;
  }
}
