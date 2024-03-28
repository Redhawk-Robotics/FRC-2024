package frc.constants;

import com.revrobotics.CANSparkBase.IdleMode;

//!Settings for the mechanisms
public interface Settings {
    public static final class Shooter {

        public static boolean topShooterInvert = false;
        public static boolean bottomShooterInvert = false;

        public static int shooterVelocityCurrentLimit = 40;

        public static final IdleMode topShooterNeutralMode = IdleMode.kCoast;
        public static final IdleMode bottomShooterNeutralMode = IdleMode.kCoast;

        public static final double shooterKP = 0.0;// FIXME //try 1.0
        public static final double shooterKI = 0.0;// FIXME
        public static final double shooterKD = 0.0;// FIXME //try 0.1
        public static final double shooterKFF = 0.0;// FIXME
    }

    public static final class Intake {
        public static boolean topIntakeInvert = true;
        public static boolean bottomIntakeInvert = false;

        public static int intakeVelocityCurrentLimit = 40;

        public static final IdleMode intakeNeutralMode = IdleMode.kBrake;

        public static final double intakeKP = 0.0;// FIXME //try 1.0
        public static final double intakeKI = 0.0;// FIXME
        public static final double intakeKD = 0.0;// FIXME //try 0.1
        public static final double intakeKFF = 0.0;// FIXME
    }

    public static final class Pivot {
        public static boolean leftPivotInvert = true;
        public static boolean rightPivotInvert = false;

        public static int pivotCurrent = 40;
        public static int armContinousCurrentLimit = 30;

        public static int maxVoltage = 12;

        public static final IdleMode pivotNeutralMode = IdleMode.kBrake;

        public static final double pivotKP = 0.0;// FIXME //try 1.0
        public static final double pivotKI = 0.0;// FIXME
        public static final double pivotKD = 0.0;// FIXME //try 0.1
        public static final double pivotKFF = 0.0;// FIXME
        public static final double kTolerance = 0.02;

        public static final double ZERO_OFFSET = 0;

        public static final double MIN_INPUT = -1.0;
        public static final double MAX_INPUT = 1.0;
        public static final double ARM_DOWN_THRESHOLD = 0;
        public static final double ARM_MIN_DOWN = 0;
        public static final double ARM_MAX_UP = 0;
    }

    public static final class Indexer {
        public static boolean leftIndexerInvert = true;
        public static boolean bottomIndexerInvert = false;

        public static int IndexerCurrent = 20;// TODO idk

        public static final IdleMode IndexerNeutralMode = IdleMode.kBrake;

        public static final double IndexerKP = 0.0;// FIXME //try 1.0
        public static final double IndexerKI = 0.0;// FIXME
        public static final double IndexerKD = 0.0;// FIXME //try 0.1
        public static final double IndexerKFF = 0.0;// FIXME
    }
}