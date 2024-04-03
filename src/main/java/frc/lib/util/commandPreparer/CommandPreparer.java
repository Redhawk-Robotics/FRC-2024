package frc.lib.util.commandPreparer;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSupportWheelStates;
import frc.robot.subsystems.shooter.ShooterWheelStates;

public class CommandPreparer {
  // ~ IntakeToPivot preparer
  public static void prepareToIntakeToPivot() {
    Pivot.setPivotState(PivotStates.kPivotHome);
    Shooter.setShooterWheelState(ShooterWheelStates.kShooterIdle);
    Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWUptake);
  }

  public static void prepareToStopIntakeToPivot() {
    Intake.setIntakeState(IntakeState.kIntakeStop);
    Shooter.setShooterWheelState(ShooterWheelStates.kShooterIdle);
    Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWStop);
  }

  // ~ Shooter end state preparer
  public static void prepareToStopAllShooter() {
    Shooter.setShooterWheelState(ShooterWheelStates.kShooterIdle);
    Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWStop);
  }

  public static void prepareShooterForAnotherIntake() {
    Pivot.setPivotState(PivotStates.kPivotHome);
    Shooter.setShooterWheelState(ShooterWheelStates.kShooterIdle);
    Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWStop);
  }

  public static void prepareToStopShooterAndPivot() {
    Pivot.setPivotState(PivotStates.kPivotHome);
    Shooter.setShooterWheelState(ShooterWheelStates.kShooterStop);
    Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWStop);
  }

  // ~ ShootNote preparer
  public static void prepareToShootNote(
      PivotStates pivotState, ShooterWheelStates shooterWheelState) {
    Shooter.setShooterWheelState(shooterWheelState);
    Pivot.setPivotState(pivotState);
  }

  // ~ SourceIntake preparer
  public static void prepareToSourceIntake() {
    Pivot.setPivotState(PivotStates.kPivotSource);
    Shooter.setShooterWheelState(ShooterWheelStates.kShooterSourceIntake);
    Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWIntake);
  }

  public static void prepareToStopSourceIntake() {
    Pivot.setPivotState(PivotStates.kPivotHome);
    Shooter.setShooterWheelState(ShooterWheelStates.kShooterStop);
    Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWStop);
    Intake.setIntakeState(IntakeState.kIntakeStop);
  }

  // ~ RejectNote preparer
  public static void prepareRobotForRejection() {
    Pivot.setPivotState(PivotStates.kPivotHome);
    Shooter.setShooterWheelState(ShooterWheelStates.kShooterIdle);
    Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWFeedShooter);
    Intake.setIntakeState(IntakeState.kIntakeReverse);
  }
}
