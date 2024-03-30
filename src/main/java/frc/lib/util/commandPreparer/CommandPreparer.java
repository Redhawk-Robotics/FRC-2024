package frc.lib.util.commandPreparer;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSupportWheelStates;
import frc.robot.subsystems.shooter.ShooterWheelStates;

public class CommandPreparer {
  public static void prepareForFloorIntakeToPivot() {
    Intake.setIntakeState(IntakeState.kIntakeStop);
    Pivot.setPivotState(PivotStates.kPivotHome);
    Shooter.setShooterWheelState(ShooterWheelStates.kShooterFullShot);
    Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWUptake);
  }

  public static void prepareForIntakeToPivotStop(PivotStates pivotState) {
    Intake.setIntakeState(IntakeState.kIntakeStop);
    Pivot.setPivotState(pivotState);
    Shooter.setShooterWheelState(ShooterWheelStates.kShooterFullShot);
    Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWStop);
  }

  public static void prepareForStoppingIntakeToPivot() {
    Intake.setIntakeState(IntakeState.kIntakeStop);
    Shooter.setShooterWheelState(ShooterWheelStates.kShooterIdle);
    Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWStop);
  }

  public static void prepareToStopAllShooter() {
    Shooter.setShooterWheelState(ShooterWheelStates.kShooterIdle);
    Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWStop);
  }
}