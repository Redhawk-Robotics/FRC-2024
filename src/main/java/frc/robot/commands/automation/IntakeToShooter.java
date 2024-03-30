// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.commandPreparer.CommandPreparer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotStates;
import frc.robot.subsystems.shooter.Shooter;

public class IntakeToShooter extends Command {
  private Intake intake;
  private Pivot pivot;
  private Shooter shooter;

  private boolean intakeSensorDetected = false;

  /** Creates a new IntakeToShooter. */
  public IntakeToShooter(Intake intake, Pivot pivot, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.pivot = pivot;
    this.shooter = shooter;

    addRequirements(intake, pivot, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandPreparer.prepareForFloorIntakeToPivot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!pivot.pivotAtReference()) {
      Intake.setIntakeState(IntakeState.kIntakeStop);
    } else {
      System.out.println("pivot at right spot");
      Intake.setIntakeState(IntakeState.kIntakeNote);
    }

    if (intake.getEntraceSensorStatus() && !intakeSensorDetected) {
      intakeSensorDetected = true;
      // TODO LED flash here
    } else {
      intakeSensorDetected = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("we got stopped bc we slow");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooter.getSensorsStatus() && pivot.pivotAtReference()) {
      CommandPreparer.prepareForIntakeToPivotStop(PivotStates.kPivotSubwoofer);
      System.out.println("Intake is now finshed");
      return true; // TODO FLIP VALUE
    }
    System.out.println("cant move onto shooting");
    return false;
  }
}
