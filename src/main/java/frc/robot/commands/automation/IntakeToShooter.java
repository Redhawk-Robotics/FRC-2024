// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.commandPreparer.CommandPreparer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class IntakeToShooter extends Command {
  private Intake intake;
  private Pivot pivot;
  private Shooter shooter;
  private static int count;

  private boolean intakeSensorDetected = false;

  /** Creates a new IntakeToShooter. */
  public IntakeToShooter(Intake intake, Pivot pivot, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.pivot = pivot;
    this.shooter = shooter;
    count++;

    addRequirements(intake, pivot, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("[Command Init] Creating a IntakeToShooter Command!");
    CommandPreparer.prepareToIntakeToPivot();
    Logger.recordOutput("Count/IntakeToShooter", count);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!pivot.pivotAtReference()) {
      System.out.println("[Command Debug] IntakeToShooter at WRONG Ref!");
      Intake.setIntakeState(IntakeState.kIntakeStop);
    } else {
      System.out.println("[Command Debug] IntakeToShooter at CORRECT Ref!");
      Intake.setIntakeState(IntakeState.kIntakeNote);
    }

    if (intake.getEntraceSensorStatus() && !intakeSensorDetected) {
      intakeSensorDetected = true;
      // LED flash placeholder
    } else {
      intakeSensorDetected = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("[Command Interrupt] Interrupting IntakeToShooter Command!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooter.isSensorsBeamBroken() && pivot.pivotAtReference()) {
      CommandPreparer.prepareToStopIntakeToPivot();
      System.out.println("[Command Debug] IntakeToShooter is now FINISHED!");
      return true;
    }
    System.out.println("[Command Debug] IntakeToShooter can't move on!");
    return false;
  }
}
