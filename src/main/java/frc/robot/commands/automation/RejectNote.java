// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.commandPreparer.CommandPreparer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class RejectNote extends Command {
  private final Shooter shooter;
  private final Pivot pivot;
  private final Intake intake;

  /** Creates a new RejectNote. */
  public RejectNote(Shooter shooter, Pivot pivot, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.pivot = pivot;
    this.intake = intake;

    addRequirements(shooter, pivot, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("[Command Init] Creating a RejectNote Command!");
    CommandPreparer.prepareRobotForRejection();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("[Command Interrupt] Interrupting RejectNote Command!");
    CommandPreparer.prepareToStopIntakeToPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
