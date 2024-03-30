// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.commandPreparer.CommandPreparer;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class SourceIntake extends Command {
  private Pivot pivot;
  private Shooter shooter;
  private Timer timer;
  private double firstSawNote = -1;

  /** Creates a new SourceIntake. */
  public SourceIntake(Pivot pivot, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    this.shooter = shooter;
    this.timer = new Timer();

    addRequirements(pivot, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandPreparer.prepareToSourceIntake();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pivot.pivotAtReference() && shooter.getSensorsStatus() && firstSawNote != -1) {
      System.out.println("pivot at right spot");
      firstSawNote = timer.get();
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.get() - firstSawNote > .2) {
      CommandPreparer.prepareToStopSourceIntake();
      return true;
    }
    return false;
  }
}
