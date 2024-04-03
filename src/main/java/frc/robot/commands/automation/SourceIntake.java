// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.commandPreparer.CommandPreparer;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class SourceIntake extends Command {
  private Pivot pivot;
  private Shooter shooter;
  private Timer timer;
  private double firstSawNote;

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
    System.out.println("[Command Init] Creating a SourceIntake Command!");
    CommandPreparer.prepareToSourceIntake();
    timer.reset();
    timer.start();
    firstSawNote = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("firsttime", firstSawNote);
    if (shooter.isSensorsBeamBroken() && firstSawNote == -1) {
      System.out.println("[Command Debug] SourceIntake will START source intaking timer!");
      firstSawNote = timer.get();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("[Command Interrupt] Interrupting SourceIntake Command!");
    CommandPreparer.prepareToStopAllShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (firstSawNote != -1 && timer.get() - firstSawNote > .1) {
      System.out.println("[Command Debug] SourceIntake STOPPED!");
      CommandPreparer.prepareToStopSourceIntake();
      return true;
    }
    return false;
  }
}
