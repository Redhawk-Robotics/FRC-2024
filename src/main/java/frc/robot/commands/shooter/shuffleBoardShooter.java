// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.commandPreparer.CommandPreparer;
import frc.robot.subsystems.shooter.Shooter;

public class shuffleBoardShooter extends Command {

  private double guardShooterSpeed;
  private double frontShooterSpeed;
  private Shooter shooter;

  /** Creates a new shuffleBoardShooter. */
  public shuffleBoardShooter(Shooter shooter, double guardShooterSpeed, double frontShooterSpeed) {
    this.guardShooterSpeed = guardShooterSpeed;
    this.frontShooterSpeed = frontShooterSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.applyShooterSpeed(frontShooterSpeed, guardShooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandPreparer.prepareToStopAllShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    CommandPreparer.prepareToStopAllShooter();
    return false;
  }
}
