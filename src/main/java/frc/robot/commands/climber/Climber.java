// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class Climber extends Command {
  private frc.robot.subsystems.climber.Climber climber;
  private DoubleSupplier leftPower;
  private DoubleSupplier rightPower;

  /** Creates a new Climber. */
  public Climber(
      frc.robot.subsystems.climber.Climber climber,
      DoubleSupplier leftPower,
      DoubleSupplier rightPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.leftPower = leftPower;
    this.rightPower = rightPower;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("are we working?");
    climber.climberApplySpeed(leftPower.getAsDouble() * 1, rightPower.getAsDouble() * 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
