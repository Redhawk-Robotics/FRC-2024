// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Settings.SwerveConfig;
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
    double leftYaxis = MathUtil.applyDeadband(leftPower.getAsDouble(), SwerveConfig.stickDeadband);
    double rightYaxis =
        MathUtil.applyDeadband(rightPower.getAsDouble(), SwerveConfig.stickDeadband);

    climber.climberApplySpeed(leftYaxis, rightYaxis);
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
