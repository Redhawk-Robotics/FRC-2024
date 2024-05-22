// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;
import java.util.function.DoubleSupplier;

public class galaPivot extends Command {
  private Pivot pivot;
  private DoubleSupplier pivotSpeed;

  /** Creates a new galaPivot. */
  public galaPivot(Pivot pivot, DoubleSupplier pivotSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    this.pivotSpeed = pivotSpeed;

    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = MathUtil.applyDeadband(pivotSpeed.getAsDouble(), 0.1);
    pivot.setPower(MathUtil.clamp(power, -.25, .25));
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
