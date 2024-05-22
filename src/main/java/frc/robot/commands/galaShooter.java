// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class galaShooter extends Command {
  private Shooter shooter;
  private DoubleSupplier shooterWheelPower;
  private BooleanSupplier shoot;

  /** Creates a new gala. */
  public galaShooter(Shooter shooter, DoubleSupplier shooterWheelPower, BooleanSupplier shoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.shooterWheelPower = shooterWheelPower;
    this.shoot = shoot;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = MathUtil.applyDeadband(shooterWheelPower.getAsDouble(), 0.1);
    shooter.setPower(MathUtil.clamp(power, 0, .5), shoot.getAsBoolean());
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
