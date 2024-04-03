// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterWheelStates;

public class PivotToShoot extends Command {
  private ShooterWheelStates desiredShooterWheelState;
  private PivotStates desiredPivotStates;
  private Pivot pivot;

  /** Creates a new PivotToShoot. */
  public PivotToShoot(
      Pivot pivot, PivotStates desiredPivotStates, ShooterWheelStates desiredShooterWheelState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.desiredShooterWheelState = desiredShooterWheelState;
    this.desiredPivotStates = desiredPivotStates;
    this.pivot = pivot;

    addRequirements(pivot);
  }

  public PivotToShoot(Pivot pivot, PivotStates desiredPivotStates) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.desiredShooterWheelState = ShooterWheelStates.kShooterFullShot;
    this.desiredPivotStates = desiredPivotStates;
    this.pivot = pivot;

    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("[Command Init] Creating a PivotToShoot Command!");
    Shooter.setShooterWheelState(desiredShooterWheelState);
    Pivot.setPivotState(desiredPivotStates);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ! Command should flash LED
    if (!pivot.pivotAtReference()) {
      System.out.println("[Command Debug] PivotToShoot at WRONG Ref!");
    } else {
      System.out.println("[Command Debug] PivotToShoot at CORRECT Ref!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("[Command Interrupt] Interrupting PivotToShoot Command!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pivot.pivotAtReference()) {
      System.out.println("[Command Debug] PivotToShoot ready to move on!");
      return true;
    }
    return false;
  }
}
