// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.Settings;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterWheelStates;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterWheels extends Command {
  private final Shooter shooter;
  private Pose2d robotPose;
  private Alliance currentAlliance = null;
  private double switchpoint = 8.28; // Unit in meters

  /** Creates a new ShooterWheels. */
  public ShooterWheels(Shooter shooter, Supplier<Pose2d> robotPose) {
    this.shooter = shooter;
    this.robotPose = robotPose.get();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    if (Settings.currentMode == Settings.Mode.REAL) { // TODO PLEASE CHANGE THIS AFTER
      return;
    }
    if (alliance.isPresent()) {
      currentAlliance =
          alliance.get() == DriverStation.Alliance.Red
              ? DriverStation.Alliance.Red
              : DriverStation.Alliance.Blue;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("alliance", currentAlliance);
    if (currentAlliance == null) {
      Settings.Shooter.shooterState = ShooterWheelStates.kIdle;
      return;
    }

    if (currentAlliance == DriverStation.Alliance.Blue) {
      if (robotPose.getX() < switchpoint) {
        Settings.Shooter.shooterState = ShooterWheelStates.kIdle;
      } else {
        Settings.Shooter.shooterState = ShooterWheelStates.kStop;
      }
    } else if (currentAlliance == DriverStation.Alliance.Red) {
      if (robotPose.getX() > switchpoint) {
        Settings.Shooter.shooterState = ShooterWheelStates.kIdle;
      } else {
        Settings.Shooter.shooterState = ShooterWheelStates.kStop;
      }
    }
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
