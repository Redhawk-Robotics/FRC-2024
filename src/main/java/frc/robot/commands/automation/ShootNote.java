// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotStates;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterSupportWheelStates;
import frc.robot.subsystems.shooter.ShooterWheelStates;
import frc.robot.utils.NoteVisualizer;
import org.littletonrobotics.junction.Logger;

public class ShootNote extends Command {
  private Shooter shooter;
  private Pivot pivot;
  private PivotStates desiredPivotStates;
  private boolean noteAtStart;
  private double timeNoteLastSeen;
  private Timer timer;
  private static int count;

  /** Creates a new ShootNote. */
  public ShootNote(Shooter shooter, Pivot pivot, PivotStates desiredPivotStates) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.pivot = pivot;
    this.desiredPivotStates = desiredPivotStates;
    this.timer = new Timer();
    count++;

    addRequirements(shooter, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("[Command Init] Creating a ShootNote Command!");
    noteAtStart = shooter.getSensorsStatus();
    timeNoteLastSeen = 0;
    Shooter.setShooterWheelState(ShooterWheelStates.kShooterFullShot);
    Pivot.setPivotState(desiredPivotStates);
    timer.reset();
    Logger.recordOutput("Count/ShootNote", count);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(
        "[Command Debug] ShootNote current Pivot state: " + desiredPivotStates + "!");
    if (!pivot.pivotAtReference()) {
      System.out.println("[Command Debug] ShootNote at WRONG Ref!");
      Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWStop);
    } else {
      System.out.println("[Command Debug] ShootNote at CORRECT Ref!");
      Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWFeedShooter);
      System.out.println("[Command Debug] ShootNote timer will now start!");
      timer.start();
    }

    if (shooter.getSensorsStatus()) {
      System.out.println("[Command Debug] ShootNote currently has a Note!");
      timeNoteLastSeen = timer.get();
    }
    // TODO change LEDs to shooting mode
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("[Command Interrupt] Interrupting ShootNote Command!");
    // // Pivot.setPivotState(PivotStates.kPivotHome);
    // // Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWStop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!noteAtStart && pivot.pivotAtReference()) {
      if (timer.get() > .35) {
        System.out.println("[Command Debug] ShootNote shot since no note was at start!");
        NoteVisualizer.shoot();
        return true;
      }
    } else {
      System.out.println("[Command Debug] ShootNote command had a Note at the start!");
      if (!shooter.getSensorsStatus()
          && timer.get() - timeNoteLastSeen > .1
          && pivot.pivotAtReference()) {
        System.out.println("[Command Debug] ShootNote shot!");
        NoteVisualizer.shoot();
        return true;
      }
    }
    return false;
  }
}
