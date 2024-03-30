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

public class ShootNote extends Command {
  private Shooter shooter;
  private Pivot pivot;
  private boolean noteAtStart;
  private double timeNoteLastSeen;
  private Timer timer;

  /** Creates a new ShootNote. */
  public ShootNote(Shooter shooter, Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.pivot = pivot;
    this.timer = new Timer();

    addRequirements(shooter, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noteAtStart = shooter.getSensorsStatus();
    timeNoteLastSeen = 0;
    Shooter.setShooterWheelState(ShooterWheelStates.kShooterFullShot);
    Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWFeedShooter);
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!pivot.pivotAtReference()) {
      Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWStop);
    } else {
      System.out.println("pivot at right spot");
      Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWFeedShooter);
      timer.start();
    }

    if (shooter.getSensorsStatus()) {
      System.out.println("Shooter has note during shooting command");
      timeNoteLastSeen = timer.get();
    }
    // TODO change LEDs to shooting mode
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Pivot.setPivotState(PivotStates.kPivotHome);
    Shooter.setSupportWheelStates(ShooterSupportWheelStates.kSWStop);
    System.out.println("we CANT SHOOT");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!noteAtStart && pivot.pivotAtReference()) {
      if (timer.get() > .35) {
        System.out.println("We left because no note at the start");
        NoteVisualizer.shoot();
        return true;
      }
    } else {
      System.out.println("Had a note at the start");
      if (!shooter.getSensorsStatus()
          && timer.get() - timeNoteLastSeen > .1
          && pivot.pivotAtReference()) {
        System.out.println("Had a note at the start BUT we SHOT");
        NoteVisualizer.shoot();
        return true;
      }
    }
    return false;
  }
}
