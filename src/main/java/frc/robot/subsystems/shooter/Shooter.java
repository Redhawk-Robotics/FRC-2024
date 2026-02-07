// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Settings;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooterIO;
  private final ShooterInputsAutoLogged shooterInputs;

  private LoggedDashboardNumber power = new LoggedDashboardNumber("Power of pivot");
  private LoggedDashboardBoolean runPower = new LoggedDashboardBoolean("Run power to pivot?");
  private LoggedDashboardBoolean overrideAutoWheels =
      new LoggedDashboardBoolean("Override auto wheels?");
  private LoggedDashboardBoolean irSim = new LoggedDashboardBoolean("Enable ir shooter?");

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
    this.shooterInputs = new ShooterInputsAutoLogged();

    this.shooterIO.updateInputs(shooterInputs);
    irSim.setDefault(false);
  }

  @Override
  public void periodic() {
    // System.out.println("yay");
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);

    if (runPower.get()) {
      shooterIO.applyShooterSpeed(power.get(), power.get());
    }

    shooterIO.applyShooterSpeed(
        getShooterWheelStates().topShooterPower, getShooterWheelStates().bottomShooterPower);
    shooterIO.applySupportWheelSpeeds(
        getShooterSupportWheelStates().guardPower, getShooterSupportWheelStates().uptakePower);
  }

  public void applyShooterSpeed(double setPowerTop, double setPowerBottom) {
    shooterIO.applyShooterSpeed(setPowerTop, setPowerBottom);
  }

  public void applySupportWheelSpeeds(double guardPower, double uptakePower) {
    shooterIO.applySupportWheelSpeeds(guardPower, uptakePower);
  }

  public boolean isSensorsBeamBroken() {
    // return irSim.get();
    return shooterIO.isShooterSensorBeamBroken();
  }

  // ~ Get Guard and uptake states
  @AutoLogOutput(key = "States/supportWheelStates")
  public ShooterSupportWheelStates getShooterSupportWheelStates() {
    return Settings.ShooterConstants.currentSupportWheelStates;
  }

  // ! Static class to set state
  public static void setSupportWheelStates(ShooterSupportWheelStates desiredState) {
    Settings.ShooterConstants.currentSupportWheelStates = desiredState;
  }

  // ~ Get Shooter wheels state
  @AutoLogOutput(key = "States/shooterWheelState")
  public ShooterWheelStates getShooterWheelStates() {
    if (overrideAutoWheels.get()) {
      setShooterWheelState(ShooterWheelStates.kShooterStop);
    }
    return Settings.ShooterConstants.currentShooterState;
  }

  // ! Static class to set state
  public static void setShooterWheelState(ShooterWheelStates desiredState) {
    Settings.ShooterConstants.currentShooterState = desiredState;
  }

  /*
   * Commands!
   */

  public Command stopShooterWheels() {
    return this.runOnce(() -> setShooterWheelState(ShooterWheelStates.kShooterStop));
  }

  public Command fullSpeedShooterWheels() {
    return this.runOnce(() -> setShooterWheelState(ShooterWheelStates.kShooterFullShot));
  }

  public Command guardsOn() {
    return this.runOnce(() -> setSupportWheelStates(ShooterSupportWheelStates.kSWFeedShooter));
  }

  public Command rejectNote() {
    return new SequentialCommandGroup(
        this.runOnce(() -> setSupportWheelStates(ShooterSupportWheelStates.kSWFeedShooter)),
        this.runOnce(() -> setShooterWheelState(ShooterWheelStates.kShooterIdle)));
  }

  public Command stopNoteRejection() {
    return new SequentialCommandGroup(
        this.runOnce(() -> setSupportWheelStates(ShooterSupportWheelStates.kSWStop)),
        this.runOnce(() -> setShooterWheelState(ShooterWheelStates.kShooterStop)));
  }
}
