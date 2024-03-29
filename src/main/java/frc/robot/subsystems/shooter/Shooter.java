// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Settings;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs;

  private LoggedDashboardNumber power = new LoggedDashboardNumber("Power of pivot");
  private LoggedDashboardBoolean runPower = new LoggedDashboardBoolean("Run power to pivot?");
  private LoggedDashboardBoolean overrideAutoWheels =
      new LoggedDashboardBoolean("Override auto wheels?");

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
    this.shooterInputs = new ShooterIOInputsAutoLogged();

    this.shooterIO.updateInputs(shooterInputs);
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);

    if (runPower.get()) {
      shooterIO.applyShooterSpeed(power.get());
    }

    shooterIO.applyShooterSpeed(getShooterWheelStates().power);
    shooterIO.applySupportWheelSpeeds(
        getShooterSupportWheelStates().guardPower, getShooterSupportWheelStates().uptakePower);
  }

  public void applyShooterSpeed(double setPower) {
    shooterIO.applyShooterSpeed(setPower);
  }

  public void applySupportWheelSpeeds(double guardPower, double uptakePower) {
    shooterIO.applySupportWheelSpeeds(guardPower, uptakePower);
  }

  @AutoLogOutput(key = "Shooter/supportWheelStates")
  public ShooterSupportWheelStates getShooterSupportWheelStates() {
    return Settings.Shooter.supportWheelStates;
  }

  @AutoLogOutput(key = "Shooter/shooterState")
  public ShooterWheelStates getShooterWheelStates() {
    if (overrideAutoWheels.get()) {
      Settings.Shooter.shooterState = ShooterWheelStates.kStop;
    }
    return Settings.Shooter.shooterState;
  }
}
