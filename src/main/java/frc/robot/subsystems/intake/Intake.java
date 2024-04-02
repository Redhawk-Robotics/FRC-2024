// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Settings;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Intake extends SubsystemBase {

  private final IntakeInputsAutoLogged intakeInputs;
  private final IntakeIO intakeIO;
  public static boolean noteInside;

  private LoggedDashboardNumber power = new LoggedDashboardNumber("Power of intake");
  private LoggedDashboardBoolean runPower = new LoggedDashboardBoolean("Run power to intake?");

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    this.intakeInputs = new IntakeInputsAutoLogged();

    this.intakeIO.updateInputs(intakeInputs);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);

    if (runPower.get()) {
      intakeIO.intakeApplySpeed(power.get());
    }

    // intakeIO.intakeApplySpeed(getIntakeState().intakePower);
  }

  public boolean getEntraceSensorStatus() {
    return intakeIO.intakeEntranceSensorsEnabled();
  }

  @AutoLogOutput(key = "States/IntakeState")
  public IntakeState getIntakeState() {
    return Settings.IntakeConstants.currentIntakeState;
  }

  public static void setIntakeState(IntakeState desiredState) {
    Settings.IntakeConstants.currentIntakeState = desiredState;
  }

  /*
   * Commands!
   */

  public Command enableIntake() {
    return this.runOnce(() -> setIntakeState(IntakeState.kIntakeNote));
  }

  public Command enableReverseIntake() {
    return this.runOnce(() -> setIntakeState(IntakeState.kIntakeReverse));
  }

  public Command stopIntake() {
    return this.runOnce(() -> setIntakeState(IntakeState.kIntakeStop));
  }
}
