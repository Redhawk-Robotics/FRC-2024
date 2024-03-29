// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Intake extends SubsystemBase {

  private final IntakeInputsAutoLogged intakeInputs;
  private final IntakeIO intakeIO;

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
  }

  public boolean getEntraceSensorStatus() {
    return intakeIO.intakeEntranceSensorsEnabled();
  }
}
