// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Settings;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Pivot extends SubsystemBase {

  private final PivotIOInputsAutoLogged pivotInputs;
  private final PivotIO pivotIO;

  private LoggedDashboardNumber power = new LoggedDashboardNumber("Power of pivot");
  private LoggedDashboardBoolean runPower = new LoggedDashboardBoolean("Run power to pivot?");

  public Pivot(PivotIO pivotIO) {
    this.pivotIO = pivotIO;
    this.pivotInputs = new PivotIOInputsAutoLogged();

    this.pivotIO.updateInputs(pivotInputs);
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Pivot", pivotInputs);

    if (runPower.get()) {
      pivotIO.pivotApplySpeed(power.get());
    }

    pivotIO.setReference(getPivotStates().encoderPose);
  }

  public void setReference(double targetPosition) {
    pivotIO.setReference(targetPosition);
  }

  @AutoLogOutput(key = "Pivot/pivotState")
  public PivotStates getPivotStates() {
    return Settings.Pivot.pivotState;
  }
}
