// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {

  private final PivotIOInputsAutoLogged pivotInputs;
  private final PivotIO pivotIO;

  public Pivot(PivotIO pivotIO) {
    this.pivotIO = pivotIO;
    this.pivotInputs = new PivotIOInputsAutoLogged();

    this.pivotIO.updateInputs(pivotInputs);
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Pivot", pivotInputs);
  }
}
