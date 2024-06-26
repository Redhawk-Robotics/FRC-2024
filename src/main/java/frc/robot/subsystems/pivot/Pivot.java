// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Settings;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Pivot extends SubsystemBase {

  private final PivotInputsAutoLogged pivotInputs;
  private final PivotIO pivotIO;

  private LoggedDashboardNumber power = new LoggedDashboardNumber("Power of pivot");
  private LoggedDashboardBoolean runPower = new LoggedDashboardBoolean("Run power to pivot?");
  private LoggedDashboardBoolean goodRef = new LoggedDashboardBoolean("Good pivot ref?");
  private boolean OpOverridePower = false;

  public Pivot(PivotIO pivotIO) {
    this.pivotIO = pivotIO;
    this.pivotInputs = new PivotInputsAutoLogged();

    this.pivotIO.updateInputs(pivotInputs);
    goodRef.setDefault(false);
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Pivot", pivotInputs);

    if (runPower.get()) {
      pivotIO.pivotApplySpeed(power.get());
    }

    // if (!OpOverridePower) {
    pivotIO.setReference(getPivotStates().encoderPose);
    // }
  }

  public void setReference(double targetPosition) {
    pivotIO.setReference(targetPosition);
  }

  @AutoLogOutput(key = "place")
  public boolean pivotAtReference() {
    // return goodRef.get(); // TODO MAKE SURE TO CHANGE FOR REAL LIFE
    return pivotIO.atReference(); // TODO MAKE SURE TO CHANGE FOR REAL LIFE
  }

  @AutoLogOutput(key = "States/pivotState")
  public PivotStates getPivotStates() {
    return Settings.PivotConstants.pivotState;
  }

  public static void setPivotState(PivotStates desiredState) {
    Settings.PivotConstants.pivotState = desiredState;
  }

  public static void setPivotPower(PivotPower desiredState) {
    Settings.PivotConstants.pivotPower = desiredState;
  }
  /*
   * Commands!
   */

  public Command pivotToHome() {
    return this.runOnce(() -> setPivotState(PivotStates.kPivotHome));
  }

  public Command pivotFromSubwoofer() {
    return this.runOnce(() -> setPivotState(PivotStates.kPivotHome));
  }

  public Command pivotUp() {
    return this.runOnce(() -> setPivotPower(PivotPower.kUp));
  }

  public void pivot(double power) {
    setOpOverridePower(true);
    if (power == 0) {
      setReference(pivotInputs.boreEncoderPose);
    } else {
      pivotIO.pivotApplySpeed(power);
    }
  }

  public Command pivotDown() {
    return this.runOnce(() -> setPivotPower(PivotPower.kDown));
  }

  public Command pivotStop() {
    return this.runOnce(
        () -> setPivotPower(PivotPower.kStop)); // ~ FIXME IDk if this need to be runOnce or run
  }

  public void setOpOverridePower(boolean set) {
    OpOverridePower = set;
  }
}
