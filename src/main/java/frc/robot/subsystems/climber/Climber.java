package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.Settings;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class Climber extends SubsystemBase {

  private final ClimberInputsAutoLogged climberInputs;
  private final ClimberIO climberIO;

  private LoggedDashboardNumber power = new LoggedDashboardNumber("Power of climber");
  private LoggedDashboardBoolean runPower = new LoggedDashboardBoolean("Run power to climber?");

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
    this.climberInputs = new ClimberInputsAutoLogged();

    this.climberIO.updateInputs(climberInputs);
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Climber", climberInputs);

    if (runPower.get()) {
      climberIO.climberApplySpeed(power.get());
    }
  }

  @AutoLogOutput(key = "States/ClimberState")
  public ClimberState getClimberState() {
    return Settings.ClimberConstants.climberState;
  }

  public static void setClimberState(ClimberState desiredState) {
    Settings.ClimberConstants.climberState = desiredState;
  }

  public static void setClimberPower(ClimberPower desiredState) {
    Settings.ClimberConstants.climberPower = desiredState;
  }

  /*
   * Commands!
   */
  public Command climberToHome() {
    return this.runOnce(() -> setClimberState(ClimberState.kHome));
  }

  public Command climberForward() {
    return this.runOnce(() -> setClimberState(ClimberState.kClimb));
  }

  public Command climberReverse() {
    return this.runOnce(() -> setClimberState(ClimberState.kClimb));
  }

  public Command climberStop() {
    return this.runOnce(() -> setClimberState(ClimberState.kStop));
  }
}
