package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {
  private ModuleIO modIo;
  private int modIndex;
  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  public Module(ModuleIO io, int index) {
    this.modIo = io;
    this.modIndex = index;
  }

  public void periodic() {
    modIo.updateInputs(inputs);
    Logger.processInputs("Swerve/Module " + getModuleNumber(), inputs);

    Logger.recordOutput("REV Mod " + getModuleNumber() + " Cancoder", getCanCoder().getDegrees());
    Logger.recordOutput(
        "REV Mod " + getModuleNumber() + " Integrated", getPosition().angle.getDegrees());
    Logger.recordOutput(
        "REV Mod " + getModuleNumber() + " Velocity", getState().speedMetersPerSecond);
  }

  // Sets the desired state of a swerve module
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    modIo.setDesiredState(desiredState, isOpenLoop);
  }

  // Gets the CanCoder angle
  public Rotation2d getCanCoder() {
    return modIo.getCanCoder();
  }

  // Sets the modules state
  public SwerveModuleState getState() {
    return modIo.getState();
  }

  // Gets the position of driving and turning encoder
  public SwerveModulePosition getPosition() {
    return modIo.getPosition();
  }

  public int getModuleNumber() {
    return modIndex;
  }
}
