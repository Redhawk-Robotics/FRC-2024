package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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
    Logger.processInputs("Swerve/Module " + modIndex, inputs);
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

}
