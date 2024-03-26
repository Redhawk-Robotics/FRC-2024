package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOPigeon implements GyroIO {
  private final Pigeon2 m_pigeon;

  public GyroIOPigeon() {
    System.out.println("[Init] Creating GyroIOPigeon");

    this.m_pigeon = new Pigeon2(39);
  }

  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.roll = m_pigeon.getRoll().getValueAsDouble();
    inputs.pitch = m_pigeon.getPitch().getValueAsDouble();
    inputs.yaw = m_pigeon.getYaw().getValueAsDouble();
  }

  @Override
  public void reset() {
    m_pigeon.reset();
  }

  @Override
  public void setYaw(double deg) {
    m_pigeon.setYaw(deg);
  }

  @Override
  public Rotation2d getRotation2d() {
    return m_pigeon.getRotation2d();
  }
}
