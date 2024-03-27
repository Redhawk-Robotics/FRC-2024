package frc.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIONavX implements GyroIO {

  private final AHRS m_gyro;

  public GyroIONavX() {
    System.out.println("[Init] Creating GyroIONavX");

    this.m_gyro = new AHRS();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = m_gyro.isConnected();
    inputs.roll = m_gyro.getRoll();
    inputs.pitch = m_gyro.getPitch();
    inputs.yaw = m_gyro.getYaw();
    inputs.angle = m_gyro.getAngle();
  }

  @Override
  public void reset() {
    m_gyro.reset();
  }

  @Override
  public void setYaw(double deg) {
    m_gyro.setAngleAdjustment(deg);
  }

  @Override
  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }
}
