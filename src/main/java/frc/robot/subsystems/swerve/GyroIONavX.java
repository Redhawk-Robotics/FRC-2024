package frc.robot.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;

public class GyroIONavX implements GyroIO {

  private final AHRS m_gyro;
  private final double[] yprDegrees = new double[3];
  private final double[] xyzDps = new double[3];

  public GyroIONavX() {
    this.m_gyro = new AHRS();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = m_gyro.isConnected();
    inputs.rollPosition = m_gyro.getRoll();
    inputs.pitchPosition = m_gyro.getPitch();
    inputs.yawPosition = m_gyro.getYaw();
    inputs.rollVelocity = xyzDps[1];
    inputs.pitchVelocity = -xyzDps[0];
    inputs.yawVelocity = xyzDps[2];
  }

}
