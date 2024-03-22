package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class CTREConfigs {

  public CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {

    swerveCanCoderConfig = new CANcoderConfiguration();

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.MagnetSensor =
        new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
    // swerveCanCoderConfig.absoluteSensorRange =
    // AbsoluteSensorRange.Unsigned_0_to_360;
    swerveCanCoderConfig.MagnetSensor =
        new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
    // swerveCanCoderConfig.sensorDirection = canCoderInvert;
    // swerveCanCoderConfig.initializationStrategy =
    // SensorInitializationStrategy.BootToAbsolutePosition;
    // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }
}
