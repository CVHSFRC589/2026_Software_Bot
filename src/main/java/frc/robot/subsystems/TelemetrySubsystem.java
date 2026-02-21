// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelemetrySubsystem extends SubsystemBase {
  private static PowerDistribution m_PDH = new PowerDistribution();

  private final CANrange m_canRange = new CANrange(61);

  /** Creates a new TelemetrySubsystem. */
  public TelemetrySubsystem() {
    CANrangeConfiguration config = new CANrangeConfiguration();

    config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; // If
    // CANrange has a signal strength of at least
    // 2000, it is a valid measurement.
    config.ProximityParams.ProximityThreshold = 0.15; // If CANrange detects an
    // object within 0.1 meters, it will trigger
    // the "isDetected" signal.

    config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz; // Make the
    // CANrange update as fast as possible at
    // 100 Hz. This requires short-range mode.

    m_canRange.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Robot Voltage", m_PDH.getVoltage());
    SmartDashboard.putNumber("PDH Port 20 Current", m_PDH.getCurrent(20));
    SmartDashboard.putNumber("Range Finder Distance", getCANRangeDistance().in(Centimeter));
    SmartDashboard.putNumber("Range Finder Signal Strength", m_canRange.getSignalStrength().getValueAsDouble());
    SmartDashboard.putBoolean("Range Finder IsDetected", m_canRange.getIsDetected().getValue());
  }

  public Distance getCANRangeDistance() {
    return m_canRange.getDistance().getValue();
  }
}
