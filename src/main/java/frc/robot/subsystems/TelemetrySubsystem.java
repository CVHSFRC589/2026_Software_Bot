// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelemetrySubsystem extends SubsystemBase {
  private static PowerDistribution m_PDH = new PowerDistribution();

  /** Creates a new TelemetrySubsystem. */
  public TelemetrySubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Robot Voltage", m_PDH.getVoltage());
    SmartDashboard.putNumber("PDH Port 20 Current", m_PDH.getCurrent(20));
  }
}
