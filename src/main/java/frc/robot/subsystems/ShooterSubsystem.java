// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  SparkMax m_topMotor, m_middleMotor, m_bottomMotor;
  RelativeEncoder m_topEncoder, m_middleEncoder, m_bottomEncoder;
  SparkMaxConfig m_topConfig, m_middleConfig, m_bottomConfig;
  SparkClosedLoopController m_closedLoopControllerTop;
  SparkClosedLoopController m_closedLoopControllerMiddle;
  SparkClosedLoopController m_closedLoopControllerBottom;
  double m_topSpeed;
  double m_middleSpeed;
  double m_bottomSpeed;
  SparkMaxConfig m_config;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_topMotor = new SparkMax(0, MotorType.kBrushless);
    m_middleMotor = new SparkMax(0, MotorType.kBrushless);
    m_bottomMotor = new SparkMax(0, MotorType.kBrushless);

    m_config = new SparkMaxConfig();

    m_config.encoder // change these values
        .velocityConversionFactor((1.0 / 9.0));
    m_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0, ClosedLoopSlot.kSlot1) // change
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-0.2, 0.2, ClosedLoopSlot.kSlot1);
    m_topMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_bottomMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_config.inverted(true);
    m_middleMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_closedLoopControllerTop = m_topMotor.getClosedLoopController();
    m_closedLoopControllerMiddle = m_middleMotor.getClosedLoopController();
    m_closedLoopControllerBottom = m_bottomMotor.getClosedLoopController();
    m_topEncoder = m_topMotor.getEncoder();
    m_middleEncoder = m_middleMotor.getEncoder();
    m_bottomEncoder = m_bottomMotor.getEncoder();
  }

  public void moveTopMotor(double PWM) {
    m_topMotor.set(PWM);
    // m_closedLoopControllerLeft.setReference(RPM, ControlType.kVelocity,
    // ClosedLoopSlot.kSlot1);
  }

  public void moveMiddleMotor(double PWM) {
    m_middleMotor.set(PWM);
    // m_closedLoopControllerRight.setReference(RPM, ControlType.kVelocity,
    // ClosedLoopSlot.kSlot1);
  }

  public void moveBottomMotor(double PWM) {
    m_bottomMotor.set(PWM);
    // m_closedLoopControllerRight.setReference(RPM, ControlType.kVelocity,
    // ClosedLoopSlot.kSlot1);
  }

  public void setSpeedTop(double speed) {
    m_topSpeed = speed;
    m_closedLoopControllerTop.setSetpoint(m_topSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  public void setSpeedMiddle(double speed) {
    m_middleSpeed = speed;
    m_closedLoopControllerMiddle.setSetpoint(m_middleSpeed, ControlType.kVoltage, ClosedLoopSlot.kSlot1);
  }

  public void setSpeedBottom(double speed) {
    m_bottomSpeed = speed;
    m_closedLoopControllerBottom.setSetpoint(m_bottomSpeed, ControlType.kVoltage, ClosedLoopSlot.kSlot1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Top Motor Speed", m_topMotor.get());
    SmartDashboard.putNumber("Middle Motor Speed", m_middleMotor.get());
    SmartDashboard.putNumber("Bottom Motor Speed", m_bottomMotor.get());
    // This method will be called once per scheduler run
  }
}
