// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  double m_topSpeed, m_middleSpeed, m_bottomSpeed;
  ShooterSubsystem m_shooterSubsystem;
  /** Creates a new Shoot. */
  public Shoot(double topSpeed, double middleSpeed, double bottomSpeed, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_topSpeed = topSpeed;
    m_middleSpeed = middleSpeed;
    m_bottomSpeed = bottomSpeed;
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.moveTopMotor(m_topSpeed);
     m_shooterSubsystem.moveMiddleMotor(m_middleSpeed);
      m_shooterSubsystem.moveBottomMotor(m_bottomSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
