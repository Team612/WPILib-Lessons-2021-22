// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class EncoderRotate extends CommandBase {

  private final RomiDrivetrain m_drivetrain;
  private final double m_degrees;
  private final double m_speed;
  private double inchPerDegree;

  /** Creates a new Rotate. */
  public EncoderRotate(double speed, double degrees, RomiDrivetrain drivetrain) {
    m_degrees = degrees;
    m_speed = speed;
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.arcadeDrive(0, 0);
    m_drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(0, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /**
     * Find circumference of the romi bot
     * Measure the length of the robot = 149mm
     * Meausre the width of a wheel = 4mm 
     * subtract the length of the robot from the width --> 149 - 2(4) = 141mm
     * convert to inches --> 5.551 inches
     */
    inchPerDegree = Math.PI * 5.551 / 360;
    // Compare distance travelled from start to distance based on degree turn
    return m_drivetrain.getAverageDistanceInch() >= (inchPerDegree * m_degrees);
  }
}
