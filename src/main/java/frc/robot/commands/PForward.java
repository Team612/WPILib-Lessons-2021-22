// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RomiDrivetrain;

public class PForward extends CommandBase {
  /** Creates a new PForward. */
  private final RomiDrivetrain m_drivetrain;
  private double target_theta;
  private double kP = 0.015;


  public PForward(RomiDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
    m_drivetrain.resetGyro();
    target_theta = m_drivetrain.getGyroAngleZ();

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double error = target_theta- m_drivetrain.getGyroAngleZ();
    m_drivetrain.tankDrive(.5 + (kP * error), .5 - (kP * error));
    System.out.println("Error " + error);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
