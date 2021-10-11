// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RomiDrivetrain extends SubsystemBase {

  //Instantiation of Spark Objects 
  private final Spark spark_left = new Spark(Constants.SPARK_LEFT);
  private final Spark spark_right = new Spark(Constants.SPARK_RIGHT);

  //Creating Differential Drive object 
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(spark_left, spark_right); //passing in the spark objects

  //making arcade drive method that takes in left and right command
  public void arcadeDrive(double left_command, double right_command) {
    m_diffDrive.arcadeDrive(left_command, right_command); //passing it into the prexisiting built in arcadeDrive method(part of differential drive class)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
