// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RomiDrivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm
  //kWheelDiameterInch * pi = 8.658

  //Instantiation of Spark Objects 
  private final Spark spark_left = new Spark(Constants.SPARK_LEFT);
  private final Spark spark_right = new Spark(Constants.SPARK_RIGHT);

  //Instantiation of Encoder Objects
  private final Encoder left_encoder = new Encoder(Constants.ENCODER_LEFT[0], Constants.ENCODER_LEFT[1]);
  private final Encoder right_encoder = new Encoder(Constants.ENCODER_RIGHT[0],Constants.ENCODER_RIGHT[1]);

  //Instantiation of Gyro Object 
  private final RomiGyro gyro = new RomiGyro();
  //Set up the Accelerometer
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  //Creating Differential Drive object 
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(spark_left, spark_right); //passing in the spark objects

  public RomiDrivetrain(){
    /*
    Set the distance per pulse for this counter. This sets the multiplier used to determine the distance driven based on the count value from the encoder.
    Set this value based on the Pulses per Revolution and factor in any gearing reductions. This distance can be in any units you like, linear or angular.
    its how u convert from encoder ticks to physical distance
    */
    left_encoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    right_encoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
  }

  //making arcade drive method that takes in left and right command
  public void arcadeDrive(double left_command, double right_command) {
    m_diffDrive.arcadeDrive(left_command, right_command); //passing it into the prexisiting built in arcadeDrive method(part of differential drive class)
  }

  //resetting encoders
  public void resetEncoders() {
    left_encoder.reset();
    right_encoder.reset();
  }

  //getting leftencoder count
  public int getLeftEncoderCount(){
    return left_encoder.get();
  }


  //getting right encoder count
  public int getRightEncoderCount(){
    return right_encoder.get();
  }

  //getting left encoder inches
  public double getLeftDistanceInch(){
    return left_encoder.getDistance();
  }

  //getting right encoder inches
  public double getRightDistanceInch(){
    return left_encoder.getDistance();
  }

  //getting average inch distance
  public double getAverageDistanceInch(){
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }
  
  //getting acceleration 
  public double getAccelX(){
    return accelerometer.getX();
  }

  public double getAccelY(){
    return accelerometer.getY();
  }

  public double getAccelZ(){
    return accelerometer.getZ();
  }

  //getting angles
  public double getGyroAngleX(){
    return gyro.getAngleX();
  }

  public double getGyroAngleY(){
    return gyro.getAngleY();
  }

  public double getGyroAngleZ(){
    return gyro.getAngleZ();
  }
  
  public void resetGyro(){
    gyro.reset();
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
