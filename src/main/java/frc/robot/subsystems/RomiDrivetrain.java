// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RomiDrivetrain extends SubsystemBase {
  //Constants
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm
  private static final double kWheelDiameterMeter = 0.07;


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

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // Also show a field diagram
  private final Field2d m_field2d = new Field2d();

  public RomiDrivetrain(){
    /*
    Set the distance per pulse for this counter. This sets the multiplier used to determine the distance driven based on the count value from the encoder.
    Set this value based on the Pulses per Revolution and factor in any gearing reductions. This distance can be in any units you like, linear or angular.
    its how u convert from encoder ticks to physical distance
    */
    left_encoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    right_encoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    resetEncoders();
    /*
    Class for differential drive odometry. Odometry allows you to track the robot's position on the field over the course of a match using readings from 2 encoders and a gyroscope.
    Teams can use odometry during the autonomous period for complex tasks like path following. 
    Furthermore, odometry can be used for latency compensation when using computer-vision systems.
    It is important that you reset your encoders to zero before using this class. Any subsequent pose resets also require the encoders to be reset to zero.
    */

    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    SmartDashboard.putData("field", m_field2d);
  }

  //making arcade drive method that takes in left and right command
  public void arcadeDrive(double left_command, double right_command) {
    m_diffDrive.arcadeDrive(left_command, right_command); //passing it into the prexisiting built in arcadeDrive method(part of differential drive class)
  }

  public void tankDrive(double left_command, double right_command){
    m_diffDrive.tankDrive(left_command, right_command);
  }


  /**
   * Controls the left and right sides of the drive directly with voltages.
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    spark_left.setVoltage(leftVolts);
    spark_right.setVoltage(-rightVolts); // We invert this to maintain +ve = forward
    m_diffDrive.feed();
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

  public double getLeftDistanceMeter() {
    return left_encoder.getDistance();
  }

  public double getRightDistanceMeter() {
    return right_encoder.getDistance();
  }

  public double getAverageDistanceMeter() {
    return (getLeftDistanceMeter() + getRightDistanceMeter()) / 2.0;
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
    // Update the odometry in the periodic block
    m_odometry.update(gyro.getRotation2d(), left_encoder.getDistance(), right_encoder.getDistance());
    
    // Also update the Field2D object (so that we can visualize this in sim)
    m_field2d.setRobotPose(getPose());
  }
  
  //Returns the position of the robot on the field. (In meters)
  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(left_encoder.getRate(), right_encoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, gyro.getRotation2d());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly
   * @param maxOutput The maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_diffDrive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot
   */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot
   * @return The robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -gyro.getRate();
  }

}
