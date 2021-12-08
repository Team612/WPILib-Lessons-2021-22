// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  //Initialization of Talons
  public final WPI_TalonSRX talon_FL = new WPI_TalonSRX(Constants.TALON_FL);
  public final WPI_TalonSRX talon_FR = new WPI_TalonSRX(Constants.TALON_FR);
  public final WPI_TalonSRX talon_BL = new WPI_TalonSRX(Constants.TALON_BL);
  public final WPI_TalonSRX talon_BR = new WPI_TalonSRX(Constants.TALON_BR);

  //Initialization of Sparks (if needed)
  //public final CANSparkMax talon_FL = new CANSparkMax(Constants.SPARK_FL, MotorType.kBrushed);
  //public final CANSparkMax talon_FR = new CANSparkMax(Constants.SPARK_FL, MotorType.kBrushed);
  //public final CANSparkMax talon_BL = new CANSparkMax(Constants.SPARK_FL, MotorType.kBrushed);
  //public final CANSparkMax talon_BR = new CANSparkMax(Constants.SPARK_FL, MotorType.kBrushed);
  
  //Deadzone
  private final double DEADZONE = 0.1;

  //making object of mecanumDrive
  private MecanumDrive drivetrain;
  
  //constructor
  public DriveTrain() {
    drivetrain = new MecanumDrive(talon_FL, talon_BL, talon_FR, talon_BR);
    talon_FL.setNeutralMode(NeutralMode.Brake);
    talon_FR.setNeutralMode(NeutralMode.Brake);
    talon_BL.setNeutralMode(NeutralMode.Brake);
    talon_BR.setNeutralMode(NeutralMode.Brake);
  }

  //drivetrain method
  public void driveMecanum(double y, double x, double zRot){
    if(Math.abs(x) < DEADZONE) x = 0;
    if(Math.abs(y) < DEADZONE) y = 0;
    if(Math.abs(zRot) < DEADZONE) zRot = 0;
    drivetrain.driveCartesian(y, x, zRot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
