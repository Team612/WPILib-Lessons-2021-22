// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AutoRoutines.Routine1;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.PForward;
import frc.robot.subsystems.RomiDrivetrain;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain m_drivetrain = new RomiDrivetrain();
  private final ArcadeDrive m_arcade = new ArcadeDrive(m_drivetrain);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    configureDefaultCommands();
  }

  private Command generateRamseteCommand(){
    /* 
      Create a voltage constraint to ensure we don't accelerate too fast
      DifferentialDriveVoltageConstraints class constrains differential drive voltage based off of system dynamics.
      Passing in Feedforward control for drivetrain (SimpleFeedForward) -- kS, kV, kA
      Passing in DriveKinematics 
      Also passing in max voltage of 10 Volts instead of 12 because it allows us to compensate for voltage sag
    */
    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics, 10
    );

    /*Create config for trajectory
    Now that we have our voltage constraint, we can create our TrajectoryConfig instance, which wraps together all of our path constraints:
    */
    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared)
    //Add kinematics to ensure max speed is actually obeyed
    .setKinematics(Constants.kDriveKinematics)
    .setReversed(true)
    // Apply the voltage Constraint
    .addConstraint(autoVoltageConstraint);


    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      //Does not take in Pose2d objects
      List.of(
            new Translation2d(-0.5,0) //(forward backward), (left right) 
            //units are in meters
            
        ),
        new Pose2d(-2, 0, new Rotation2d(0)),
        config      
    );

    //Creating RamseteCommand
    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_drivetrain::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        m_drivetrain::tankDriveVolts,
        m_drivetrain);

        m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Set up a sequence of commands
    // First, we want to reset the drivetrain odometry
    return new InstantCommand(() -> m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose()), m_drivetrain)
        // next, we run the actual ramsete command
        .andThen(ramseteCommand)

        // Finally, we make sure that the robot stops
        .andThen(new InstantCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain));
  }

  private void configureButtonBindings() {
    m_chooser.addOption("Autonomous Trajectory", new Routine1(m_drivetrain));
    m_chooser.addOption("P forward", new PForward(m_drivetrain));
    m_chooser.addOption("Ramsete Trajectory", generateRamseteCommand());
    SmartDashboard.putData(m_chooser);
  }

  private void configureDefaultCommands(){
    m_drivetrain.setDefaultCommand(m_arcade);
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }
  
}
