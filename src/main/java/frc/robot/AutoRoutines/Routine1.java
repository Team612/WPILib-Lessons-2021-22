// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EncoderForward;
import frc.robot.commands.EncoderRotate;
import frc.robot.subsystems.RomiDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Routine1 extends SequentialCommandGroup {
  /** Creates a new Routine1. */
  public Routine1(RomiDrivetrain drivetrain) {
    addCommands(
      new EncoderRotate(0.5, 45, drivetrain),
      new EncoderForward(1.0, 100, drivetrain)
    );
  }
}
