// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Sparks
    public static int SPARK_LEFT = 0;
    public static int SPARK_RIGHT = 1;

    //Encoders
    public static int[] ENCODER_LEFT = {4,5};
    public static int[] ENCODER_RIGHT = {6,7};

    //PID Constants and constraints
    public static final double ksVolts = 0.615; //0.443;
    public static final double kvVoltSecondsPerMeter = 10.1; //10.4;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0864;// 0.122;
    public static final double kPDriveVel = 0.161; //0.282;
    public static final double kTrackwidthMeters = 0.15336111932578503; //0.14823496129391722;

    //Helper Class that converts chassis velocity (speed and direction/angle) to left and right wheel velocities for differential drive
    //Inverse kinematics converts a desired chassis speed into left and right velocity components whereas forward kinematics converts left and right component velocities into a linear and angular chassis speed.
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters); //Trackwidth is the distance between the wheels

    //Autonomous constraints
    public static final double kMaxSpeedMetersPerSecond = 0.4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.4;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

}
