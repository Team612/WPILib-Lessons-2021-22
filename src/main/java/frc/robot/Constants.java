// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Talon Ports **change accordingly**
    public static int TALON_FR = 6; 
    public static int TALON_FL = 3;
    public static int TALON_BR = 7;
    public static int TALON_BL = 2;

    //Spark Ports (if needed) **change accordingly**
    public static int SPARK_FR = 4; 
    public static int SPARK_FL = 5; 
    public static int SPARK_BR = 6; 
    public static int SPARK_BL = 7; 

    //Driver Ports **change accordingly**
    public static int DRIVER_PORT = 0;
    public static int GUNNER_PORT = 1;

    //Joystick Ports **change accordingly**
    public static int X_AXIS = 0;
    public static int Y_AXIS = 1;
    public static int ROTATION_AXIS = 4;
}
