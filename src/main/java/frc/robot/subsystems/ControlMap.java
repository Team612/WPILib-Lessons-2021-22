package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import javax.swing.KeyStroke;
public class ControlMap {
    public static Joystick m_keyboard = new Joystick(0);
    public static JoystickButton button_a = new JoystickButton(m_keyboard, 1);
    public static JoystickButton button_w = new JoystickButton(m_keyboard, 1);
    
};
