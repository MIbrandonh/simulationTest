package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.JoystickCommand;
import frc.robot.subsystems.*;

public class RobotContainer {
    public static Drivetrain drivetrain = new Drivetrain();
    public static final Joystick joystick = new Joystick(0);

    public RobotContainer() {
        drivetrain.setDefaultCommand(
                new JoystickCommand(drivetrain, () -> joystick.getX() / 2, () -> joystick.getY() / -2));
    }

    public void simulationPeriodic() {
        drivetrain.simulationPeriodic();
    }
}
