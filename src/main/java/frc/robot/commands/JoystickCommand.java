package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class JoystickCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private DoubleSupplier joyX;
    private DoubleSupplier joyY;

    public JoystickCommand(Drivetrain drivetrain, DoubleSupplier joyX, DoubleSupplier joyY) {
        this.drivetrain = drivetrain;
        this.joyX = joyX;
        this.joyY = joyY;
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("INFO: JoystickCommand initialized");
    }

    @Override
    public void execute() {
        // System.out.printf("%f / %f\n", joyX.getAsDouble(), joyY.getAsDouble());
        drivetrain.arcadeDrive(joyY.getAsDouble(), joyX.getAsDouble());
    }

}
