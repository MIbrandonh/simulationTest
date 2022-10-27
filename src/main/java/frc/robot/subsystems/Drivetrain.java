package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TalonFXUtil;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonFX leftMaster = new WPI_TalonFX(5);
    private WPI_TalonFX leftSlave = new WPI_TalonFX(4);
    private WPI_TalonFX rightMaster = new WPI_TalonFX(2);
    private WPI_TalonFX rightSlave = new WPI_TalonFX(3);

    private Field2d field = new Field2d();
    private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d());

    private DifferentialDrive diffDrive = new DifferentialDrive(leftMaster, rightMaster);
    WPI_Pigeon2 pidgey = new WPI_Pigeon2(1);

    public Drivetrain() {
        leftMaster.configFactoryDefault();
        leftSlave.configFactoryDefault();
        rightMaster.configFactoryDefault();
        rightSlave.configFactoryDefault();

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        // Spin counterclockwise (default) - Spin Clockwise (invert direction)
        leftMaster.setInverted(TalonFXInvertType.CounterClockwise);
        rightMaster.setInverted(TalonFXInvertType.Clockwise);

        leftSlave.setInverted(InvertType.FollowMaster);
        rightSlave.setInverted(InvertType.FollowMaster);

        diffDrive.setSafetyEnabled(false);

        SmartDashboard.putData("Field", field);
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        diffDrive.arcadeDrive(xSpeed, zRotation);
    }

    private DrivebaseSim driveSim = new DrivebaseSim(leftMaster, rightMaster, pidgey);

    public void updateOdometry() {
        odometry.update(pidgey.getRotation2d(),
                TalonFXUtil.nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorPosition()),
                TalonFXUtil.nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorPosition()));
    }

    public void reset() {
        leftMaster.setSelectedSensorPosition(0.0);
        rightMaster.setSelectedSensorPosition(0.0);
    }

    public void updateField() {
        field.setRobotPose(odometry.getPoseMeters());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        reset();
        odometry.resetPosition(pose, pidgey.getRotation2d());
    }

    @Override
    public void periodic() {
        updateOdometry();
        updateField();
    }

    @Override
    public void simulationPeriodic() {
        driveSim.run();
    }
}
