package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase {
    SwerveDriveOdometry odometry;
    Drivetrain drive;
    Pose2d robotPose = new Pose2d();
    private final Field2d field = new Field2d();

    public Odometry(Drivetrain drive) {
        super();
        this.drive = drive;

        odometry = new SwerveDriveOdometry(drive.getKinematics(), drive.getHeading());
        SmartDashboard.putData("Field", field);
    }
    
    @Override
    public void periodic() {
        robotPose = odometry.update(drive.getHeading(), drive.getSwerveStates());
        field.setRobotPose(robotPose);

        // TODO: Add swerve corners to field if requested
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(pose, drive.getHeading());
    }
}
