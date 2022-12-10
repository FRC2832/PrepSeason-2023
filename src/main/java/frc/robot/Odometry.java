package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase {
    private final boolean PLOT_SWERVE_CORNERS = false;
    SwerveDriveOdometry odometry;
    Drivetrain drive;
    Pose2d robotPose = new Pose2d();
    private final Field2d field = new Field2d();
    private Translation2d[] swervePositions = {
        Constants.SWERVE_FRONT_LEFT_LOCATION,
        Constants.SWERVE_FRONT_RIGHT_LOCATION,
        Constants.SWERVE_BACK_LEFT_LOCATION,
        Constants.SWERVE_BACK_RIGHT_LOCATION
    };

    public Odometry(Drivetrain drive) {
        super();
        this.drive = drive;

        odometry = new SwerveDriveOdometry(drive.getKinematics(), drive.getHeading());
        SmartDashboard.putData("Field", field);
    }
    
    @Override
    public void periodic() {
        Rotation2d heading = drive.getHeading();
        SwerveModuleState[] states = drive.getSwerveStates();
        robotPose = odometry.update(heading, states);
        field.setRobotPose(robotPose);

        if(PLOT_SWERVE_CORNERS) {
            // Update the poses for the swerveModules. Note that the order of rotating the
            // position and then adding the translation matters
            var modulePoses = new Pose2d[swervePositions.length];
            for (int i = 0; i < swervePositions.length; i++) {
                Translation2d modulePositionFromChassis = swervePositions[i].rotateBy(heading).plus(robotPose.getTranslation());

                // Module's heading is it's angle relative to the chassis heading
                modulePoses[i] = new Pose2d(modulePositionFromChassis,
                    states[i].angle.plus(robotPose.getRotation()));
            }
            field.getObject("Swerve Modules").setPoses(modulePoses);
        }
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(pose, drive.getHeading());
    }

    public Pose2d getPose() {
        return robotPose;
    }
}
