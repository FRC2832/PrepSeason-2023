package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.ISwerveDrive;
import frc.robot.interfaces.ISwerveDriveIo;

public class SwerveDriveTrain implements ISwerveDrive {
    private SwerveDriveKinematics kinematics;
    private ISwerveDriveIo hardware;
    private SwerveModuleState[] swerveStates;
    private Pose2d robotPose;
    private String moduleNames[];

    public SwerveDriveTrain(ISwerveDriveIo hSwerveDriveIo) {
        this.hardware = hSwerveDriveIo;

        //initialize the corner locations
        kinematics = new SwerveDriveKinematics(
            Constants.SWERVE_FRONT_LEFT_LOCATION,
            Constants.SWERVE_FRONT_RIGHT_LOCATION,
            Constants.SWERVE_BACK_LEFT_LOCATION,
            Constants.SWERVE_BACK_RIGHT_LOCATION);
        hardware.setKinematics(kinematics);
        
        //initialize the swerve states
        swerveStates = new SwerveModuleState[Constants.NUM_WHEELS];
        for(int wheel = 0; wheel < Constants.NUM_WHEELS; wheel++) {
            swerveStates[wheel] = new SwerveModuleState();
        }

        //initialize module names
        moduleNames = new String[Constants.NUM_WHEELS];
        moduleNames[FL] = "Module FL/";
        moduleNames[FR] = "Module FR/";
        moduleNames[RL] = "Module RL/";
        moduleNames[RR] = "Module RR/";
    }
    
    @Override
    public void periodic() {
        hardware.updateInputs();

        //read the swerve corner state
        for(int wheel = 0; wheel < Constants.NUM_WHEELS; wheel++) {
            swerveStates[wheel].speedMetersPerSecond = hardware.getCornerSpeed(wheel);
            swerveStates[wheel].angle = Rotation2d.fromDegrees(hardware.getCornerAbsAngle(wheel));
        }

        //display data on SmartDashboard
        SmartDashboard.putNumber("Gyro Angle", getHeading().getDegrees());
        for(int wheel=0; wheel < Constants.NUM_WHEELS; wheel++) {
            SmartDashboard.putNumber(moduleNames[wheel] + "Abs Sensor", hardware.getCornerAbsAngle(wheel));
            SmartDashboard.putNumber(moduleNames[wheel] + "Turn Sensor", hardware.getCornerAngle(wheel));
            SmartDashboard.putNumber(moduleNames[wheel] + "Drive Speed Sensor", hardware.getCornerSpeed(wheel));
            SmartDashboard.putNumber(moduleNames[wheel] + "Calc Angle", swerveStates[wheel].angle.getDegrees());
        }
    }

    @Override
    public void SwerveDrive(double xSpeed, double ySpeed, double turn, boolean fieldOriented) {
        // ask the kinematics to determine our swerve command
        ChassisSpeeds speeds;
        if (fieldOriented) {
            //90* is needed since we view the field on a 90* rotation
            var angle = robotPose.getRotation().minus(Rotation2d.fromDegrees(90));
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turn, angle);
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, turn);
        }
        
        //calculate the states from the speeds
        SwerveModuleState[] requestStates = kinematics.toSwerveModuleStates(speeds);
        // sometime the Kinematics spits out too fast of speeds, so this will fix this
        SwerveDriveKinematics.desaturateWheelSpeeds(requestStates, Constants.MAX_DRIVETRAIN_SPEED);

        // command each swerve module
        for (int i = 0; i < requestStates.length; i++) {
            hardware.setCornerState(i, requestStates[i]);
        }
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public Rotation2d getHeading() {
        return hardware.getHeading();
    }

    @Override
    public SwerveModuleState[] getSwerveStates() {
        return swerveStates;
    }

    @Override
    public void setPose(Pose2d robotPose) {
        this.robotPose = robotPose;        
    }
}
