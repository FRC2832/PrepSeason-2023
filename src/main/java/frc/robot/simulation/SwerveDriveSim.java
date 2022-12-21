package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.interfaces.ISwerveDriveIo;

public class SwerveDriveSim implements ISwerveDriveIo {
    private SwerveModuleState swerveStates[];
    private SwerveDriveKinematics kinematics;
    private double chassisAngle;

    private double absAngle[];
    private double turnAngle[];
    private double driveSpeed[];

    public SwerveDriveSim() {
        swerveStates = new SwerveModuleState[Constants.NUM_WHEELS];
        absAngle = new double[Constants.NUM_WHEELS];
        turnAngle = new double[Constants.NUM_WHEELS];
        driveSpeed = new double[Constants.NUM_WHEELS];
        for(int i=0; i<swerveStates.length; i++) {
            swerveStates[i] = new SwerveModuleState();
        }
        chassisAngle = 0;
    }

    @Override
    public void setKinematics(SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
    }
    
    @Override
    public void updateInputs() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(swerveStates);
        
        chassisAngle += Math.toDegrees(speeds.omegaRadiansPerSecond * Constants.LOOP_TIME);
        
        //xyz_mps[0] = speeds.vxMetersPerSecond / Constants.LOOP_TIME;
        //xyz_mps[1] = speeds.vyMetersPerSecond / Constants.LOOP_TIME;

        //TODO: Simulate the actual swerve corners... https://www.chiefdelphi.com/t/sysid-gains-on-sds-mk4i-modules/400373/7
        for(int i=0; i<swerveStates.length; i++) {
            driveSpeed[i] = swerveStates[i].speedMetersPerSecond;
            turnAngle[i] = swerveStates[i].angle.getDegrees();
            absAngle[i] = turnAngle[i];
        }
        
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(chassisAngle);
    }

    @Override
    public void setTurnMotorBrakeMode(boolean brakeOn) {
        // Not a Sim function
    }

    @Override
    public void setDriveMotorBrakeMode(boolean brakeOn) {
        // Not a Sim function
    }

    @Override
    public double getCornerAbsAngle(int wheel) {
        return absAngle[wheel];
    }

    @Override
    public double getCornerAngle(int wheel) {
        return turnAngle[wheel];
    }

    @Override
    public double getCornerSpeed(int wheel) {
        return driveSpeed[wheel];
    }

    @Override
    public void setCornerState(int wheel, SwerveModuleState swerveModuleState) {
        swerveStates[wheel] = swerveModuleState;
    }
    
}
