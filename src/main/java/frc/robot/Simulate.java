package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Simulate {
    // robot parts
    private Drivetrain drive;
    private SwerveDriveKinematics kinematics;

    // gyro simulation
    private double[] ypr_deg = new double[3];
    private double[] xyz_mps = new double[3];

    public Simulate(Drivetrain drive) {
        this.drive = drive;

        kinematics = drive.getKinematics();
    }

    public void Init() {

    }


    public void Periodic() {
        updateDriveTrain();
    }

    private void updateDriveTrain() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(drive.getSwerveStates());
        
        ypr_deg[0] += Math.toDegrees(speeds.omegaRadiansPerSecond * Robot.kDefaultPeriod);
        xyz_mps[0] = speeds.vxMetersPerSecond / Robot.kDefaultPeriod;
        xyz_mps[1] = speeds.vyMetersPerSecond / Robot.kDefaultPeriod;
    }

    public double[] getPigeonYpr() {
        return ypr_deg;
    }

    public double[] getPigeonXyz() {
        return xyz_mps;
    }
}
