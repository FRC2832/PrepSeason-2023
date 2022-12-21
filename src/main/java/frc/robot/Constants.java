package frc.robot;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static final int NUM_WHEELS = 4;

    public static final int PIGEON_IMU_ID = 50;
    public static final double LOOP_TIME = 0.02;

    //falcon 500 CAN ids for Drive Motors
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 14;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 3;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 8;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 9;

    //falcon 500 CAN ids for Turn Motors
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 4;
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 1;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 10;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 7;

    //CanCoder CAN ids for swerve corners
    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 15;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 14;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 16;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 17;

    //CanCoder zero locations
    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(258.39-180.0);
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(315.09-180.0);
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(122.52+180.0);
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(222.01-180.0);

    //Swerve corner locations for kinematics
    public static final Translation2d SWERVE_FRONT_LEFT_LOCATION = new Translation2d(0.261, 0.261);
    public static final Translation2d SWERVE_FRONT_RIGHT_LOCATION = new Translation2d(0.261, -0.261);
    public static final Translation2d SWERVE_BACK_LEFT_LOCATION = new Translation2d(-0.261, 0.261);
    public static final Translation2d SWERVE_BACK_RIGHT_LOCATION = new Translation2d(-0.261, -0.261);

    public static final GearRatio SWERVE_GEAR_SET = Mk4iSwerveModuleHelper.GearRatio.L2;  //16.3 ft/s
    public static final double MAX_DRIVETRAIN_SPEED = 4.96;         //max meters per second the swerve modules can go
    public static final double MAX_DRIVETRAIN_OMEGA = 3 * Math.PI;  //max Radians per Second the robot can spin
    public static final double NOM_BATTERY_VOLTAGE = 12.5;

    public static final double MAX_DRIVER_SPEED = 3;                //Max speed (meters/sed) the driver can go
    public static final double MAX_DRIVER_OMEGA = 1.5 * Math.PI;    //Max angle (rad/sec) the driver can go
    public static final double STICK_DEADBAND = 0.13;               //how much of the sticks from the driver should we remove

    public static final Pose2d START_POS = new Pose2d(3.186,6.072,Rotation2d.fromDegrees(-135));  //where does the robot start at?
}
