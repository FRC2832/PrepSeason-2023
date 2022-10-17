package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Make a drivetrain subsystem for our robot
 */
public class Drivetrain extends SubsystemBase {
    // robot parts
    private PigeonIMU pigeon;
    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;

    //16384 counts = 1G, 1G = 9.806m/s^2
    private final double PIGEON_RAW_TO_MPS = 9.806/16384;
    private double[] ypr_deg = new double[3];
    private double[] xyz_mps = new double[3];
    private short[] ba_xyz = new short[3]; 
    private Rotation2d heading = new Rotation2d();
    private SwerveModuleState[] states = new SwerveModuleState[4];

    /**
     * Initialize the Drivetrain components
     * @param sim Simulation object to fill
     */
    public Drivetrain() {
        //call the register function from SubsystemBase
        super();

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        SwerveModule frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withPosition(2, 0)
                        .withSize(2, 4),
                Constants.SWERVE_GEAR_SET,
                Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT,
                Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET
        );
        SwerveModule frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withPosition(4, 0)
                        .withSize(2, 4),
                Constants.SWERVE_GEAR_SET,
                Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT,
                Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET
        );
        SwerveModule backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withPosition(6, 0)
                        .withSize(2, 4),
                Constants.SWERVE_GEAR_SET,
                Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT,
                Constants.DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET
        );
        SwerveModule backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withPosition(8, 0)
                        .withSize(2, 4),
                Constants.SWERVE_GEAR_SET,
                Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR,
                Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR,
                Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT,
                Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET
        );

        modules = new SwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};
        
        kinematics = new SwerveDriveKinematics(
            Constants.SWERVE_FRONT_LEFT_LOCATION, 
            Constants.SWERVE_FRONT_RIGHT_LOCATION,
            Constants.SWERVE_BACK_LEFT_LOCATION,
            Constants.SWERVE_BACK_RIGHT_LOCATION);
        
        for(int i=0; i< states.length; i++) {
            states[i] = new SwerveModuleState();
        }

        if(Robot.isReal()) {
            pigeon = new PigeonIMU(0);
        } else {
        }
    }

    @Override
    public void periodic() {
        //read all sensors
        if(Robot.isReal()) {
            pigeon.getBiasedAccelerometer(ba_xyz);
            for(int i=0; i<ba_xyz.length; i++) {
                xyz_mps[i] = ba_xyz[i] * PIGEON_RAW_TO_MPS;
            }
            pigeon.getYawPitchRoll(ypr_deg);
        } else {
            ypr_deg = Robot.sim.getPigeonYpr();
            xyz_mps = Robot.sim.getPigeonXyz();
        }

        //calculate positions for the rest of the loop
        // TODO: can we get rid of this allocation
        heading = new Rotation2d(Math.toRadians(ypr_deg[0]));
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void swerveDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // ask the kinematics to determine our swerve command
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }
        
        //calculate the states from the speeds
        states = kinematics.toSwerveModuleStates(speeds);
        // sometime the Kinematics spits out too fast of speeds, so this will fix this
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_DRIVETRAIN_SPEED);

        // command each swerve module
        for (int i = 0; i < modules.length; i++) {
            modules[i].set(states[i].speedMetersPerSecond / Constants.MAX_DRIVETRAIN_SPEED * Constants.NOM_BATTERY_VOLTAGE, states[i].angle.getRadians());
        }
    }

    /**
     * Get drivetrain angle.  Will keep incrementing past a circle (360 to 361*).
     * @return Angle the drivetrain is facing in degrees
     */
    public double getAngle() {
        return ypr_deg[0];
    }

    public Rotation2d getHeading() {        
        return heading;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModuleState[] getSwerveStates() {
        return states;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Subsystem");
  
      builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
      builder.addStringProperty(".default", () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none", null);
      builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
      builder.addStringProperty(".command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none", null);
      builder.addDoubleProperty(".yaw", ()-> ypr_deg[0], null);
      builder.addDoubleProperty(".pitch", ()-> ypr_deg[1], null);
      builder.addDoubleProperty(".roll", ()-> ypr_deg[2], null);
      builder.addDoubleProperty(".aX", ()-> xyz_mps[0], null);
      builder.addDoubleProperty(".aY", ()-> xyz_mps[1], null);
      builder.addDoubleProperty(".aZ", ()-> xyz_mps[2], null);
    }
}
