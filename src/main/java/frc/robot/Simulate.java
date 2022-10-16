package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Simulate {
    // robot parts
    private TimedRobot robot;
    private ADXRS450_Gyro gyro;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private XboxController cont;

    // kinetic parts
    private Field2d field;
    private DifferentialDriveOdometry odometry;

    private DifferentialDrivetrainSim driveSim;
    private ADXRS450_GyroSim gyroSim;
    private EncoderSim leftEncoderSim;
    private EncoderSim rightEncoderSim;
    private NetworkTableEntry leftSpeedEntry;
    private NetworkTableEntry rightSpeedEntry;

    public Simulate(TimedRobot robot, XboxController cont) {
        this.robot = robot;
        this.cont = cont;
    }

    public void Init() {
        field = new Field2d();
        SmartDashboard.putData("Field", field);
        SmartDashboard.putBoolean("Reset Position", false);
        leftSpeedEntry = NetworkTableInstance.getDefault()
                .getEntry("/LiveWindow/Ungrouped/DifferentialDrive[1]/Left Motor Speed");
        rightSpeedEntry = NetworkTableInstance.getDefault()
                .getEntry("/LiveWindow/Ungrouped/DifferentialDrive[1]/Right Motor Speed");

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), new Pose2d(0.0, 0, new Rotation2d()));
        resetRobotPosition();

        //map the simulated xbox controller
        new XboxControllerSim(cont);
        driveSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDualCIMPerSide, KitbotGearing.k10p71,
                KitbotWheelSize.kEightInch, null);
    }

    public void RegisterDrivetrain(ADXRS450_Gyro gyro, Encoder leftEncoder, Encoder rightEncoder) {
        this.gyro = gyro;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        
        gyroSim = new ADXRS450_GyroSim(gyro);
        leftEncoderSim = new EncoderSim(leftEncoder);
        rightEncoderSim = new EncoderSim(rightEncoder);
    }

    public void Periodic() {
        // Get my gyro angle
        var gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());

        // Update the pose
        odometry.update(gyroAngle, leftEncoder.getDistance(), rightEncoder.getDistance());
        field.setRobotPose(odometry.getPoseMeters());

        // if the driver requests to reset position, do it
        if (SmartDashboard.getBoolean("Reset Position", false) == true) {
            resetRobotPosition();
            SmartDashboard.putBoolean("Reset Position", false);
        }

        final double RATE = 0.02; // rate we run at, 20ms loop time
        var volt = RobotController.getBatteryVoltage();
        double last;

        //only update the robot position if we are enabled
        if(robot.isEnabled()) {
            // run the drivetrain
            var leftPct = leftSpeedEntry.getNumber(0).doubleValue();
            var rightPct = rightSpeedEntry.getNumber(0).doubleValue();
            driveSim.setInputs(leftPct * volt, rightPct * volt);
            driveSim.update(RATE);
        }

        // set the gyro based on drivetrain
        last = gyro.getAngle();
        gyroSim.setAngle(driveSim.getHeading().getDegrees());
        gyroSim.setRate((gyro.getAngle() - last) / RATE);

        // set the left encoder based on drivetrain
        leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    }

    // in case the simulation goes crazy, allow the driver to reset the position of
    // the robot back to start
    private void resetRobotPosition() {
        leftEncoder.reset();
        rightEncoder.reset();
        odometry.resetPosition(new Pose2d(0.4, 6.1, gyro.getRotation2d()), gyro.getRotation2d());
        if (Robot.isSimulation()) {
            // reset the drivetrain
            if (driveSim != null) {
                driveSim.setPose(new Pose2d(0, 6, new Rotation2d()));
            }
        }
    }
}
