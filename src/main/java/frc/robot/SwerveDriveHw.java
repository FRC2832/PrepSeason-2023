package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.interfaces.ISwerveDrive;
import frc.robot.interfaces.ISwerveDriveIo;

public class SwerveDriveHw implements ISwerveDriveIo {
    //motors and sensors
    private TalonFX turnMotor[];
    private TalonFX driveMotor[];
    private CANCoder absSensor[];
    private PigeonIMU pigeon;
    
    //sensor value buffers
    private double ypr_deg[];
    private double absSensorValue[];
    private double driveWheelVelocity[];
    private double turnMotorAngle[];

    public SwerveDriveHw() {
        //initialize array sizes
        turnMotor = new TalonFX[Constants.NUM_WHEELS];
        driveMotor = new TalonFX[Constants.NUM_WHEELS];
        absSensor = new CANCoder[Constants.NUM_WHEELS];
        
        //initialize each motor/sensor
        turnMotor[ISwerveDrive.FL] = new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR);
        turnMotor[ISwerveDrive.FR] = new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR);
        turnMotor[ISwerveDrive.RL] = new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR);
        turnMotor[ISwerveDrive.RR] = new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR);

        driveMotor[ISwerveDrive.FL] = new TalonFX(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR);
        driveMotor[ISwerveDrive.FR] = new TalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR);
        driveMotor[ISwerveDrive.RL] = new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR);
        driveMotor[ISwerveDrive.RR] = new TalonFX(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR);

        absSensor[ISwerveDrive.FL] = new CANCoder(Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_PORT);
        absSensor[ISwerveDrive.FR] = new CANCoder(Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT);
        absSensor[ISwerveDrive.RL] = new CANCoder(Constants.DRIVETRAIN_BACK_LEFT_ENCODER_PORT);
        absSensor[ISwerveDrive.RR] = new CANCoder(Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_PORT);

        pigeon = new PigeonIMU(Constants.PIGEON_IMU_ID);
        
        //initialize sensor buffers
        ypr_deg = new double[3];
        absSensorValue = new double[Constants.NUM_WHEELS];
    }

    @Override
    public void updateInputs() {
        pigeon.getYawPitchRoll(ypr_deg);

        for(int i=0; i<Constants.NUM_WHEELS; i++) {
            absSensorValue[i] = absSensor[i].getAbsolutePosition();
            driveWheelVelocity[i] = driveMotor[i].getSelectedSensorVelocity();
            turnMotorAngle[i] = turnMotor[i].getSelectedSensorPosition();
        }
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(ypr_deg[0]);
    }

    @Override
    public void setTurnMotorBrakeMode(boolean brakeOn) {
        NeutralMode mode;

        if(brakeOn) {
            mode = NeutralMode.Brake;
        } else {
            mode = NeutralMode.Coast;
        }

        for (TalonFX motor : turnMotor) {
            motor.setNeutralMode(mode);
        }
    }

    @Override
    public void setDriveMotorBrakeMode(boolean brakeOn) {
        NeutralMode mode;

        if(brakeOn) {
            mode = NeutralMode.Brake;
        } else {
            mode = NeutralMode.Coast;
        }
        
        for (TalonFX motor : driveMotor) {
            motor.setNeutralMode(mode);
        }
    }

    @Override
    public double getCornerAbsAngle(int wheel) {
        return absSensorValue[wheel];
    }

    @Override
    public double getCornerAngle(int wheel) {
        return turnMotorAngle[wheel];
    }

    @Override
    public double getCornerSpeed(int wheel) {
        return driveWheelVelocity[wheel];
    }

    @Override
    public void setCornerState(int wheel, SwerveModuleState swerveModuleState) {
        driveMotor[wheel].set(ControlMode.Velocity, swerveModuleState.speedMetersPerSecond);
        turnMotor[wheel].set(ControlMode.Position, swerveModuleState.angle.getDegrees());
    }
    
}
