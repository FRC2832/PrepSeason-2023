package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Drivetrain;
import frc.robot.Odometry;

/**
 * Drive the robot to a specific point on the field 
 */
public class DriveToPoint extends CommandBase {
    private Drivetrain drive;
    private Odometry odometry;
    private Pose2d dest;
    private final double TARGET_ERROR = Units.feetToMeters(0.25);
    private boolean isFinished;

    public DriveToPoint(Drivetrain drive, Odometry odometry, Pose2d dest) {
        this.drive = drive;
        this.odometry = odometry;
        this.dest = dest;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        isFinished = false;

        //test code!!!  Pick a random point to go to
        Pose2d[] points = {
            new Pose2d(2.25, 5.03, Rotation2d.fromDegrees(-135)),
            new Pose2d(5.97, 10.97, Rotation2d.fromDegrees(45)),
            new Pose2d(2.25, 10.97, Rotation2d.fromDegrees(135)),
            new Pose2d(5.97, 5.03, Rotation2d.fromDegrees(-45)),
        };
        dest = points[(int)(Math.random()*points.length)];
    }

    @Override
    public void execute() {
        Pose2d currentPose = odometry.getPose();
        double distX = currentPose.getX() - dest.getX();
        double distY = currentPose.getY() - dest.getY();
        double distLeft = Math.sqrt((distX * distX) + (distY * distY));

        if(distLeft > TARGET_ERROR) {
            //since we know the dist left, we can scale the speeds based on max distance
            //formula (max speed) / (delta speed) = (distLeft) / (distx/y)
            double scale = Constants.MAX_DRIVETRAIN_SPEED / distLeft;
            drive.swerveDrive(
                distX  * scale, 
                distY  * scale, 
                0, //TODO: Fix Me!
                false);
        } else {
            //we are at our spot, stop
            drive.swerveDrive(0, 0, 0, false);
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        //test code!!!  make the commands keep repeating
        Command newCommand = new SequentialCommandGroup(new WaitCommand(2),new DriveToPoint(drive, odometry, dest));
        CommandScheduler.getInstance().schedule(newCommand);
    }
}

