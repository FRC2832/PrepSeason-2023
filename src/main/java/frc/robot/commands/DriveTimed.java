package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Drivetrain;

/**
 * Drive the robot at 50% power
 */
public class DriveTimed extends CommandBase {
    private Drivetrain drive;
    private Timer timer;
    private double stopTime;

    /**
     * Drive the robot at 50% power for stopTime time.
     * @param drive Drivetrain subsystem to command
     * @param stopTime Time to drive
     */
    public DriveTimed(Drivetrain drive, double stopTime) {
        //copy inputs to the command
        this.drive = drive;
        this.stopTime = stopTime;

        //add this command to the drivetrain subsystem
        addRequirements(drive);

        //initialize the timer
        timer = new Timer();
    }

    @Override
    public void initialize() {
        //start the timer when the command starts
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        //command 50% straight
        drive.swerveDrive(0.5 * Constants.MAX_DRIVETRAIN_SPEED, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        //check if the timer has finished
        return timer.hasElapsed(stopTime);
    }

    @Override
    public void end(boolean interrupted) {
        //when stopped, stop the drivetrain
        drive.swerveDrive(0, 0, 0, false);
    }
}
