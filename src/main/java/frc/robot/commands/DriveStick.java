package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Drivetrain;

/**
 * Drive the robot with joysticks 
 */
public class DriveStick extends CommandBase {
    private Drivetrain drive;
    private XboxController cont;

    /**
     * Inject the drivetain and controller to use
     * @param drive Drivetrain to command
     * @param cont Controller to read from
     */
    public DriveStick(Drivetrain drive, XboxController cont) {
        this.drive = drive;
        this.cont = cont;
        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xSpeed = -cont.getLeftY() * Constants.MAX_DRIVER_SPEED;
        double ySpeed = -cont.getLeftX() * Constants.MAX_DRIVER_SPEED;      
        double turn = -cont.getRightX() * Constants.MAX_DRIVER_OMEGA;      

        drive.swerveDrive(xSpeed, ySpeed, turn, false);
    }

    @Override
    public boolean isFinished() {
        //never end
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
