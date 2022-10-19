package frc.robot.commands;

import org.livoniawarriors.UtilFunctions;

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
        double xSpeed = -UtilFunctions.deadband(cont.getLeftY(), Constants.STICK_DEADBAND);
        double ySpeed = -UtilFunctions.deadband(cont.getLeftX(), Constants.STICK_DEADBAND);      
        double turn   = -UtilFunctions.deadband(cont.getRightX(), Constants.STICK_DEADBAND);      

        drive.swerveDrive(
            xSpeed  * Constants.MAX_DRIVER_SPEED, 
            ySpeed  * Constants.MAX_DRIVER_SPEED, 
            turn    * Constants.MAX_DRIVER_OMEGA, 
            false);
    }

    @Override
    public boolean isFinished() {
        //never end
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
