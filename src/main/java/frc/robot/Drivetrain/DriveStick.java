package frc.robot.Drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
        double speed = -cont.getRawAxis(1);           //get the speed based on left stick up/down
        double turn = -cont.getRawAxis(4) * 0.7;      //get the turn based on right stick left/right
        drive.arcadeDrive(speed, turn);               //command the drivetrain based on our inputs
    }

    @Override
    public boolean isFinished() {
        //never end
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
