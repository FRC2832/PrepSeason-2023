// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedNetworkTables;
import org.littletonrobotics.junction.io.ByteLogReceiver;
import org.littletonrobotics.junction.io.LogSocketServer;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

    // robot parts
    private XboxController driverCont;
    private CommandScheduler schedule;

    // robot features
    public static Simulate sim;
    private Drivetrain drive;
    private Odometry odometry;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        setUseTiming(true); // Run as fast as possible during replay
        LoggedNetworkTables.getInstance().addTable("/SmartDashboard"); // Log & replay "SmartDashboard" values (no tables are logged by default).
        LoggedNetworkTables.getInstance().addTable("/LiveWindow");
        LoggedNetworkTables.getInstance().addTable("/Shuffleboard");
        Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        if (isReal()) {
            Logger.getInstance().addDataReceiver(new ByteLogReceiver("/media/sda1/")); // Log to USB stick (name will be selected automatically)
            Logger.getInstance().addDataReceiver(new LogSocketServer(5800)); // Provide log data over the network, viewable in Advantage Scope.
        } else {
            String path = Filesystem.getOperatingDirectory().getAbsolutePath().replace('\\', '/'); // Prompt the user for a file path on the command line
            //Logger.getInstance().setReplaySource(new ByteLogReplay(path)); // Read log file for replay
            Logger.getInstance().addDataReceiver(new ByteLogReceiver(path)); // Save replay results to a new log
        }
        Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        // initialize robot parts and locations where they are
        driverCont = new XboxController(0);         //XboxController plugged into Joystick port 0 on the driver station

        // initialize robot features
        schedule = CommandScheduler.getInstance();
        drive = new Drivetrain();
        
        //subsystems that we don't need to save the reference to, calling new schedules them
        odometry = new Odometry(drive);
        odometry.resetPose(Constants.START_POS);

        //set the default commands to run
        drive.setDefaultCommand(new DriveStick(drive, driverCont));
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     */
    @Override
    public void robotPeriodic() {
        //run the command schedule no matter what mode we are in
        schedule.run();
    }

    /** This function is called once when autonomous is enabled. */
    @Override
    public void autonomousInit() {
        //reset the schedule when auto starts to run the sequence we want
        schedule.cancelAll();

        //make a command that combines our sequence together
        SequentialCommandGroup commands = new SequentialCommandGroup(
            //drive forward 2 sec, turn right, forward 2 sec, left, drive 1 sec
            new DriveTimed(drive, 2),
            new WaitCommand(1.5),
            new DriveTimed(drive, 2)
        );

        //schedule this command for our autonomous
        //schedule.schedule(commands);
        odometry.resetPose(Constants.START_POS);
        DriveToPoint driveToPoint = new DriveToPoint(drive, odometry, Constants.START_POS);
        SmartDashboard.putData(driveToPoint);
        schedule.schedule(driveToPoint);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        //stop all autonomous commands when teleop starts
        //the default commands should take over
        schedule.cancelAll();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /* Where to initialize simulation objects */
    @Override
    public void simulationInit() {
        sim = new Simulate(drive);
        sim.Init();
    }

    /* where to map simulation physics, like drive commands to encoder counts */
    @Override
    public void simulationPeriodic() {
        sim.Periodic();
    }
}
