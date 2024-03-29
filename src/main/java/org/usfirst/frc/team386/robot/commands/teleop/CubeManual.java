package org.usfirst.frc.team386.robot.commands.teleop;

import org.usfirst.frc.team386.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Use the left and right xBox joystick controls to manipulate the cube
 * subsystem.
 */
public class CubeManual extends Command {

    public CubeManual() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.cubeSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.cubeSubsystem.run(-1 * Robot.oi.manipulator.getRawAxis(1), Robot.oi.manipulator.getRawAxis(3));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
