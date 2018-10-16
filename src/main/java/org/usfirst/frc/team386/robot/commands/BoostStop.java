package org.usfirst.frc.team386.robot.commands;

import org.usfirst.frc.team386.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Stop the speed boost.
 */
public class BoostStop extends InstantCommand {

    public BoostStop() {
	super();
	// Use requires() here to declare subsystem dependencies
	requires(Robot.driveSubsystem);
    }

    // Called once when the command executes
    protected void initialize() {
	Robot.driveSubsystem.stopBoost();
    }

}
