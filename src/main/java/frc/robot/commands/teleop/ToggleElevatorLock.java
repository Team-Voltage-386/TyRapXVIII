package frc.robot.commands.teleop;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Lock the elevator in place.
 */
public class ToggleElevatorLock extends InstantCommand {

    public ToggleElevatorLock() {
        super();
        requires(Robot.elevatorSubsystem);
    }

    // Called once when the command executes
    protected void initialize() {
        Robot.elevatorSubsystem.toggleElevatorLock();
    }

}
