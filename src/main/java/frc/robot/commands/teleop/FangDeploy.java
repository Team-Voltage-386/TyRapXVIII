package frc.robot.commands.teleop;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class FangDeploy extends TimedCommand {

    public FangDeploy() {
        super(1.0);
    }

    @Override
    protected void end() {
        Robot.elevatorSubsystem.toggleElevatorLock();
    }

}
