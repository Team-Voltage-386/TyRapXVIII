package org.usfirst.frc.team386.robot.commands;

import org.usfirst.frc.team386.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CubeSuckUltra extends Command {
    public static final double MOTOR_SPEED = 1;

    Timer timer = new Timer();
    double time;

    /**
     * Sucks the cube for a number of seconds or until a cube is detected, whichever
     * is first
     * 
     * @param timeIn the number of seconds
     */
    public CubeSuckUltra(double timeIn) {
        // Use requires() here to declare subsystem dependencies
        this.time = timeIn;

        requires(Robot.cubeSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        timer.start();
    }

    protected void execute() {
        Robot.cubeSubsystem.symetricalCube(MOTOR_SPEED);
    }

    @Override
    protected boolean isFinished() {
        // TODO Auto-generated method stub
        return timer.get() > time || Robot.cubeSubsystem.hasCube();
    }

    @Override
    protected void end() {
        Robot.cubeSubsystem.stop();
    }
}
