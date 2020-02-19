package frc.robot.commands.chassis;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

public class LimelightTurnToAngle extends Command {
    private Timer timer;
    private PIDController pid;
    private double angle;

    // Robot.m_limelight.getX(); // < - use this

    public LimelightTurnToAngle(double angle) {

        timer = new Timer();
        timer.start();
        this.angle = angle;

        pid = new PIDController(5e-5, 1e-6, 0);
        pid.setSetpoint(angle);
        pid.setTolerance(0.1f);
        requires(Robot.m_drivetrain);
    }

    public void execute() {

        double val = pid.calculate(Robot.m_limelight.getX(), angle);
        Robot.m_drivetrain.arcadeDrive(0.0, val);
        if (Robot.m_drivetrain.leftMotorA.getEncoder().getVelocity() > 0.4
                || Robot.m_drivetrain.rightMotorA.getEncoder().getVelocity() > 0.4) {
            timer.reset();
            timer.start();
        }
    }

    public void initialize() {

        pid.reset();
    }

    public boolean isFinished() {
        return timer.get() > 1;
    }

    public void end() {
        Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
    }

    public void interrupted() {
        Robot.m_drivetrain.arcadeDrive(0.0, 0.0);
    }
}