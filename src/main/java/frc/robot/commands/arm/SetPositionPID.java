/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;

import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SetPositionPID extends Command {
	public SetPositionPID() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.arm);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		//setTimeout(1.2); 
		// if(Robot.arm.hawkTalonA.getSelectedSensorPosition() > -200) {
		// 	Robot.arm.hawkTalonA.setSelectedSensorPosition(0);
		// // }
		// Robot.pneumatics.setBikebrakeState(RobotMap.PistonIn);
		// Robot.arm.hawkTalonA.configMotionAcceleration(500);
		// Robot.arm.hawkTalonA.configMotionCruiseVelocity(500);
		// Robot.arm.hawkTalonA.setSensorPhase(true);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
        Robot.arm.hawkTalonA.set(ControlMode.Position, -400);
        if(Robot.arm.hawkTalonA.getSelectedSensorPosition() < -400) {
            OI.driverController.setRumble(RumbleType.kLeftRumble, 0.5);
        }
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
	    return false; //|| isTimedOut();
	}

	// Called once after isFinished returns true
	protected void end() {
		// Robot.pneumatics.setBikebrakeState(RobotMap.PistonOut);
		Robot.arm.hawkTalonA.set(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		// Robot.pneumatics.setBikebrakeState(RobotMap.PistonOut);
		Robot.arm.hawkTalonA.set(0);
	}
}