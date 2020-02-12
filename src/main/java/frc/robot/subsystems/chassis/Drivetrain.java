package frc.robot.subsystems.chassis;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.motionprofile.*;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.subsystems.chassis.HawkTalons;
import frc.robot.subsystems.chassis.SRXPID;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

public class Drivetrain extends Subsystem implements PIDOutput{
	PIDController turnController;
	private double rotateToAngle;
	public boolean turnPID;
	public static final int timeout = 10;

	public HawkTalons leftMotor1  = new HawkTalons(RobotMap.sparkMax2);
	public WPI_VictorSPX leftMotor2  = new WPI_VictorSPX(RobotMap.sparkMax1);

	public HawkTalons rightMotor1 = new HawkTalons(RobotMap.rightMotorA);
	public WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(RobotMap.rightMotorB);

	private DifferentialDrive drive = new DifferentialDrive(leftMotor1, rightMotor1);

	public static final double unitsPerInch = 1000; //No
	
		public final double distanceTolerance = 0.1f;
	public double last_world_linear_accel_x;
	public double last_world_linear_accel_y;
	public double last_world_linear_accel_z;
	public double currentJerkX;
	public double currentJerkY;
	public double currentJerkZ;

	public Drivetrain() {
		turnPID = false;
		leftMotor1.setInverted(true);
		rightMotor1.setInverted(true);
		leftMotor2.setInverted(true);
		rightMotor2.setInverted(true);
		drive.setSafetyEnabled(false);
		this.configTalons();
		leftMotor2.follow(leftMotor1);
		rightMotor2.follow(rightMotor1);
		turnController = new PIDController(RobotMap.turnP, RobotMap.turnI,
				RobotMap.turnD, RobotMap.turnF, Robot.navX, this);
		turnController.setInputRange(-180, 180);
		turnController.setOutputRange(-1, 1);
		turnController.setAbsoluteTolerance(0.1f);
		turnController.setContinuous(true);
		// leftMotor1.setInverted(true);
		// leftMotor2.setInverted(true);
	}

	public void setUsingTurnPID(boolean set) {
		turnPID = set;
		if (turnPID) {
			turnController.enable();
		} else {
			turnController.disable();
		}
	}

	// public void setUsingDistancePID(boolean set) {
	// 	Robot.drivetrainCompanion.setUsingDistancePID(set);
	// }

	public boolean getUsingTurnPID() {
		return turnPID;
	}

	// public boolean getUsingDistancePID() {
	// 	return Robot.drivetrainCompanion.getUsingDistancePID();
	// }

	public double getPIDRotateRate() {
		return rotateToAngle;
	}

	// public double getPIDSpeed() {
	// 	return Robot.drivetrainCompanion.getPIDSpeed();
	// }

	public PIDController getTurnController() {
		return turnController;
	}

	public void pidWrite(double output) {
		if (turnPID) {
			rotateToAngle = output;
		}
	}
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new ArcadeDrive());
		
	}

	public void arcadeDrive(double speed, double rotate) {
		drive.arcadeDrive(speed, rotate);
	}

	public void profileDrive(double left, double right) {
		leftMotor1.set(left);
		leftMotor2.set(left);

		rightMotor1.set(right);
		rightMotor2.set(right);
	}

	public HawkTalons getRightTalon() {
		return this.rightMotor1;
	}

	public WPI_VictorSPX getRightSlaveVictor() {
		return this.rightMotor2;
	}

	public HawkTalons getLeftTalon() {
		return this.leftMotor1;
	}

	public WPI_VictorSPX getLeftSlaveVictor() {
		return this.leftMotor2;
	}

	public void configTalons() {

		// RIGHT MOTOR CONFIG
		rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		rightMotor1.setSensorPhase(false); /* keep sensor and motor in phase */
		rightMotor1.configNeutralDeadband(0.001, timeout);
		rightMotor1.configNominalOutputForward(0, timeout);
		rightMotor1.configNominalOutputReverse(0, timeout);
		rightMotor1.configPeakOutputForward(1, timeout);
		rightMotor1.configPeakOutputReverse(-1, timeout);
        // --
		rightMotor1.setConfig(new SRXPID(RobotMap.distF, RobotMap.distP, RobotMap.distI, RobotMap.distD), 0); //Tune later on
		rightMotor1.setConfig(RobotMap.turnGains, 1);
		rightMotor1.configMotionProfileTrajectoryPeriod(10, timeout); 
		rightMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20, timeout);
		rightMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20, timeout);
		rightMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 20, timeout);
		rightMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10, timeout);
		rightMotor1.configMotionCruiseVelocity(100, timeout);
		rightMotor1.configMotionAcceleration(100, timeout);

		// LEFT MOTOR CONFIG
		leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		leftMotor1.setSensorPhase(false); /* keep sensor and motor in phase */
		leftMotor1.configNeutralDeadband(0.01, timeout);
		leftMotor1.configNominalOutputForward(0, timeout);
		leftMotor1.configNominalOutputReverse(0, timeout);
		leftMotor1.configPeakOutputForward(1, timeout);
		leftMotor1.configPeakOutputReverse(-1, timeout);
        // --
		leftMotor1.setConfig(new SRXPID(RobotMap.distF, RobotMap.distP, RobotMap.distI, RobotMap.distD), 0); //Tune later on
		leftMotor1.setConfig(RobotMap.turnGains, 1);
		leftMotor1.config_IntegralZone(0, RobotMap.distIZone);
		leftMotor1.config_IntegralZone(1, RobotMap.turnIZone);
		leftMotor1.configClosedLoopPeakOutput(0, RobotMap.distPeakOutput);
		leftMotor1.configClosedLoopPeakOutput(1, RobotMap.turnPeakOutput);
		leftMotor1.configMotionProfileTrajectoryPeriod(10, timeout); 
		leftMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, timeout);
		leftMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, timeout);
		rightMotor1.configAuxPIDPolarity(false, timeout);
	}



	public double getDistanceMoved() { //Note: right is negative as forward is the negative direction on the right side.
		return (this.leftMotor1.getSelectedSensorPosition(0) + -this.rightMotor1.getSelectedSensorPosition(0)) / 2.0; //Fix maybe?
	}

	public double getLeftPos() {
		return this.leftMotor1.getSelectedSensorPosition(0) / unitsPerInch;
	}

	public double getLeftVel() {
		return this.leftMotor1.getSelectedSensorVelocity(0) / unitsPerInch;
	}

	public double getRightPos() {
		return this.rightMotor1.getSelectedSensorPosition(0) / unitsPerInch;
	}

	public double getRightVel() {
		return this.rightMotor1.getSelectedSensorVelocity(0) / unitsPerInch;
	}

	public void zeroSensors() {
		this.rightMotor1.resetEncoder();
		this.leftMotor1.resetEncoder();
	}
	
	public void resetSpeed() {
		rightMotor1.setSpeed(0);
		leftMotor1.setSpeed(0);
	}

	public void straightDrive(double distance, double turn) {
	  rightMotor1.set(ControlMode.MotionMagic, distance, DemandType.AuxPID, turn);
	  leftMotor1.follow(leftMotor1);
      SmartDashboard.putNumber("BUS Voltage", rightMotor1.getBusVoltage());
      SmartDashboard.putNumber("Motor Output", rightMotor1.getMotorOutputPercent());
      SmartDashboard.putNumber("Motor Voltage", rightMotor1.getMotorOutputVoltage());
	}
	// public boolean isEncoderOn() {
	// 	if(this.rightMotor1.)
	// }

	public void runCollisionDetection() {
		double curr_world_linear_accel_x = Robot.navX.getWorldLinearAccelX();
		currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_y = Robot.navX.getWorldLinearAccelY();
		currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
		last_world_linear_accel_y = curr_world_linear_accel_y;
		// double curr_world_linear_accel_z = Robot.navX.getWorldLinearAccelZ();
		// currentJerkZ = curr_world_linear_accel_z - last_world_linear_accel_z;
		// last_world_linear_accel_z = curr_world_linear_accel_z;
	}

	public double getCurrentJerkX() {
		return currentJerkX;
	}

	public double getCurrentJerkY() {
		return currentJerkY;
	}

}