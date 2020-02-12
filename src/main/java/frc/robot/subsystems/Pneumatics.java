package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**

 *

 */

public class Pneumatics extends Subsystem {

	// Put methods for controlling this subsystem

	// here. Call these from Commands.

	public Compressor compressor;

	public DoubleSolenoid clamperAA;
	public Solenoid clamperBA;
	public Solenoid clamperBB;
	public DoubleSolenoid bikebrakePiston;
	public DoubleSolenoid umbrellaPiston;

	public Pneumatics() {
		compressor = new Compressor(RobotMap.compressor);
		clamperAA = new DoubleSolenoid(0, 1);
		clamperBA = new Solenoid(7);
		clamperBB = new Solenoid(6);
		bikebrakePiston = new DoubleSolenoid(4, 5);
		umbrellaPiston = new DoubleSolenoid(2, 3);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new CompressorControl());
	}

	public DoubleSolenoid.Value getClamperAAState() {
		return clamperAA.get();
	}
	public boolean getClamperBAState() {
		return clamperBA.get();
	}
	public DoubleSolenoid.Value getBikebrakeState() {
		return bikebrakePiston.get();
		// return RobotMap.PistonIn;
	}
	public DoubleSolenoid.Value getUmbrellaState() {
		return umbrellaPiston.get();
	}

	public void setClamperAAState(DoubleSolenoid.Value state) {
		clamperAA.set(state);
	}
	public void setClamperBAState(boolean state) {
		clamperBA.set(state);
	
	}
	public void setClamperBBState(boolean state) {
		clamperBB.set(state);
	
	}
	public void setBikebrakeState(DoubleSolenoid.Value state) {
		bikebrakePiston.set(state);
	}
	public void setUmbrellaState(DoubleSolenoid.Value state) {
		umbrellaPiston.set(state);
	}
}