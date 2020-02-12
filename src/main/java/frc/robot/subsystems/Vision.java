/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 *
 */
public class Vision extends Subsystem {
	private NetworkTable table;
	private NetworkTableEntry tx;
	private NetworkTableEntry ty;
	private NetworkTableEntry ta;
	private NetworkTableEntry camMode;
	private NetworkTableEntry pipeline;

	public static boolean upper;

	public Vision() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		tx = table.getEntry("tx");
		ty = table.getEntry("ty");
		ta = table.getEntry("ta");
		camMode = table.getEntry("camMode");
		pipeline = table.getEntry("pipeline");
		setPipeline(2);
		setCamMode(0);
	}

	public void initDefaultCommand() {
	}

	public double getCamMode() {
		return camMode.getDouble(0);
	}

	public void setCamMode(double camMode) {
		this.camMode.setDouble(camMode);
	}

	public double getPipeline() {
		return pipeline.getDouble(0);
	}

	public void setPipeline(double pipeline) {
		this.pipeline.setDouble(pipeline);
	}

	public double getDistance() {
		double distance = 0.0;

		// 1.5 = 38.4
		// 2.5 = 3.34
		// 3.5 = 1.61
		// 4.5 = .908
		// 5.5 = .612
		// 6.5 = .462
		// 7.5 = .365
		// 8.5 = .257

		// LinReg() = -.116 * ta + 5.67
		// distance = -.116 * getArea() + 5.67;

		// ExpReg() = 5.34 * 0.966 ^ ta
		// distance = 5.34 * Math.pow(0.966, getArea());

		// PowReg() = 4.73 * ta ^ -.340
		 distance = 4.73 * Math.pow(getArea(), -.340);

		// QuadReg() = 0.0447 * ta * ta + -1.87 * ta + 7.43
		// distance = 0.0447 * getArea() * getArea() + -1.87 * getArea() + 7.43;

		// LnReg() = 5.30 + -1.37 * ln(ta)
		// distance = 5.30 + -1.37 * Math.log(getArea());

        // distance *= 12;4
        //distance = (31.5-21.5)/Math.tan(Math.toRadians(getY()));
        // distance = distance/Math.cos(Math.toRadians(getX()));
		return distance;
	}

	public double getArea() {
		return ta.getDouble(0.0);
	}

	public double getX() {
		return tx.getDouble(0.0);
	}

	public double getY() {
		return ty.getDouble(0.0);
	}

	public void setLED(double mode) {
		table.getEntry("ledMode").setDouble(mode);
	}

	public void stop() {
		setPipeline(2);
		setCamMode(0);
	}

	public void reset() {
		setPipeline(2);
		setCamMode(0);
	}
}