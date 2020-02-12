/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.climber.SetSpeedClimber;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // public CANSparkMax sparkMaxC;
  // public CANSparkMax sparkMaxD;
  // public CANSparkMax sparkMaxF; //Unspooler

  // public Servo servoA = new Servo(RobotMap.dioServoPort1);
  // public Servo servoB = new Servo(RobotMap.dioServoPort2);
	
	public double servoStart = 0.0;


  public Climber() {
    // sparkMaxC = new CANSparkMax(RobotMap.sparkMaxC, MotorType.kBrushless);
    // sparkMaxD = new CANSparkMax(RobotMap.sparkMaxD, MotorType.kBrushless);
    // sparkMaxF = new CANSparkMax(RobotMap.sparkMaxF, MotorType.kBrushed);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new SetSpeedClimber());
    // setServos(servoStart);
  }
  public void setSpeedA(double speed) {
      // sparkMaxC.set(speed);
  }
  public void setSpeedBoth(double speed) {
      // sparkMaxC.set(speed);
      // sparkMaxD.set(speed);
  }
  public void spool(double speed) {
      // sparkMaxF.set(speed);
  }
  // public void setServos(double value) {
  //   servoA.set(value);
  //   servoB.set(value);
	// }
}
