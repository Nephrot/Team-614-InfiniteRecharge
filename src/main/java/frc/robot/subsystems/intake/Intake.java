/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.serializer.Serializer;
import com.playingwithfusion.*;


public class Intake extends Subsystem {
   // Put methods for controlling this subsystem
   // here. Call these from Commands.

   // One Intake Motor, simple speed
   public CANSparkMax intakeMotor;
   public DoubleSolenoid intakeSolenoidA;
   public DoubleSolenoid intakeSolenoidB;

   DoubleSolenoid.Value valueSolenoid1 = DoubleSolenoid.Value.kReverse;
   DoubleSolenoid.Value valueSolenoid2 = DoubleSolenoid.Value.kForward;

   public Intake() {
      intakeMotor = new CANSparkMax(RobotMap.intakeMotorPort, MotorType.kBrushed);
      intakeSolenoidA = new DoubleSolenoid(RobotMap.doubleSolenoidAPort1, RobotMap.doubleSolenoidAPort2);
      intakeSolenoidB = new DoubleSolenoid(RobotMap.doubleSolenoidBPort1, RobotMap.doubleSolenoidBPort2);
   }

   @Override
   public void initDefaultCommand() {
      // Set the default command for a subsystem here.
      // setDefaultCommand(new MySpecialCommand());
      // setDefaultCommand(new SetSpeed());
   }

   public DoubleSolenoid.Value getDoubleSolenoidA() {
      return intakeSolenoidA.get();
   }

   public DoubleSolenoid.Value getDoubleSolenoidB() {
      return intakeSolenoidB.get();
   }

   public void setDoubleSolenoidA(DoubleSolenoid.Value state) {
      intakeSolenoidA.set(state);
   }

   public void setDoubleSolenoidB(DoubleSolenoid.Value state) {
      intakeSolenoidB.set(state);
   }

   public DoubleSolenoid.Value getOppositeState(DoubleSolenoid.Value solenoid) {
      if (solenoid.equals(valueSolenoid1)) {
         return valueSolenoid2;
      } else {
         return valueSolenoid1;
      }
   }

   public void toggleDoubleSolenoidA() {
      // intakeSolenoidA = getOppositeState(getDoubleSolenoidA());
      //setDoubleSolenoidA(getOppositeState(getDoubleSolenoidA()));
      setDoubleSolenoidA(DoubleSolenoid.Value.kForward);
   }

   public void toggleDoubleSolenoidB() {
      // setDoubleSolenoidB(DoubleSolenoid.Value.kForward);
    
     setDoubleSolenoidB(DoubleSolenoid.Value.kForward);
   }

   public void untoggleDoubleSolenoidA() {
      setDoubleSolenoidA(DoubleSolenoid.Value.kForward);
   }


   public void untoggleDoubleSolenoidB() {
      setDoubleSolenoidB(DoubleSolenoid.Value.kReverse);
    
     setDoubleSolenoidB(DoubleSolenoid.Value.kForward);
   }


   public void intakeMotorSpeed(double speed) {
      intakeMotor.set(speed);
   }

   public double getCurrent() {
      return intakeMotor.getOutputCurrent();
   }

   public boolean checkCurrent(double current) {
      return intakeMotor.getOutputCurrent() > current ? true : false;
   }

   public void runSerializer(Serializer serializer, double current, double speed) {
      if (checkCurrent(current)) {
         serializer.runMotorFunction(speed);
      }
   }
}