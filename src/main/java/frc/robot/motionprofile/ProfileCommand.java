
package frc.robot.motionprofile;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.TrajPointReading;
import frc.robot.*;

import com.ctre.phoenix.motion.*;
import com.ctre.phoenix.motion.TrajectoryPoint;



public class ProfileCommand {
    private MotionProfileStatus _status = new MotionProfileStatus();
    
	double _pos=0;
	double _vel=0; 
	double _heading=0;

	private TalonSRX _talon;
	private int _state = 0;
	public int _loopTimeout = -1;
	private boolean _bStart = false;
	private SetValueMotionProfile _setValue = SetValueMotionProfile.Disable;
	private static final int kMinPointsInTalon = 5;
	private static final int kNumLoopsTimeout = 10;

	

	private double[][] path;

	class PeriodicRunnable implements java.lang.Runnable {
	    public void run() {  _talon.processMotionProfileBuffer();    }
	}
	Notifier _notifer = new Notifier(new PeriodicRunnable());

	public ProfileCommand(TalonSRX talon, double[][] path) {
		_talon = talon;
		_talon.changeMotionControlFramePeriod(5);
		_notifer.startPeriodic(0.005);

		this.path = path;
	}

	public void reset() {
		_talon.clearMotionProfileTrajectories();
		_setValue = SetValueMotionProfile.Disable;
		_state = 0;
		_loopTimeout = -1;
		_bStart = false;
	}

	public MotionProfileStatus control() {
		_talon.getMotionProfileStatus(_status);
		if (_loopTimeout < 0) {

		} else {
			if (_loopTimeout == 0) {
				System.err.println("Error | Talon doesn't exist, check CANID");
			} else {
				--_loopTimeout;
			}
		}

		if (_talon.getControlMode() != ControlMode.MotionProfile) {
			_state = 0;
			_loopTimeout = -1;
		} else {
			switch (_state) {
				case 0: 
					if (_bStart) {
						_bStart = false;
						_setValue = SetValueMotionProfile.Disable;
						startFilling();
						_state = 1;
						_loopTimeout = kNumLoopsTimeout;
                    }
					break;

				case 1: 
					if (_status.btmBufferCnt > kMinPointsInTalon) {
						_setValue = SetValueMotionProfile.Enable;
						_state = 2;
						_loopTimeout = kNumLoopsTimeout;
					}
					break;
				case 2:
					if (_status.isUnderrun == false) {
						_loopTimeout = kNumLoopsTimeout;
					}
					if (_status.activePointValid && _status.isLast) {
						_setValue = SetValueMotionProfile.Hold;
						_state = 0;
						_loopTimeout = -1;
					}
					break;

			}
			_talon.getMotionProfileStatus(_status);
			_heading = _talon.getActiveTrajectoryHeading();
			_pos = _talon.getActiveTrajectoryPosition();
         	_vel = _talon.getActiveTrajectoryVelocity();

			SmartDashboard.putNumber("Right Error", Robot.drivetrain.getRightTalon().getClosedLoopError(0));
			SmartDashboard.putNumber("Left Error", Robot.drivetrain.getLeftTalon().getClosedLoopError(0));

			/* printfs and/or logging */
            System.out.println("Status: " + _status);
            System.out.println("Heading: " + _heading);
            System.out.println("Pos: " + _pos);
            System.out.println("Vel: " + _vel);

		}
		return _status;
	}

	private void startFilling() {
		/* since this example only has one talon, just update that one */
		startFilling(this.path, this.path.length);
    }
    
	private void startFilling(double[][] profile, int totalCnt) {
        TrajectoryPoint point = new TrajectoryPoint();

		if (_status.hasUnderrun) {
			System.out.print("Creating Point...");
			_talon.clearMotionProfileHasUnderrun(0);
		}
		_talon.clearMotionProfileTrajectories();
		_talon.configMotionProfileTrajectoryPeriod(0, Robot.drivetrain.timeout);

		for (int i = 0; i < totalCnt; ++i) {
			double positionRot = profile[i][0];

			double velocityRPM = profile[i][1];
			point.position = positionRot * (Robot.drivetrain.unitsPerInch * 12); //Convert Revolutions to Units
			point.velocity = (velocityRPM * (Robot.drivetrain.unitsPerInch * 12)); //Convert RPM to Units/100ms
			point.headingDeg = 0; /* future feature - not used in this example*/
			point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
			point.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
			point.timeDur = 10;
			point.zeroPos = false;
			if (i == 0)
                point.zeroPos = true; /* set this to true on the first point */
                
			point.isLastPoint = false;
			if ((i + 1) == totalCnt)
				point.isLastPoint = true; /* set this to true on the last point  */

			_talon.pushMotionProfileTrajectory(point);

		}

	}

	public void startMotionProfile() {
		_bStart = true;
    }
    
	public SetValueMotionProfile getSetValue() {
		return _setValue;
	}

}