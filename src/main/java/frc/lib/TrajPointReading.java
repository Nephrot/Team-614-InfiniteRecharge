package frc.lib;

import java.util.Timer;
// import com.ctre.phoenix.motion.TrajectoryPoint;
public class TrajPointReading {

	public static double[][] target;

	boolean reverse = false;

	public static double totalTime = 0;
	public static TrajPointReading tmp;

	public static double timer = 0;

	public static ProfilePoint testPoint;

	public static double before;
	public static double current;

	public static int j;

	public TrajPointReading(double[][] path) {
		this.target = path.clone();
		for (int i = 0; i < target.length; i++) {
			if (i == 0) {
				if (target[i][2] != 0) {
					totalTime += ((target[i][1] - 0) / target[i][2]);
				}
			} else if (i == target.length) {
				totalTime += ((0 - target[i][1]) / target[i][1]);
			} else {
				totalTime += ((target[i][1] - target[i - 1][1]) / target[i][2]);
			}
		}
	}

	// public static void main(String[] args) {
	// tmp = new TMP(CreateNewPath.pathLeft);
	// double i = 0;
	// while (i < totalTime) {
	// testPoint = getAtTime(i);
	// System.out.println("Position: " + testPoint.pos);
	// System.out.println("Velocity: " + testPoint.vel);
	// System.out.println("Acceleration: " + testPoint.acc);
	// System.out.println("Output: " + testPoint.vel * 0.111);
	// i += 0.01;
	// }
	// }
	// For Testing

	public static ProfilePoint getAtTime(double t) {
		ProfilePoint  point = new  ProfilePoint();
       
		for (int i = 0; timer < t; i++) {
		 if(i < target.length) {
			if (i == 0) {
				if (target[i][2] != 0) {
					timer += ((target[i][1] - 0) / target[i][2]);
				}
			} else if (i == target.length) {
				timer += ((0 - target[i][1]) / target[i][1]);
			} else {
				timer += ((target[i][1] - target[i - 1][1]) / target[i][2]);
			}
			point.pos = target[i][0];
			point.vel = target[i][1];
			point.acc = target[i][2];
			point.head = target[i][3];
		 }
		 else {
			 break;
		 }
		}
		timer = 0;
		return point;
	}

	public double getDuration() {
		return totalTime;
	}
}