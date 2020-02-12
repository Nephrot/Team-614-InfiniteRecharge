package frc.lib;
import java.util.ArrayList;
import frc.lib.*;
public class ProfilePoint {
	public double pos = 0;
	public double vel = 0;
	public double acc = 0;
	public double head = 0;

	// Motion Profiling for VictorSP as they do not come with their own trajectory point
	// feature
	public ProfilePoint(double pos, double vel, double acc, double heading) {
		this.pos = pos;
		this.vel = vel;
		this.acc = acc;
		this.head = heading;
	}

	public ProfilePoint() {}
	public String returnString() {
		return ("(" + pos + ", " + vel + ", " + acc + ")");
	}
}