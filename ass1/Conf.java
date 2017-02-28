package ass1;

/*
 * An immutable configuration of how the robot can be positioned.
 * Angles start with 0 for robot base angle. Joints only exist if a link
 * exists coming off that joint.
 * Joint positions describe the END POSITION of the link COMING OFF THAT
 * JOINT, not the position of the joint itself.
 */
public class Conf {
	private final double xPos;
	private final double yPos;
	private final double jointAng[]; // Relative angles
	private final double jointAngGbl[]; // Global angles to x-axis
	private final double linkEndPos[][];
	
	public Conf(String conf) {
		if (conf == null) {
			throw new RuntimeException("Invalid configuration specified");
		}
		
		// Split into parts
		String confParts[] = conf.split("\\s+");
		if (confParts.length < 2) {
			throw new RuntimeException("Invalid configuration specified");
		}
		
		// xPos and yPos
		this.xPos = Double.parseDouble(confParts[0]);
		this.yPos = Double.parseDouble(confParts[1]);
		
		// Joints arrays
		this.jointAng = new double[confParts.length - 2];
		this.jointAngGbl = new double[confParts.length - 2];
		this.linkEndPos = new double[confParts.length - 2][2];
		
		// Fill these arrays
		double jointAng[] = new double[confParts.length - 2];
		for (int i = 2; i < confParts.length; i++) {
			double curPart = Double.parseDouble(confParts[i]);
			jointAng[i - 2] = curPart;
		}
		setJointAngles(jointAng);
		
		// Check if all angles etc are valid
		if (inInvalidState()) {
			throw new RuntimeException("Invalid configuration specified");
		}
	}
	
	public Conf(double xPos, double yPos, double jointAng[]) {
		this.xPos = xPos;
		this.yPos = yPos;
		this.jointAng = new double[jointAng.length];
		this.jointAngGbl = new double[jointAng.length];
		this.linkEndPos = new double[jointAng.length][2];
		
		setJointAngles(jointAng);
		
		if (inInvalidState()) {
			throw new RuntimeException("Invalid configuration specified");
		}
	}
	
	public double getXPos() {
		return this.xPos;
	}
	
	public double getYPos() {
		return this.yPos;
	}
	
	public int getJointNum() {
		return this.jointAng.length;
	}
	
	public double getJointAng(int i) {
		return this.jointAng[i];
	}
	
	public double getLinkStartX(int i) {
		return (i == 0) ? this.xPos : this.linkEndPos[i - 1][0];
	}
	
	public double getLinkStartY(int i) {
		return (i == 0) ? this.yPos : this.linkEndPos[i - 1][1];
	}
	
	public double getLinkEndX(int i) {
		return this.linkEndPos[i][0];
	}
	
	public double getLinkEndY(int i) {
		return this.linkEndPos[i][1];
	}
	
	// Get the xyDistance between this and another conf
	public double xyDistance(Conf other) {
		double xShift = other.xPos - this.xPos;
		double yShift = other.yPos - this.yPos;
		
		return Math.sqrt(xShift*xShift + yShift*yShift);
	}
	
	// Finds the distance metric between this and another configuration
	// Given by euclidian distance in all dimensions, split into number of steps
	public double distance(Conf other) {
		// Get number of steps away xPos and yPos
		double xStepsDist = (other.xPos - this.xPos)/Helper.maxStep;
		double yStepsDist = (other.yPos - this.yPos)/Helper.maxStep;
		
		// Get number of squared distance this is away
		double sumSquaresDist = xStepsDist*xStepsDist + yStepsDist*yStepsDist;
		
		// Get steps for angles and add these as well
		for (int i = 0; i < other.jointAng.length; i++) {
			double angStepsDist = (other.jointAng[i] - this.jointAng[i])/Helper.maxAngStep;
			sumSquaresDist += angStepsDist*angStepsDist;
		}
		
		return Math.sqrt(sumSquaresDist);
	}
	
	// Returns a new conf just like this one but shifted by the amounts given,
	// or null if invalid.
	public Conf shift(double xPosShift, double yPosShift, double angleShift[]) {
		double xPosNew = this.xPos + xPosShift;
		double yPosNew = this.yPos + yPosShift;
		double angleNew[] = new double[Math.max(angleShift.length, this.jointAng.length)];
		
		for (int i = 0; i < angleShift.length; i++) {
			angleNew[i] = this.jointAng[i] + angleShift[i];
		}
		
		try {
			return new Conf(xPosNew, yPosNew, angleNew);
		} catch (RuntimeException e) {
			return null;
		}
	}
	
	@Override
	public boolean equals(Object o) {
		if (!(o instanceof Conf)) {
			return false;
		}
		
		Conf r = (Conf)o;
		
		return (this.xPos == r.xPos && this.yPos == r.yPos &&
				this.jointAng.equals(r.jointAng) &&
				this.jointAngGbl.equals(r.jointAngGbl));
	}
	
	@Override
    public int hashCode() {
        return (int) Math.round(this.xPos + this.yPos) + this.jointAng.hashCode() +
        		this.jointAngGbl.hashCode();
    }
	
	@Override
	public String toString() {
		String toRet = this.xPos + " " + this.yPos;
		for (double angle : this.jointAng) {
			toRet += " " + angle;
		}
		return toRet;
	}
	
	// Set up all the angles and joint end positions from the array given
	private void setJointAngles(double jointAng[]) {
		// Set angles and positions
		double curX = this.xPos;
		double curY = this.yPos;
		double curAng = 0;
		
		for (int i = 0; i < jointAng.length; i++) {
			// Individual joint angles are just the same
			this.jointAng[i] = jointAng[i];
			
			// Get updated X-Y positions and angles for each
			curAng += jointAng[i];
			curX += Math.cos(curAng)*Helper.linkLen;
			curY += Math.sin(curAng)*Helper.linkLen;
			
			// Use these to set link positions
			this.jointAngGbl[i] = curAng;
			this.linkEndPos[i][0] = curX;
			this.linkEndPos[i][1] = curY;
		}
	}
	
	// Test whether our configuration makes sense, or is invalid
	private boolean inInvalidState() {
		// Valid position
		if (xPos < 0 || xPos > 1 || yPos < 0 || yPos > 1) {
			return true;
		}
		
		// Valid angles
		for (int i = 0; i < jointAng.length; i++) {
			// Angles within valid bounds
			if (jointAng[i] <= Helper.minAng || jointAng[i] >= Helper.maxAng ||
					linkEndPos[i][0] < 0 || linkEndPos[i][0] > 1 ||
					linkEndPos[i][1] < 0 || linkEndPos[i][1] > 1) {
				return true;
			}
			
			// Not colliding with self
			// We test this joint with previous joints except the one right
			// before it, as this can never happen
			for (int j = 0; j < i - 1; j++) {
				if (Helper.linesCollide(getLinkStartX(j), getLinkStartY(j),
						getLinkEndX(j), getLinkEndY(j), getLinkStartX(i),
						getLinkStartY(i), getLinkEndX(i), getLinkEndY(i))) {
					return true;
				}
			}
		}
		
		return false;
	}
}
