package ass1;

// Helper methods
public class Helper {
	public final static double maxAng = 5*Math.PI/6;
	public final static double minAng = -5*Math.PI/6;
	public final static double linkLen = 0.05;
	public final static double maxStep = 0.001;
	public final static double maxAngStep = Math.PI/1800;
	
	/*
	 * Tests whether two lines u and v collide, defined each by two points
	 * u1, u2 and v1, v2. Returns true on collision, false otherwise.
	 */
	public static boolean linesCollide(double u1x, double u1y, double u2x,
			double u2y, double v1x, double v1y, double v2x, double v2y) {
		// Test if in same interval
		if (Math.min(u1x, u2x) > Math.max(v1x, v2x) || Math.max(u1x, u2x) < Math.min(v1x, v2x) ||
				Math.min(u1y, u2y) > Math.max(v1y, v2y) || Math.max(u1y, u2y) < Math.min(v1y, v2y)) {
			return false;
		}
		
		// Get vector movements
		double dux = u2x - u1x;
		double duy = u2y - u1y;
		double dvx = v2x - v1x;
		double dvy = v2y - v1y;
		
		// Test determinant = 0
		double det = dvx*duy - dux*dvy;
		
		// If yes, colinear test, as lines are parallel
		if (det == 0) {
			if (dux == 0 || dvx == 0) {
				// If lines are vertical, then they must cross, as we already
				// showed their intervals intersect
				return true;
			}
			// Otherwise, test if they have the same vertical shift
			return (u2x*u1y - u1x*u2y)/dux == (v2x*v1y - v1x*v2y)/dvx;
		}
		
		// If no, standard test
		// Get multipliers for intersection num
		double l = (duy*(u1x - v1x) - dux*(u1y - v1y))/det;
		double k = (dvy*(u1x - v1x) - dvx*(u1y - v1y))/det;
		
		// If multipliers are between 0x and 1x length, bam.
		return (l >= 0 && l <= 1 && k >= 0 && k <= 1);
	}
}
