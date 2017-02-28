package ass1;

/*
 * An immutable description of an obstacle. Obstacles are considered to
 * be a group of 4 lines which cannot be crossed. We can tell if they have
 * been crossed if either the robot's x-y position are inside, or if one
 * of the lines connects with one of the obstacle's outer lines.
 */
public class Obstacle {
	private final double ax; //Top left
	private final double ay; //Top left
	private final double bx; //Top right
	private final double by; //Top right
	private final double cx; //Bottom left
	private final double cy; //Bottom left
	private final double dx; //Bottom right
	private final double dy; //Bottom right
	
	public Obstacle(String conf) {
		if (conf == null) {
			throw new RuntimeException("Invalid configuration specified");
		}
		
		// Split into parts
		String confParts[] = conf.split("\\s+");
		if (confParts.length != 4) {
			throw new RuntimeException("Invalid configuration specified");
		}
		
		// Top left, bottom right
		this.ax = Double.parseDouble(confParts[0]);
		this.ay = Double.parseDouble(confParts[1]);
		this.dx = Double.parseDouble(confParts[2]);
		this.dy = Double.parseDouble(confParts[3]);
		if (this.ax > this.dx || this.ay < this.dy) {
			throw new RuntimeException("Invalid configuration specified");
		}
		
		// Top right, bottom left
		this.bx = this.dx;
		this.by = this.ay;
		this.cx = this.ax;
		this.cy = this.dy;
	}
	
	public boolean collidesWith(double x, double y) {
		// Ensure not encased within rectangle
		if (x <= dx && x >= ax && y >= dy && y <= ay) {
			return true;
		}
		
		return false;
	}
	
	public boolean collidesWith(double x1, double y1, double x2, double y2) {
		// Collides with a given line?
		if (collidesWith(x1, y1) || collidesWith(x2, y2)) {
			return true;
		}
		
		// Top line
		if (Helper.linesCollide(x1, y1, x2, y2, ax, ay, bx, by)) {
			return true;
		}
		
		// Left
		if (Helper.linesCollide(x1, y1, x2, y2, ax, ay, cx, cy)) {
			return true;
		}
		
		// Right
		if (Helper.linesCollide(x1, y1, x2, y2, bx, by, dx, dy)) {
			return true;
		}
		
		// Bottom
		if (Helper.linesCollide(x1, y1, x2, y2, cx, cy, dx, dy)) {
			return true;
		}
		
		return false;
	}
	
	public boolean collidesWith(Conf c) {
		// Ensure not encased within rectangle
		if (collidesWith(c.getXPos(), c.getYPos())) {
			return true;
		}
		
		// For all joints, get whether that joint collides with edges of
		// this rectangle.
		for (int i = 0; i < c.getJointNum(); i++) {
			// Top line
			if (Helper.linesCollide(c.getLinkStartX(i), c.getLinkStartY(i),
					c.getLinkEndX(i), c.getLinkEndY(i), ax, ay, bx, by)) {
				return true;
			}
			
			// Left
			if (Helper.linesCollide(c.getLinkStartX(i), c.getLinkStartY(i),
					c.getLinkEndX(i), c.getLinkEndY(i), ax, ay, cx, cy)) {
				return true;
			}
			
			// Right
			if (Helper.linesCollide(c.getLinkStartX(i), c.getLinkStartY(i),
					c.getLinkEndX(i), c.getLinkEndY(i), bx, by, dx, dy)) {
				return true;
			}
			
			// Bottom
			if (Helper.linesCollide(c.getLinkStartX(i), c.getLinkStartY(i),
					c.getLinkEndX(i), c.getLinkEndY(i), cx, cy, dx, dy)) {
				return true;
			}
		}
		
		return false;
	}
	
	public boolean collidesWith(Square s) {
		// No corner can be within rectangle
		if (collidesWith(s.ax, s.ay) || collidesWith(s.bx, s.by) || 
				collidesWith(s.cx, s.cy) || collidesWith(s.dx, s.dy)) {
			return true;
		}
		
		// We assume squares are too small for lines to contact each other
		
		return false;
	}
}
