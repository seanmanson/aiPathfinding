package ass1;

/*
 * An immutable description of a square on a 1.0x1.0 grid. Squares are taken
 * as four boundary lines, and are considered to collide if an obstacle crosses
 * them whatsoever.
 */
public class Square {
	public final double ax; //Top left
	public final double ay; //Top left
	public final double bx; //Top right
	public final double by; //Top right
	public final double cx; //Bottom left
	public final double cy; //Bottom left
	public final double dx; //Bottom right
	public final double dy; //Bottom right
	public final int squareX; //square number counting from left
	public final int squareY; //square number counting from bottom
	public final int squaresInWidth; //number of squares in width
	
	public Square(double xPos, double yPos, int squaresInWidth) {
		this.squaresInWidth = squaresInWidth;
		double grain = 1.0/squaresInWidth;
		this.squareX = (int) (xPos/grain);
		this.squareY = (int) (yPos/grain);
		this.ax = squareX*grain;
		this.ay = (squareY+1)*grain;
		this.bx = (squareX+1)*grain;
		this.by = (squareY+1)*grain;
		this.cx = squareX*grain;
		this.cy = squareY*grain;
		this.dx = (squareX+1)*grain;
		this.dy = squareY*grain;
	}
	
	public Square(int squareX, int squareY, int squaresInWidth) {
		this.squaresInWidth = squaresInWidth;
		double grain = 1.0/squaresInWidth;
		this.squareX = squareX;
		this.squareY = squareY;
		this.ax = squareX*grain;
		this.ay = (squareY+1)*grain;
		this.bx = (squareX+1)*grain;
		this.by = (squareY+1)*grain;
		this.cx = squareX*grain;
		this.cy = squareY*grain;
		this.dx = (squareX+1)*grain;
		this.dy = squareY*grain;
	}
	
	public double distance(Square other) {
		double xShift = other.ax - this.ax;
		double yShift = other.ay - this.ay;
		
		return Math.sqrt(xShift*xShift + yShift*yShift);
	}
	
	public Square sAbove() {
		if (squareY + 1 >= squaresInWidth) {
			return null;
		}
		
		return new Square(squareX, squareY + 1, squaresInWidth);
	}
	
	public Square sLeft() {
		if (squareX - 1 < 0) {
			return null;
		}
		
		return new Square(squareX - 1, squareY, squaresInWidth);
	}
	
	public Square sRight() {
		if (squareX + 1 >= squaresInWidth) {
			return null;
		}
		
		return new Square(squareX + 1, squareY, squaresInWidth);
	}
	
	public Square sBelow() {
		if (squareY - 1 < 0) {
			return null;
		}
		
		return new Square(squareX, squareY - 1, squaresInWidth);
	}
	
	@Override
	public boolean equals(Object o) {
		if (!(o instanceof Square)) {
			return false;
		}
		
		Square r = (Square)o;
		
		return (this.squareX == r.squareX && this.squareY == r.squareY);
	}
	
	@Override
    public int hashCode() {
		return this.squareX * 31 + this.squareY;
    }
	
	@Override
	public String toString() {
		return "[" + squareX + ", " + squareY + "] (" + ax + " " + ay + ") (" + dx + " " + dy + ")";
	}
}
