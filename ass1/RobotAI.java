package ass1;

import java.io.*;
import java.util.*;

public class RobotAI {
	private Conf start;
	private Conf end;
	private Obstacle obs[];
	
	public RobotAI(String inFileLoc, String outFileLoc) throws IOException {
		// Read in conditions from input file
		setupConditions(inFileLoc);
		
		// Open output file for completion
		PrintWriter outFile = new PrintWriter (new BufferedWriter(new FileWriter(
				outFileLoc)));
		
		System.out.println("Read input files.");
		
		// Find grid-based path for point robot
		List<Square> gridPath = obtainGridPath(new Square(start.getXPos(), start.getYPos(), 100),
				new Square(end.getXPos(), end.getYPos(), 100));
		if (gridPath == null) {
			System.err.println("No grid path to end could be found; path sampling will not be active.");
		}
		
		// Create graph for samples
		Graph<Conf> roadMap = new Graph<Conf>();
		Node<Conf> startNode = roadMap.addNode(start, null);
		Node<Conf> endNode = roadMap.addNode(end, null);
		
		// Repeat until solution found, increasing nodes each time.
		List<Conf> keyPath = null;
		int sampleNum = 2;
		
		while (true) {
			// Construction phase
			System.out.println("Constructing with " + sampleNum + " more samples...");
			findSamples(roadMap, gridPath, sampleNum);
			
			// Query phase
			System.out.println("Querying...");
			keyPath = obtainPathAlongGraph(roadMap, startNode, endNode);
			
			if (keyPath != null) {
				break;
			} else {
				sampleNum += sampleNum/4 + 2;
				if (sampleNum > 200) {
					sampleNum = 200;
				}
			}
		}
		
		// Print this solution by moving linearly between all nodes
		System.out.println("Found path! Printing to file...");
		List<Conf> fullPath = getFullPath(keyPath);
		outFile.println(fullPath.size() - 1);
		for (Conf p : fullPath) {
			outFile.println(p);
		}
		
		outFile.close();
		System.out.println("Done.");
	}
	
	private void setupConditions(String inFileLoc) throws IOException {
		BufferedReader inFile = new BufferedReader(new FileReader(inFileLoc));
		
		// Setup start conditions
		String curLine = inFile.readLine();
		this.start = new Conf(curLine);
		
		// Setup end conditions
		curLine = inFile.readLine();
		this.end = new Conf(curLine);
		
		// Number of obstacles
		curLine = inFile.readLine();
		this.obs = new Obstacle[Integer.parseInt(curLine)];
		
		// Obstacle info
		for (int i = 0; i < this.obs.length; i++) {
			curLine = inFile.readLine();
			this.obs[i] = new Obstacle(curLine);
		}
		
		inFile.close();
		
		// Check valid
		if (this.start.getJointNum() != this.end.getJointNum()) {
			throw new RuntimeException("Invalid configuration specified");
		}
		for (Obstacle o : this.obs) {
			if (o.collidesWith(this.start) || o.collidesWith(this.end)) {
				throw new RuntimeException("Invalid configuration specified");
			}
		}
	}
	
	/*
	 * Uses A* algorithm to find a grid path for the robot as a point robot
	 * to the end, returning null if no path found.
	 */
	private List<Square> obtainGridPath(Square start, Square end) {
		// Initial setup
		Set<Square> closed = new HashSet<>();
		Map<Square, Square> source = new HashMap<>();
		Map<Square, Integer> gVals = new HashMap<>();
		Map<Square, Double> fVals = new HashMap<>();
		PriorityQueue<Square> open = new PriorityQueue<Square>(200, new CompVal<Square>(fVals));
		
		// Setup first node
		gVals.put(start, 0);
		fVals.put(start, start.distance(end));
		open.add(start);
		
		// Constantly expand the highest priority nodes (lowest number)
		Square cur;
		while ((cur = open.poll()) != null) {
			// Test if we finished
			if (cur.equals(end)) {
				// Reconstruct path
				LinkedList<Square> path = new LinkedList<>();
				Square tracer = cur;
				while (tracer != null) {
					path.addFirst(tracer);
					tracer = source.get(tracer);
				}
				return path;
			}
			
			// If not, visit this node
			closed.add(cur);
			
			// Get successors
			Square successors[] = {cur.sAbove(), cur.sBelow(), cur.sLeft(), cur.sRight()};
			
			for (Square successor : successors) {
				// If null, ignore as this is not valid
				if (successor == null) {
					continue;
				}
				
				// Ignore if already visited
				if (closed.contains(successor)) {
					continue;
				}
				
				// If collides with obstacle, ignore as this isn't relevant either
				if (collidesObs(successor)) {
					continue;
				}
				
				// Get g-distance to this node (always +1 as adjacent)
				int newGVal = gVals.get(cur) + 1;
				Integer currentGVal = gVals.get(successor);
				
				// If we haven't added it, or this distance is better, add it
				boolean inOpenSet = open.contains(successor);
				if (!inOpenSet || currentGVal == null || newGVal < currentGVal) {
					// Set this as a new node/update previous node data
					source.put(successor, cur);
					gVals.put(successor, newGVal);
					fVals.put(successor, successor.distance(end));
					
					// Add this node to overall queue to visit if needed
					if (!inOpenSet) {
						open.add(successor);
					}
				}
			}
		}
		
		// Fail if all nodes visited without success
		return null;
	}
	
	/*
	 * Returns a random RobotConf in a valid state.
	 */
	private Conf randomConf(Random rand) {
		Conf r;
		
		while (true) {
			// Randomise values
			double xPos = rand.nextDouble();
			double yPos = rand.nextDouble();
			double jointAng[] = new double[this.start.getJointNum()];
			for (int i = 0; i < jointAng.length; i++) {
				jointAng[i] = rand.nextDouble() * (Helper.maxAng - Helper.minAng) + Helper.minAng;
			}
			
			// Create this random RobotConf
			try {
				r = new Conf(xPos, yPos, jointAng);
			} catch (RuntimeException e) {
				// Invalid; we'll make another one instead.
				continue;
			}
			
			// Success!
			break;
		}
		
		return r;
	}
	
	/*
	 * Returns whether the given conf collides with any obstacles.
	 */
	private boolean collidesObs(Conf c) {
		for (Obstacle o : this.obs) {
			if (o.collidesWith(c)) {
				return true;
			}
		}
		
		return false;
	}
	private boolean collidesObs(Square s) {
		for (Obstacle o : this.obs) {
			if (o.collidesWith(s)) {
				return true;
			}
		}
		
		return false;
	}
	
	/*
	 * Gets random configuration samples (up to the number specified) and
	 * places them as nodes in the graph, connecting them if possible.
	 * Connections are made by checking if it is possible to move linearly to
	 * this position. 
	 */
	private void findSamples(Graph<Conf> roadMap, List<Square> gridPath, int sampleNum) {
		Conf randConf;
		Random rand = new Random();
		int curStrategy;
		
		int gridPathPref[] = new int[gridPath.size()];
		Arrays.fill(gridPathPref, 1);
		
		for (int i = 0; i < sampleNum; i++) {
			// Find which strategy to use=
			if (gridPath == null) {
				// With no path, we can only randomly choose
				curStrategy = 0;
			} else {
				// Use path 80% of the time, random 20%
				if (rand.nextDouble() > 0.8) {
					curStrategy = 0;
				} else {
					curStrategy = 1;
				}
			}
			
			// Get a conf using this strategy
			if (curStrategy == 0) {
				randConf = findSimpleSample(rand);
			} else {
				randConf = findPathSample(rand, gridPath, gridPathPref);
			}
			
			// Find all graph nodes that can be reached linearly
			List<Node<Conf>> neighbours = new LinkedList<>();
			for (Node<Conf> n : roadMap.nodes) {
				// Only bother if nearby
				if (randConf.distance(n.data) > 3000) {
					continue;
				}
				
				if (canConnectLinearly(randConf, n.data)) {
					neighbours.add(n);
				}
			}
			
			// Create this node and add it to the graph
			roadMap.addNode(randConf, neighbours);
		}
	}
	
	/*
	 * Finds a sample at pure random in the graph.
	 */
	private Conf findSimpleSample(Random rand) {
		Conf s;
		
		while (true) {
			// Get a purely random conf
			s = randomConf(rand);
			
			// If it doesn't collide, OK!
			if (!collidesObs(s)) {
				return s;
			}
		}
	}
	
	/*
	 * Finds a sample on a path of grid squares known to travel from start to end
	 */
	private Conf findPathSample(Random rand, List<Square> gridPath, int gridPathPref[]) {
		Square s = null;
		Conf r;
		int i = -1;
		
		// Find a square to test within
		if (rand.nextDouble() > 0.8) {
			s = gridPath.get(rand.nextInt(gridPath.size()));
		} else {
			int sum = 0;
			for (int p : gridPathPref) {
				sum += p;
			}
			int val = rand.nextInt(sum);
			for (i = 0; i < gridPathPref.length; i++) {
				val -= gridPathPref[i];
				if (val <= 0) {
					s = gridPath.get(i);
					break;
				}
			}
		}
		
		while (true) {
			// Find a random point within this square
			double xPos = rand.nextDouble() * (s.bx - s.ax) + s.ax;
			double yPos = rand.nextDouble() * (s.ay - s.dy) + s.dy;
			double jointAng[] = new double[this.start.getJointNum()];
			for (int j = 0; j < jointAng.length; j++) {
				jointAng[j] = rand.nextDouble() * (Helper.maxAng - Helper.minAng) + Helper.minAng;
			}
			
			// Create this random RobotConf
			try {
				r = new Conf(xPos, yPos, jointAng);
				
				// See if it collides
				if (!collidesObs(r)) {
					return r; //Success!
				}
			} catch (RuntimeException e) {
				// Invalid; we'll make another one instead.
			}
			
			if (i != -1) {
				gridPathPref[i]++;
			}
		}
	}
	
	/*
	 * Returns whether the given two configurations can connect linearly.
	 * A linear connection tries to travel directly from one position to the
	 * next, rotating arms constantly as needed from the beginning.
	 */
	private boolean canConnectLinearly(Conf from, Conf to) {
		double stepXShift;
		double stepYShift;
		double stepAngleShift[] = new double[from.getJointNum()];
		
		// First test if the beeline for the base collides
		for (Obstacle o : obs) {
			if (o.collidesWith(from.getXPos(), from.getYPos(), to.getXPos(), to.getYPos())) {
				return false;
			}
		}
		
		// Get how many steps to do this over (max number of steps for all movements/angles)
		long xySteps = Math.round(to.xyDistance(from)/Helper.maxStep);
		long angleSteps = 0;
		for (int i = 0; i < from.getJointNum(); i++) {
			long curAngleSteps = Math.round(Math.abs(to.getJointAng(i) - from.getJointAng(i))/Helper.maxAngStep);
			if (curAngleSteps > angleSteps) {
				angleSteps = curAngleSteps;
			}
		}
		long steps = (xySteps > angleSteps) ? xySteps : angleSteps;
		
		// Get step sizes
		stepXShift = (to.getXPos() - from.getXPos())/steps;
		stepYShift = (to.getYPos() - from.getYPos())/steps;
		for (int i = 0; i < stepAngleShift.length; i++) {
			stepAngleShift[i] = (to.getJointAng(i) - from.getJointAng(i))/steps;
		}
		double angleShift[] = new double[stepAngleShift.length];
		
		// Prepare for binary sifting
		long sectSize = steps;
		
		Conf shifted = from;
		while (true) {
			sectSize /= 2;
			
			// For each value separated by sectSize
			for (long curStep = 0; curStep < steps; curStep += sectSize) {
				// See if this has been tested before
				boolean testedBefore = false;
				for (long prevSectSize = steps/2; prevSectSize != sectSize; prevSectSize/=2) {
					if (curStep % (prevSectSize) == 0) {
						testedBefore = true;
						break;
					}
				}
				
				if (!testedBefore) {
					// Try and shift this configuration
					for (int k = 0; k < angleShift.length; k++) {
						angleShift[k] = curStep*stepAngleShift[k];
					}
					shifted = from.shift(curStep*stepXShift, curStep*stepYShift, angleShift);
					if (shifted == null) {
						return false;
					}
					
					// Test if it collides
					for (Obstacle o : obs) {
						if (o.collidesWith(shifted)) {
							return false;
						}
					}
					
				}
			}
			
			// If we have gone through finest system, return
			if (sectSize == 1) {
				break;
			}
			
		}
		
		return true;
	}
	
	/*
	 * Finds a path along the given graph between two nodes using A*, returning
	 * null if no path can be found.
	 */
	private List<Conf> obtainPathAlongGraph(Graph<Conf> roadMap,
			Node<Conf> startNode, Node<Conf> endNode) {
		// Initial setup
		Set<Node<Conf>> closed = new HashSet<>();
		Map<Conf, Conf> source = new HashMap<>();
		Map<Conf, Double> gVals = new HashMap<>();
		Map<Conf, Double> fVals = new HashMap<>();
		PriorityQueue<Node<Conf>> open = new PriorityQueue<>(roadMap.nodes.size(), new NodeCompVal<Conf>(fVals));
		
		// Setup first node
		gVals.put(startNode.data, 0.0);
		fVals.put(startNode.data, startNode.data.distance(endNode.data));
		open.add(startNode);
		
		// Constantly expand the highest priority nodes (lowest number)
		Node<Conf> cur;
		while ((cur = open.poll()) != null) {
			// Test if we finished
			if (cur.equals(endNode)) {
				// Reconstruct path
				LinkedList<Conf> path = new LinkedList<>();
				Conf tracer = cur.data;
				while (tracer != null) {
					path.addFirst(tracer);
					tracer = source.get(tracer);
				}
				return path;
			}
			
			// If not, visit this node
			closed.add(cur);
			
			// For all successors
			for (Node<Conf> successor : cur.neighbours) {
				// Ignore if already visited
				if (closed.contains(successor)) {
					continue;
				}
				
				// Get g-distance to this node
				double newGVal = gVals.get(cur.data) + cur.data.distance(successor.data);
				Double currentGVal = gVals.get(successor.data);
				
				// If we haven't added it, or this distance is better, add it
				boolean inOpenSet = open.contains(successor);
				if (!inOpenSet || currentGVal == null || newGVal < currentGVal) {
					// Set this as a new node/update previous node data
					source.put(successor.data, cur.data);
					gVals.put(successor.data, newGVal);
					fVals.put(successor.data, successor.data.distance(endNode.data));
					
					// Add this node to overall queue to visit if needed
					if (!inOpenSet) {
						open.add(successor);
					}
				}
			}
		}
		
		// Fail if all nodes visited without success
		return null;
	}
	
	/*
	 * Given a path of Confs, all known to connect linearly, this prints the
	 * linear steps taken shifting along this path with the same algorithm
	 * used to detect colisions.
	 */
	private List<Conf> getFullPath(List<Conf> path) {
		LinkedList<Conf> fullPath = new LinkedList<Conf>();
		
		// Add each edge's start but not end
		for (int i = 0; i < path.size() - 1; i++) {
			addSingleLine(fullPath, path.get(i), path.get(i + 1));
		}
		
		// Add the final end point
		fullPath.add(path.get(path.size() - 1));
		return fullPath;
	}
	
	private void addSingleLine(LinkedList<Conf> fullPath, Conf from, Conf to) {	
		double totalXShift = 0;
		double totalYShift = 0;
		double totalAngleShift[] = new double[from.getJointNum()];
		
		fullPath.add(from);
		
		// Get how many steps to do this over (max number of steps for all movements/angles)
		long xySteps = Math.round(to.xyDistance(from)/Helper.maxStep);
		long angleSteps = 0;
		for (int i = 0; i < from.getJointNum(); i++) {
			long curAngleSteps = Math.round(Math.abs(to.getJointAng(i) - from.getJointAng(i))/Helper.maxAngStep);
			if (curAngleSteps > angleSteps) {
				angleSteps = curAngleSteps;
			}
		}
		long steps = (xySteps > angleSteps) ? xySteps : angleSteps;
		
		Conf shifted = from;
		for (long j = 0; j < steps; j++) {
			// Only move one step in x and y
			totalXShift += (to.getXPos() - from.getXPos())/steps;
			totalYShift += (to.getYPos() - from.getYPos())/steps;
			
			// Move all angles neccessary;
			for (int i = 0; i < totalAngleShift.length; i++) {
				totalAngleShift[i] += (to.getJointAng(i) - from.getJointAng(i))/steps;
			}
			
			// Try and shift this configuration
			shifted = from.shift(totalXShift, totalYShift, totalAngleShift);
			if (shifted == null) {
				System.err.println("Something went wrong retracing path");
			}
			
			fullPath.add(shifted);
		}
	}
	
	/*
	 * Used to compare two maps and return the one with the lowest set F value
	 */
	private class CompVal<T> implements Comparator<T> {
		private Map<T, Double> fVals;
		
		public CompVal(Map<T, Double> fVals) {
			this.fVals = fVals;
		}
		
		public int compare(T a, T b) {
			Double aVal = fVals.get(a);
			Double bVal = fVals.get(b);
			if (aVal == null) {
				if (bVal == null) {
					return 0;
				}
				return 1;
			} else if (bVal == null) {
				return -1;
			}
			
			if (aVal > bVal) {
				return 1;
			} else if (aVal == bVal) {
				return 0;
			} else {
				return -1;
			}
		}
	}
	
	private class NodeCompVal<T> implements Comparator<Node<T>> {
		private Map<T, Double> fVals;
		
		public NodeCompVal(Map<T, Double> fVals) {
			this.fVals = fVals;
		}
		
		public int compare(Node<T> a, Node<T> b) {
			Double aVal = fVals.get(a.data);
			Double bVal = fVals.get(b.data);
			if (aVal == null) {
				if (bVal == null) {
					return 0;
				}
				return 1;
			} else if (bVal == null) {
				return -1;
			}
			
			if (aVal > bVal) {
				return 1;
			} else if (aVal == bVal) {
				return 0;
			} else {
				return -1;
			}
		}
	}
	
	
	/*
	 * Usage: a1-3702 inputFileName outputFileName
	 */
	public static void main(String[] args) {
		if (args.length != 2) {
			System.err.println("Usage: a1-3702 inputFileName outputFileName");
			return;
		}
		
		// Open files for read/write
		try {
			new RobotAI(args[0], args[1]);
		} catch (IOException e) {
			System.err.println("File read/write error: " + e);
		}
	}
}