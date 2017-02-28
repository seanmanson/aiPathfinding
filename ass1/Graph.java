package ass1;

import java.util.*;


public class Graph<T> {
	public List<Node<T>> nodes;
	
	
	public Graph() {
		nodes = new LinkedList<Node<T>>();
	}

	public Node<T> addNode(T data, List<Node<T>> neighbours) {
		// Set up node
		Node<T> node = new Node<T>();
		node.data = data;
		node.neighbours = (neighbours != null) ? neighbours : new LinkedList<Node<T>>();
		
		// Set up their neighbours
		for (Node<T> n : node.neighbours) {
			n.neighbours.add(node);
		}
		
		// Add this to overall graph
		nodes.add(node);
		
		return node;
	}
	
}
