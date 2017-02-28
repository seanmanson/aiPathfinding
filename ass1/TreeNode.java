package ass1;

import java.util.*;


public class TreeNode<T> {
	public T data;
	public TreeNode<T> parent;
	public List<TreeNode<T>> children;
	
	public TreeNode(T data) {
		this.data = data;
		this.children = new LinkedList<TreeNode<T>>();
	}

	public boolean isRoot() {
		return parent == null;
	}

	public boolean isLeaf() {
		return children.size() == 0;
	}

	public TreeNode<T> addChild(T child) {
		TreeNode<T> childNode = new TreeNode<T>(child);
		childNode.parent = this;
		this.children.add(childNode);
		return childNode;
	}

	public int getLevel() {
		return this.isRoot() ? 0 : parent.getLevel() + 1;
	}

}
