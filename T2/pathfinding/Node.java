package pathfinding;
// Main authors to code/concepts: James and Sean
public class Node{
	/** 
	* J: The node class is a custom data-structure implemented to supplement the Map and AStar.
	* The Map is formed by a network of nodes, which is operated on by AStar to generate a path. 
	* Nodes have properties useful to the implementation of Map and AStar,
	* i.e. the property of being online or offline
	* @author James Burroughs, Seokhwan Jung
	*/

	int x;
    int y;
    private boolean isOffline;
    private Node nodesParent;
	
	/** 
	*  J: heuristic cost (represents estimated cost from node n to goal node)
	*/
    private int hCostOfNode;
	
	/** 
	* Exact cost of path from the start node to node n
	*/
    private int gCostOfNode; 
	
	/** 
	* final cost, F, equal to h cost plus G cost of node and g cost of node
	*/
    private int fCombinedCostOfNode;  //final cost, F, equal to h cost plus G cost of node and g cost of node
       
	/** 
	*  J: The class is constructed by taking a coordinate (row and column value). 
	*  Super() for parent node functionality
	*  @param row
	*  @param column
	*/
    public Node(int row, int column){
        super();
        this.x = row;
        this.y = column;
    }
    
       /** 
	* J: This method returns the Offline status of the node (will return TRUE if the node is OFFLINE)
	*/
    public boolean isNodeOffline(){
        return isOffline;
    }
	
	/** 
	* J: This method sets the nodes Offline status (if TRUE is passed as a parameter then the node is
	* OFFLINE and not to be considered during pathfinding with AStar
	*/
    public void setOffline(boolean z){
        this.isOffline = z;
    }
	
       /** 
	*  J: Takes the current state of the (LIVE NODE)
	* and is used to update the state of the node as AStar progresses
	* i.e. for when a better G-Cost is discovered.
	* @param currentState
	* @param cost
	*/
    public void setState(Node currentState, int cost){
        int costToG = currentState.getGCost() + cost;
        setParental(currentState);
        setGCost(costToG);
        findFinalCost();
    }
	
        /** 
	* J: This method calculates the chosen heuristic: the Manhattan Heuristic. I.e. the deltas on x and y between a node in question
	* and the destination are calculated
	* @param destination
	*/
    public void generateManhattanHeuristic(Node destination){
        this.hCostOfNode = Math.abs(destination.getY()-getY()) + Math.abs(destination.getX()-getX()) ;
    }
    
	/** 
	* J: Method returns true and updates the state if a shorter route with respect to gCost exists
	* (as heuristic node is constant for individual nodes)for THIS node's current state
	* @param currentState
	* @param cost
	*/
    public boolean isThereAShorterRoute(Node currentState,int cost){
        int gCost = currentState.getGCost() +cost;
        if (gCost<getGCost()){ setState(currentState, cost);
            return true;}
        return false;
    }
    
	/** 
	* J: Method calculates fCost (finalCost) for THIS node
	*/
    private void findFinalCost(){
        int finalCost= getHeuristic() + getGCost();
        setFCost(finalCost);}
        
	/** 
	* J: This method prints useful information for THIS node
	*/
    public void printNodeProperties(){
        System.out.println("This node's coordinates are: " + "(" + x + "," + y + ")" + " ... " + "Is this node offline (true or false)? " + isOffline);}
    
	/** 
	* J: This method returns the G-Cost of THIS node
	*/ 
    public int getGCost(){
        return gCostOfNode;
    }
	
	/** 
	*  J: This method sets the G-Cost of THIS node
	* @param g
	*/
    public void setGCost(int g){
        this.gCostOfNode = g;
    }
	
	/** 
	* J: This method gets the F-Cost of THIS node
	*/
    public int getFCost(){
        return fCombinedCostOfNode;
    }
	
	
	/** 
	* J: This method sets the F-Cost of THIS node
	* @param f
	*/
    public void setFCost(int f){
        this.fCombinedCostOfNode = f;
    }
	
       /** 
	* J: This is a redundant method used to convert between simple coordinate systems during prior testing
	*/
    public void nodeConverter(){
    	if (x  == 5) {
    		this.x = 0;	
    	}
    	else if (x == 4) {
    		this.x = 1;
    	}
    	else if (x == 3) {
    		this.x = 2;
    	}
    	else if (x == 2) {
    		this.x = 3;
    	}
    	else if (x == 1) {
    		this.x = 4;
    	}
    	else if (x == 0) {
    		this.x = 5;
    	}
    	}
   
	
    /** 
     * J: This method returns the heuristic (H-Cost) of THIS node	
     */ 
    public int getHeuristic(){
        return hCostOfNode;
    }
	
    /** 
     * J: This method is used to set a Heuristic value for THIS node (h passed as a parameter)
     * @param h
     */
    public void setHeuristic(int h){
        this.hCostOfNode = h;
    }
	
        /** 
	* J: This method returns the parent node of THIS node
	*/
    public Node getParentNode(){
        return nodesParent;
    }
	
       /** 
	* J: This method sets the parent node of THIS node to the parent passed as a parameter
	* @param parent
	*/
    public void setParental(Node parent){
        this.nodesParent = parent;
    }
	
       /** 
	* J: This method compares takes a node param and returns TRUE if it is has same coordinates as THIS node.
	* @param toCompare
	*/
    public boolean equals(Node toCompare){
        Node anotherNode = toCompare;
        return this.getX()==anotherNode.getX() && this.getY()==anotherNode.getY();
    }
	
	/** 
	* J: This method returns the X coordinate value of the node (its value on the X axis)
	*/
    public int getX(){
        return x;
    }
	/** 
	* J: This method sets the X coordinate value of the node to the param value
	* @param row
	*/
    public void setX(int row){
        this.x = row;
    }
	
	/** 
	*  J: This method returns the Y coordinate value of the node
	*/
    public int getY(){
        return y;
    }
	/** 
	* J: This method sets the Y coordinate value of the node to the param value
	* @param column
	*/
    public void setY(int column) {
        this.y = column;
    }
    

}
