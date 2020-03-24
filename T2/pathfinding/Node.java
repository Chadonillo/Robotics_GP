package pathfinding;
// Main authors to code/concepts: James and Sean
public class Node{
    
	int x;
    int y;
    private boolean isOffline;
    private Node nodesParent;
    private int hCostOfNode;  // heuristic cost (represents estimated cost from node n to goal node)
    private int gCostOfNode;  //Exact cost of path from the start node to node n
    private int fCombinedCostOfNode;  //final Cost


    public Node(int row, int column){
        super();
        this.x = row;
        this.y = column;
    }
    
    public boolean isNodeOffline(){
        return isOffline;
    }
    
    public void setOffline(boolean z){
        this.isOffline = z;
    }
    
    public void setState(Node currentState, int cost){
        int costToG = currentState.getGCost() + cost;
        setParental(currentState);
        setGCost(costToG);
        findFinalCost();
    }

    public void generateHeuristic(Node destination){
        this.hCostOfNode = Math.abs(destination.getY()-getY()) + Math.abs(destination.getX()-getX()) ;
    }


    public boolean isThereAShorterRoute(Node currentState,int cost){
        int gCost = currentState.getGCost() +cost;
        if (gCost<getGCost()){ setState(currentState, cost);
            return true;}
        return false;
    }

    private void findFinalCost(){
        int finalCost= getHeuristic() + getGCost();
        setFCost(finalCost);}


    public void printNodeProperties(){
        System.out.println("This node's coordinates are: " + "(" + x + "," + y + ")" + " ... " + "Is this node offline (true or false)? " + isOffline);}
    
    public int getGCost(){
        return gCostOfNode;
    }

    public void setGCost(int g){
        this.gCostOfNode = g;
    }

    public int getFCost(){
        return fCombinedCostOfNode;
    }

    public void setFCost(int f){
        this.fCombinedCostOfNode = f;
    }
    
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
   
    public int getHeuristic(){
        return hCostOfNode;
    }

    public void setHeuristic(int h){
        this.hCostOfNode = h;
    }

    public Node getParentNode(){
        return nodesParent;
    }

    public void setParental(Node parent){
        this.nodesParent = parent;
    }
    
    public boolean equals(Node toCompare){
        Node anotherNode = toCompare;
        return this.getX()==anotherNode.getX() && this.getY()==anotherNode.getY();
    }

    public int getX(){
        return x;
    }

    public void setX(int row){
        this.x = row;
    }

    public int getY(){
        return y;
    }

    public void setY(int column) {
        this.y = column;
    }
    

}
