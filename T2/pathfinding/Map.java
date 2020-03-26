package pathfinding;
//Main contributors to concepts/code: James, Sean
//Author of comments does not neccasarily imply the physical author of the code.
import java.util.List;
/**
* J: This class encodes the map as a series of nodes to be used within the A* pathfinding class. 
* In essence, the map is a grid derived from dimensions that correlate to the square arena given
* in the task. The concept can be visualised by imagining a grid network of online nodes and offline
* nodes, aka places where Wall-z is unable to go and are not included for pathfinding. 
*
* The locations on the grid such as walls, objects and obstacles are represented by offline nodes.
* I.e. these nodes are offline, are not considered by AStar and should not be visited by Wall-Z.
* By adding the corresponding node to the offline array (A* will not consider offline nodes)
* and set the boolean offline status of the node to false. Blocked nodes signify places we do not want Wall-Z to go or
* collide with. Hence when we feed this map into the A* pathfinding class, a valid route is retrived 
* in which Wall-Z behaves and traverses the arena as desired and required. 
* 
* Another feature of this class is that it permits a variety of routes to be dynamically generated
* as required, E.g. once Wall-Z learns information about the arena, such as the
* position of another obstacle after reading the colour in the box, this map is updated dynamically.
* By dynamically editing the offline node list we prevent Wall-Z visiting and/or colliding with obstacles, walls, etc.
* 
* Also, once wall-z localises himself on the strip, he proceeds to move onto the nearest node on this
* map/grid node network. His starting node is noted and used to calculate an efficient route to any online node on the map
* Using various starting (Wall-Z current positions)and finishing points Wall-Z is able calculate a path with A* and 
* traverse/interact with the arena as required using Brandon's navigation classes.
* @author James Burroughs, Seokhwan Jung
*/
public class Map {
	private Node[][] mapArea;
	/** 
	* J: A list of offline nodes to be set, e.g. nodes that are not considered in A* pathfinding routes.
	*/
	private static int[][] offlineNodes = new int[][]{{12, 6}, {12, 9}, {12, 11},
												{11, 6}, {11, 9}, {11, 12},
												{10, 5}, {10, 6}, {10, 9}, {10, 13},
												{9, 4}, {9, 6}, {9, 9},{9, 14},
												{8, 3}, {8, 15},
												{7, 2}, {7, 7}, {7, 11}, {7, 16},
												{6, 1}, {6, 7}, {6, 8}, {6, 9}, {6, 10}, {6, 11}, {6, 17},
												{5, 2}, {5, 7}, {5, 9}, {5, 11}, {5, 16},
												{4, 3}, {4, 9}, {4, 15},
												{3, 4}, {3, 9}, {3, 14},
												{2, 5}, {2, 9}, {2, 13},
												{1, 6}, {1, 9}, {1, 12},
												{0, 7}, {0, 9}, {0, 11}};
	
	public Map(int y, int x, int[][] offlineNodes){
		this.mapArea = new Node[y][x];
		generateNodeNetwork();
		turnNodesOff(offlineNodes);
	}
	
	/** 
	* J: This method specifies the map area (13 by 19 blocks).
	*/
	public Map(){
		this(13, 19, offlineNodes);
	}
	
	/** 
	* J: This method goes through nodes on map and generate their Manhattan Heuristics to a specified destination node
	* @param destination
	*/ 
	public void setHeuristic(Node destination) {
        for (int i = mapArea.length-1; i >= 0 ; i--) {
            for (int j = 0; j < mapArea[0].length; j++) {
            	this.mapArea[i][j].generateManhattanHeuristic(destination);
            }
        }
    }
	
	
	/** 
	*  J: the showMap methods print the map/routes to console visually. Very useful in error diagnostic and visualising
	*  the map scenario.
	*/
	public void showMap(){
    	for (int i = mapArea.length-1; i >= 0 ; i--){
    		if(i/10 >= 1){System.out.print(i);}
    		else{System.out.print(i+" ");}
    		
            for (int j = 0; j < mapArea[0].length; j++){
            	if(mapArea[i][j].isNodeOffline()){System.out.print(" B ");}
            	else{System.out.print(" - ");}
            }
            System.out.println();
        }
    	System.out.print("  ");
    	for (int j = 0; j < mapArea[0].length; j++){
    		if(j/10 >= 1){System.out.print(j+" ");}
    		else{System.out.print(" "+j+" ");}
    	}
    }
	
	public void showMap(List<Node> path){
    	for (int i = mapArea.length-1; i >= 0 ; i--){
    		if(i/10 >= 1){System.out.print(i);}
    		else{System.out.print(i+" ");}
    		
            for (int j = 0; j < mapArea[0].length; j++){
            	if(mapArea[i][j].isNodeOffline()){System.out.print(" B ");}
            	else if(path.contains(mapArea[i][j])){System.out.print(" * ");}
            	else{System.out.print(" - ");}
            }
            System.out.println();
        }
    	System.out.print("  ");
    	for (int j = 0; j < mapArea[0].length; j++){
    		if(j/10 >= 1){System.out.print(j+" ");}
    		else{System.out.print(" "+j+" ");}
    	}
    }
	
	public Node[][] getMap(){
		return mapArea;
	}
	
	
	/** 
	* J: This method brings a specified node offline on the map by setting its offline state to true. Removes nodes consideration by A*.
	* @param element
	*/

	public void setNodeOffline(Node element){
		this.mapArea[element.getX()][element.getY()].setOffline(true);
	}
	
	/** 
	* J: This method brings a specified node back online for consideration by A* by setting its offline status state to false.
	* @param element
	*/
	public void bringNodeOnline(Node element){
		this.mapArea[element.getX()][element.getY()].setOffline(false);
	}
	
	/** 
	* J: This method generates THIS maps network of nodes using dimensions specified above
	*/
	private void generateNodeNetwork() {
        for (int i = mapArea.length-1; i >= 0 ; i--) {
            for (int j = 0; j < mapArea[0].length; j++){
                Node node = new Node(i, j);
                this.mapArea[i][j] = node;
            }
        }
    }
	
	/** 
	* J: This method takes an array encoding nodes on the map and sets their offline status to True.
	* @param arrayOfNodes
	*/
	private void turnNodesOff(int[][] arrayOfNodes) {
        for (int i = 0; i < arrayOfNodes.length; i++) {
            int y = arrayOfNodes[i][0];
            int x = arrayOfNodes[i][1];
            this.mapArea[y][x].setOffline(true);
        }
    }
}
