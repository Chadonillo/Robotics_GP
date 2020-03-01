package T2;

import java.util.List;

public class Map {
	private Node[][] mapArea;
	private static int[][] blocks = new int[][]{{12, 6}, {12, 9}, {12, 11},
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
	
	public Map(int y, int x, int[][] blockedNodes){
		this.mapArea = new Node[y][x];
		setNodes();
		setBlocks(blockedNodes);
	}
	
	public Map(){
		this(13, 19, blocks);
	}

	public void setHeuristic(Node finalNode) {
        for (int i = mapArea.length-1; i >= 0 ; i--) {
            for (int j = 0; j < mapArea[0].length; j++) {
            	this.mapArea[i][j].calculateHeuristic(finalNode);
            }
        }
    }

	public void showMap(){
    	for (int i = mapArea.length-1; i >= 0 ; i--){
    		if(i/10 >= 1){System.out.print(i);}
    		else{System.out.print(i+" ");}
    		
            for (int j = 0; j < mapArea[0].length; j++){
            	if(mapArea[i][j].isBlock()){System.out.print(" B ");}
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
            	if(mapArea[i][j].isBlock()){System.out.print(" B ");}
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
	
	public void addBlockedNode(Node element){
		this.mapArea[element.getRow()][element.getCol()].setBlock(true);
	}
	
	public void removeBlockedNode(Node element){
		this.mapArea[element.getRow()][element.getCol()].setBlock(false);
	}
	
	private void setNodes() {
        for (int i = mapArea.length-1; i >= 0 ; i--) {
            for (int j = 0; j < mapArea[0].length; j++){
                Node node = new Node(i, j);
                this.mapArea[i][j] = node;
            }
        }
    }
	
	private void setBlocks(int[][] blocksArray) {
        for (int i = 0; i < blocksArray.length; i++) {
            int y = blocksArray[i][0];
            int x = blocksArray[i][1];
            this.mapArea[y][x].setBlock(true);
        }
    }
}
