package T2;

import java.util.List;

public class Map {
	private Node[][] mapArea;
	private static int[][] blocks = new int[][]{{24, 13}, {24, 14}, {24, 15}, {24, 16},{24,17},{24,18},{24,19},{24,20},{24,21},{24,22},
												{23, 12}, {23, 13}, {23, 14}, {23, 23},
												{22, 11}, {22, 14}, {22, 15}, {22, 24},
												{21, 10}, {21, 15}, {21, 25},
												{20, 9}, {20, 15}, {20, 26},
												{19, 8}, {19, 14}, {19, 15}, {19, 27},
												{18, 7}, {18, 13}, {18, 14}, {18, 28},
												{17, 6}, {17, 12}, {17, 13}, {17, 29},
												{16, 5}, {16, 30},
												{15, 4}, {15, 31},
												{14, 3}, {14, 13}, {14, 32}, {14, 22},
												{13, 2}, {13, 13}, {13, 14}, {13, 21}, {13, 22},{13, 33},
												{12, 2}, {12, 13}, {12, 14}, {12, 15}, {12, 16}, {12, 17}, {12, 18}, {12, 19}, {12, 20}, {12, 21}, {12, 22},{12, 34},
												{11, 3}, {11, 13}, {11, 14}, {11, 18}, {11, 21}, {11, 22}, {11, 33},
												{10, 4}, {10, 18}, {10, 32}, {10, 13}, {10, 22},
												{9, 5}, {9, 18}, {9, 31},
												{8, 6}, {8, 18}, {8, 30},
												{7, 7}, {7, 18}, {7, 29},
												{6, 8}, {6, 18}, {6, 28},
												{5, 9}, {5, 18}, {5, 27},
												{4, 10}, {4, 18}, {4, 26},
												{3, 11}, {3, 18}, {3, 25},
												{2, 12}, {2, 18}, {2, 24},
												{1, 13}, {1, 18}, {1, 23},
												{0, 14}, {0, 15}, {0, 16}, {0, 17}, {0, 18}, {0, 19}, {0, 20}, {0, 21}, {0, 22}};
	
	public Map(int y, int x, int[][] blockedNodes){
		this.mapArea = new Node[y][x];
		setNodes();
		setBlocks(blockedNodes);
	}
	
	public Map(){
		this(25, 37, blocks);
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
