package a;

import java.util.ArrayList;

public class DequeMe {
	int mySize;
	ArrayList<Float> myList = new ArrayList<Float>();
	
	public DequeMe(int size){
		mySize=size;
	}
	
	public void add(float val){
		if(myList.size() >= mySize){
			myList.remove(0);
		}
		myList.add(val);
	}
	
	public boolean allValsLessThan(int max){
		for(int i=0; i<myList.size(); ++i){
			if(myList.get(i) > max){
				return false;
			}	
		}
		return true;
	}

}
