package T2;
//Main contributors to concepts/code: Brandon, James, Avinash, Sean.

//James's comments marked with @J
public class TheStrip {
    
    //@J we define an array to store as sequence of bayesian probablities for the locations on the strip.
    private double[] bayesianProbs;
    private final boolean[] squareIsBlue;
    //@J we define another boolean array for the strip itself. The strip is encoded within boolean values 
    // i.e. moving each small block unit from one end to another (around 3cm), we ask the question "is the 
    // strip blue here?" If the answer is true (i.e. it the strip is blue) we append true to the array (resulting in
    // the below being initialised within the class constructor).
    
    // The two arrays above throughout should be seen to correspond to one another, i.e. we are defining the physical 
    // strip itself encoded with booleans and a corresponding array of an identical length with probabilties that
    // relate to those positions.
    
    public TheStrip() {
    	squareIsBlue = new boolean[] {
                false, false, true, true ,true,
                false, true, true, false, false,
                true, true, true, false, true, 
                true, false, false, true, true, 
                true, false, false, true, true, 
                true, false, true, true, false, 
                false, true, true, true, false, 
                true, true};
        bayesianProbs = new double[squareIsBlue.length];
        //@J We initialise the array of bayesian probabilities to contain elements equal to the length of the strip.
        // this makes sense as bayesian probability requires an associated probability for each location on the strip
        // the robot can be.
    
        //@J In the bayesian probability array, we now set the probability the sensor is positioned within the first 8 blocks
        // as 0 (it is impossible for the light sensor to read these positions if the robot is placed on the strip facing forward)
        // This makes sense, because the robot, being oriented forward is 8 blocks long itself, meaning the minimum
        // position for the wall-z's sensor could be (when he is placed at random) is the 8th block.
        for(int i = 0; i < 9; ++i){
            bayesianProbs[i] = 0;
        }
        //@J Before the robot has even been positioned, we have eliminated 8 positions the robot's sensor could have read,
        // because the total probability that he is on the strip is known to be 100% (given we have placed him somewhere on the strip)
        // we now distribute this remaining probability of 1 between all the other positions on the strip that are left.
        // Since the strip is composed of 37 boolean blocks, and we have removed 8 of them, we now distribute that 100% over
        // the remaining 29 giving wall-z from the outset an approximate 3% chance of being on any individual of those 29 blocks.
        for(int i = 9; i < bayesianProbs.length; ++i){
            bayesianProbs[i] = 1.0 / (squareIsBlue.length - 9);
            
        }
    }
    
    // @J This method sets the bayesian probabilities for each index in the array as wall-z physically moves/iterates through 
    // the strip observing whether it is blue or not with a light sensor, it then compares what it sees to the the known encoded
    // boolean array of the strip. Using bayesian mathematics the probabilties for each location in the probability array are deduced.
    public void setBayesianProbabilities(boolean movedFoward, boolean readBlue, double sensorProba, double moveProba) {
        double normalization = 0.0;
        for(int i = 0; i < bayesianProbs.length; ++i) {
            if(squareIsBlue[i] == readBlue) {
                bayesianProbs[i] *= sensorProba;
            }
            else {
                bayesianProbs[i] *= 1 - sensorProba;
            }
            normalization += bayesianProbs[i];
        }
        normalize(normalization);

        normalization = 0.0;

        if(movedFoward) {

            for (int i = bayesianProbs.length - 1; i > 8; --i) {
                bayesianProbs[i] = bayesianProbs[i - 1] * moveProba + bayesianProbs[i] * (1 - moveProba);
                normalization += bayesianProbs[i];
            }

        } 
        else {
            for (int i = 9; i < bayesianProbs.length - 1; ++i) {
                bayesianProbs[i] = bayesianProbs[i + 1] * moveProba + bayesianProbs[i] * (1 - moveProba);
                normalization += bayesianProbs[i];
            }
            bayesianProbs[bayesianProbs.length - 1]=0;
        }
        
        normalize(normalization);
    }
    
    //@J it is required that for the array of bayesian probabilities, the total of all must sum to 1.
    // as the robot moves through the strip, and probabilities are updated, they must also be normalised
    // (still all add to total of 1). This function helps to accomplish this, by iterating through the elements in the
    // bayesian probability array, and dividing them by the normalization value. The normalization value is derived
    // from the above **setBayesianProbabilities method**.
    private void normalize(double normalization) {
        for(int i = 0; i < bayesianProbs.length; ++i) {
            bayesianProbs[i] /= normalization;
        }
    }
    
    
    //@J getLocation is a method that 
    public int getLocation() {
        double max = bayesianProbs[bayesianProbs.length-1];
        int index = bayesianProbs.length-1;
        for(int i = bayesianProbs.length-1; i >= 0; --i) {
            if(bayesianProbs[i] > max) {
                max = bayesianProbs[i];
                index = i;
            }
        }
        return index;

    }

    public double getHighestProbability() {
        return bayesianProbs[getLocation()];
    }
    
    //@J getProbabilty returns the probability for a location of the probability array specificied by passing an index.
    public double getProbability(int pos) {
        return bayesianProbs[pos-1];
    }
    
    public void resetProbs(){
        for(int i = 0; i < 9; ++i){
            bayesianProbs[i] = 0;
        }
        for(int i = 9; i < bayesianProbs.length; ++i){
            bayesianProbs[i] = 1.0 / (squareIsBlue.length - 9);
        }
    }
}
