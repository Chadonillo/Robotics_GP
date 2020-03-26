package localization;
/**
* This class implements localization functionality. Specific detail is given within method descriptions
* rather than here, James's comments marked with J:
* 
* @author Brandon Cardillo, James Burroughs, Avinash Patel, Seokhwan Jung
*/

public class TheStrip { 
	/**
	* J: we define an array to store as sequence of bayesian probablities for the locations on the strip.
	*/
    private double[] proababilityHeIsAt;
	/**
	* J: we define an array to store locations on the strip.
	*/
    private final boolean[] squareIsBlue;
        /**
	 * J: here we define another boolean array for the strip itself. The strip is encoded within boolean values 
         * i.e. moving each small block unit from one end to another (around 3cm), we ask the question "is the 
         * strip blue here?" If the answer is true (i.e. it the strip is blue) we append true to the array (resulting in
         * the below being initialised within the class constructor).

         * The two arrays above throughout should be seen to correspond to one another, i.e. we are defining the physical 
         * strip itself encoded with booleans and a corresponding array of an identical length with probabilties that
         * relate to those positions.
	 */
    public TheStrip() {
    	squareIsBlue = new boolean[] {
             false, false, true, true ,true, false, true, true, false, false,
                true, true, true, false, true, true, false, false, true, true, 
                  true, false, false, true, true, true, false, true, true, false, 
                   false, true, true, true, false, true, true };
        proababilityHeIsAt = new double[squareIsBlue.length];
        
        //@J We initialise the array of bayesian probabilities to contain elements equal to the length of the strip.
        // this makes sense as bayesian probability requires an associated probability for each location on the strip
        // the robot can be.

        //@J In the bayesian probability array, we now set the probability the sensor is positioned within the first 8 blocks
        // as 0 (it is impossible for the light sensor to read these positions if the robot is placed on the strip facing forward)
        // This makes sense, because the robot, being oriented forward is 9 blocks long itself, meaning the minimum
        // position for the wall-z's sensor could be (when he is placed at random) is the 9th block.
        
        for(int i = 0; i < 9; ++i){
            proababilityHeIsAt[i] = 0;
        }
        
        //@J Before the robot has even been positioned, we have eliminated 9 positions the robot's sensor could have read,
        // because the total probability that he is on the strip is known to be 100% (given we have placed him somewhere on the strip)
        // we now distribute this remaining probability of 1 between all the other positions on the strip that are left.
        // Since the strip is composed of 37 boolean blocks, and we have removed 9 of them, we now distribute that 100% over
        // the remaining 28 giving wall-z from the outset an approximate 3% chance of being on any individual of those 28 blocks.
        
        for(int i = 9; i < proababilityHeIsAt.length; ++i){
            proababilityHeIsAt[i] = 1.0 / (squareIsBlue.length - 9);
        }
    }
	
	/**
	* J: This method sets the bayesian probabilities for each index in the array as wall-z physically moves/iterates through 
        * the strip observing whether it is blue or not with a light sensor, it then compares what it sees to the the known encoded
        * boolean array of the strip. Using bayesian mathematics the probabilties for each location in the probability array are deduced.
	* @param wallzGoesAhead
	* @param wallzSensedBlue
	*/
    public void updateProbablityMap(boolean wallzGoesAhead, boolean wallzSensedBlue) {
        double normalisationCoefficient = 0;
        double hardwarePrecision = 0.94;
        for(int i = 0; i < proababilityHeIsAt.length; ++i) {
            if(squareIsBlue[i] == wallzSensedBlue) {
                proababilityHeIsAt[i] *=hardwarePrecision;
            }
            else {
                proababilityHeIsAt[i] *= 1 -hardwarePrecision;
            }
            normalisationCoefficient +=proababilityHeIsAt[i];
        }
        normalizeValues(normalisationCoefficient);
        normalisationCoefficient = 0;
        
        if(wallzGoesAhead) {
            for (int i = proababilityHeIsAt.length - 1; i > 8; --i) {
                proababilityHeIsAt[i] = proababilityHeIsAt[i-1];
                normalisationCoefficient += proababilityHeIsAt[i];
            }
        } 
        else {
            for (int i = 9; i < proababilityHeIsAt.length - 1; ++i) {
                proababilityHeIsAt[i] = proababilityHeIsAt[i + 1];
                normalisationCoefficient += proababilityHeIsAt[i];
            }
            proababilityHeIsAt[proababilityHeIsAt.length - 1]=0;
        }
        normalizeValues(normalisationCoefficient);
    }
	
    /**
    * J: it is required that for the array of bayesian probabilities, the total of all must sum to 1.
    * as the robot moves through the strip, and probabilities are updated, they must also be normalised
    * (still all add to total of 1). This function helps to accomplish this, by iterating through the elements in the
    * bayesian probability array, and dividing them by the normalization value. The normalization value is derived
    * from the above updateProbabilityMap method.
    * @param normalizationCoefficient
    */
    private void normalizeValues(double normalizationCoefficient) {
        for(int i = 0; i < proababilityHeIsAt.length; ++i) {
            proababilityHeIsAt[i] /=normalizationCoefficient;
        }
    }
    /**
    * Gets the probability of the likely position
    * @return probablity value (double)
    */
    public double getProbabilityOfLikelyPosition() {
        return proababilityHeIsAt[getLikelyPosition()];
    }
	
    /**
    * Gets the likly position
    * @return position
    */
    public int getLikelyPosition() {
        double limit = proababilityHeIsAt[proababilityHeIsAt.length-1];
        int position = proababilityHeIsAt.length-1;
        for(int i = proababilityHeIsAt.length-1; i >= 0; --i) {
            if(proababilityHeIsAt[i] > limit) {
                limit = proababilityHeIsAt[i];
                position = i;
            }
        }
        return position;
    }
	
    /**
    * getProb returns the probability for a location of the probability array specificied by passing an index.
    * @return probablity value (double)
    */
    public double getProb(int pos) {
        return proababilityHeIsAt[pos-1];
    }
	
    /**
    * resetProbs resets the probabilities
    */
    public void resetProbs(){
        for(int i = 0; i < 9; ++i){
            proababilityHeIsAt[i] = 0;
        }
        for(int i = 9; i < proababilityHeIsAt.length; ++i){
            proababilityHeIsAt[i] = 1.0 / (squareIsBlue.length - 9);
        }
    }
}
