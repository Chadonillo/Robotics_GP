package localization;

public class TheStrip {
    private double[] bayesianProbs;
    private final boolean[] squareIsBlue;

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

        for(int i = 0; i < 9; ++i){
            bayesianProbs[i] = 0;
        }
        for(int i = 9; i < bayesianProbs.length; ++i){
            bayesianProbs[i] = 1.0 / (squareIsBlue.length - 9);
        }
    }
    
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

    private void normalize(double normalization) {
        for(int i = 0; i < bayesianProbs.length; ++i) {
            bayesianProbs[i] /= normalization;
        }
    }

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
