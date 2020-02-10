package T2;

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

            for (int i = bayesianProbs.length - 1; i > 0; --i) {
                bayesianProbs[i] = bayesianProbs[i - 1] * moveProba + bayesianProbs[i] * (1 - moveProba);
                normalization += bayesianProbs[i];
            }
            
            bayesianProbs[0] = bayesianProbs[bayesianProbs.length - 1] * moveProba + bayesianProbs[0] * (1 - moveProba);
            normalization += bayesianProbs[0];

        } 
        else {
            for (int i = 0; i < bayesianProbs.length - 1; ++i) {
                bayesianProbs[i] = bayesianProbs[i + 1] * moveProba + bayesianProbs[i] * (1 - moveProba);
                normalization += bayesianProbs[i];
            }
            bayesianProbs[bayesianProbs.length - 1] = bayesianProbs[0] * moveProba + bayesianProbs[bayesianProbs.length - 1] * (1 - moveProba);
            normalization += bayesianProbs[bayesianProbs.length - 1];
        }
        
        normalize(normalization);
    }



    private void normalize(double normalization) {
        for(int i = 0; i < bayesianProbs.length; ++i) {
            bayesianProbs[i] /= normalization;
        }
    }

    public int getLocation() {
        double max = bayesianProbs[0];
        int index = 0;
        for(int i = 1; i < bayesianProbs.length; ++i) {
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
}
