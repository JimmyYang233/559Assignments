package comp559.particle;

public class Midpoint implements Integrator {

    @Override
    public String getName() {
        return "midpoint";
    }

    private double[] tmp;
    
    @Override
    public void step(double[] p, int n, double t, double h, double[] pout, Function derivs) {
        // TODO: Objective 4, implement midpoint method

    	// You will probably want a temporary array in this method and perhaps
    	// multiple temporary arrays in other higher order explicit integrators.
    	// Avoid thrashing memory by reallocating only when necessary.
    	if ( tmp == null || tmp.length != n ) {
            tmp = new double[n];
    	}
    	
    }

}
