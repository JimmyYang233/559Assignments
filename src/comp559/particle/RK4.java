package comp559.particle;

public class RK4 implements Integrator {
    
    @Override
    public String getName() {
        return "RK4";
    }

    @Override
    public void step(double[] p, int n, double t, double h, double[] pout, Function derivs) {
        // TODO: Objective 6, implement the RK4 integration method
    	// see also efficient memory management suggestion in provided code for the Midpoint method.

    }
}
