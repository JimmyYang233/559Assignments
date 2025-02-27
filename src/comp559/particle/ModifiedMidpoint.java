package comp559.particle;

public class ModifiedMidpoint implements Integrator {

    @Override
    public String getName() {
        return "modified midpoint";
    }

    @Override
    public void step(double[] p, int n, double t, double h, double[] pout, Function derivs) {
    	// TODO: Objective 5, implmement the modified midpoint (2/3) method.
    	// see also efficient memory management suggestion in provided code for the Midpoint method.
    	double[] tmp = new double[n];
    	if ( tmp == null || tmp.length != n ) {
            tmp = new double[n];
    	}
    	derivs.derivs(t, p, tmp);
    	for(int i = 0; i<n; i++)
    	{
    		tmp[i] = p[i]+2*h*tmp[i]/3;
    	}
    	double[] dpdt = new double[n];
    	derivs.derivs(t, tmp, dpdt);
    	for(int i = 0; i<n; i++)
    	{
    		pout[i] = p[i]+h*dpdt[i];
    	}
    }

}
