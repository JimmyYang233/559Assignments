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
    	double[] k1 = new double[n];
    	double[] k11 = new double[n];
    	derivs.derivs(t, p, k1);
    	for(int i = 0; i<n; i++)
    	{
    		k11[i] = p[i]+h*k1[i]/2;
    	}
    	double[] k2 = new double[n];
    	double[] k22 = new double[n];
    	derivs.derivs(t, k11 , k2);
    	for(int i = 0; i<n; i++)
    	{
    		k22[i] = p[i]+h*k2[i]/2;
    	}
    	double[] k3 = new double[n];
    	double[] k33 = new double[n];
    	derivs.derivs(t, k22, k3);
    	for(int i = 0; i<n; i++)
    	{
    		k33[i] = p[i]+h*k3[i];
    	}
    	double[] k4 = new double[n];
    	derivs.derivs(t, k33, k4);
    	for(int i = 0; i<n; i++)
    	{
    		pout[i] = p[i] + h/6*(k1[i]+2*k2[i]+2*k3[i]+k4[i]);
    	}
    	
    }
}
