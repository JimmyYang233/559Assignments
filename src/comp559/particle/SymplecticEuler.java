package comp559.particle;

public class SymplecticEuler implements Integrator {

    @Override
    public String getName() {
        return "symplectic Euler";
    }

    @Override
    public void step(double[] p, int n, double t, double h, double[] pout, Function derivs) {
        // TODO: Objective 7, complete the symplectic Euler integration method.
    	// note you'll need to know how p is packed to properly implement this, so go
    	// look at ParticleSystem.getPhaseSpace()
    	double[] dpdt = new double[n];
    	derivs.derivs(t, p, dpdt);
    	int count = 0;
    	while(count<n)
    	{
    		pout[count+2] = p[count+2]+h*dpdt[count+2];
    		pout[count+3] = p[count+3]+h*dpdt[count+3];
    		pout[count] = p[count]+pout[count+2];
    		pout[count+1] = p[count+1] + pout[count+3];
    		count= count+4;
    	}
    	for(int i = 0; i<n; i++)
    	{
    		pout[i] = p[i]+h*dpdt[i];
    	}

    }

}
