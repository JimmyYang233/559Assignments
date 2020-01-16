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

    }

}
