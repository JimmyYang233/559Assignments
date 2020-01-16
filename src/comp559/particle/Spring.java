package comp559.particle;

import javax.vecmath.Vector2d;

import no.uib.cipr.matrix.Matrix;
import no.uib.cipr.matrix.Vector;

/**
 * Spring class for 599 assignment 1
 * @author kry
 */
public class Spring {

    Particle p1 = null;
    Particle p2 = null;
    
    /** Spring stiffness, sometimes written k_s in equations */
    public static double k = 1;
    /** Spring damping (along spring direction), sometimes written k_d in equations */
    public static double c = 1;
    /** Rest length of this spring */
    double l0 = 0;
    
    /**
     * Creates a spring between two particles
     * @param p1
     * @param p2
     */
    public Spring( Particle p1, Particle p2 ) {
        this.p1 = p1;
        this.p2 = p2;
        recomputeRestLength();
        p1.springs.add(this);
        p2.springs.add(this);
    }
    
    /**
     * Computes and sets the rest length based on the original position of the two particles 
     */
    public void recomputeRestLength() {
        l0 = p1.p0.distance( p2.p0 );
    }
    
    /**
     * Applies the spring force by adding a force to each particle
     */
    public void apply() {
        // TODO: Objective 1, FINISH THIS CODE!
        
    }
   
    /** TODO: the functions below are for the backwards Euler solver */
    
    /**
     * Computes the force and adds it to the appropriate components of the force vector.
     * (This function is something you might use for a backward Euler integrator)
     * @param f
     */
    public void addForce( Vector f ) {
        // TODO: Objective 8, FINISH THIS CODE for backward Euler method (probably very simlar to what you did above)
        
    }
    
    /**
     * Adds this springs contribution to the stiffness matrix
     * @param dfdx
     */
    public void addDfdx( Matrix dfdx ) {
        // TODO: Objective 8, FINISH THIS CODE... necessary for backward euler integration
        
    }   
 
    /**
     * Adds this springs damping contribution to the implicit damping matrix
     * @param dfdv
     */
    public void addDfdv( Matrix dfdv ) {
        // TODO: Objective 8, FINISH THIS CODE... necessary for backward Euler integration
        
    } 
    
}
