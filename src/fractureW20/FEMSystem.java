package fractureW20;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;

import javax.swing.JPanel;
import javax.swing.JTextArea;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.CollapsiblePanel;
import mintools.swing.HorizontalFlowPanel;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.SceneGraphNode;
import no.uib.cipr.matrix.DenseVector;
import no.uib.cipr.matrix.Vector;

/**
 * Implementation of a simple particle system.
 * 
 * Note that the particle system implements Function, that is, it evaluates
 * its derivatives to return to the step method which is called by implementations
 * of the Integrator interface.
 * 
 * Note also that it is actually the updateParticles method in this class which 
 * should be calling the Integrator step method! 
 * 
 * @author kry
 */
public class FEMSystem implements SceneGraphNode, Filter, MatrixMult {
    
    /** the particle list */
    public List<Particle> particles = new LinkedList<Particle>();
    
    /** particles for applying traction (i.e., bottom row of block tests) */
    public List<Particle> tractionParticles = new LinkedList<Particle>();

    /** the spring list for collisions */
    public List<Edge> collidableEdges = new LinkedList<Edge>();
    
    /** leaf springs connect 3 particles with a hinge like spring */
    public List<FEMTriangle> femSprings = new LinkedList<FEMTriangle>();
    
    /** List of collision objects **/
    public List<Collision> collisionList = new LinkedList<Collision>();
      
    /** The name of the particle system. */
    public String name = "";
    
    MouseSpring mouseSpring;
    boolean useMouseSpring = false;
    
    
// visualizaiton related parameters
    
    BooleanParameter drawParticles = new BooleanParameter( "draw particles", true ) ;
    BooleanParameter drawCollisions = new BooleanParameter( "draw collisions", true ) ;
    BooleanParameter drawCollisionBondary = new BooleanParameter( "draw collision bondary", false ) ;
    DoubleParameter pointSize = new DoubleParameter("point size", 2, 1, 25);
    DoubleParameter stressEigVecScale = new DoubleParameter("stress eigenvector viz scale", 0.1, 1e-3, 1e3 );
    DoubleParameter strainEigVecScale = new DoubleParameter("strain eigenvector viz scale", 1e3, 1, 1e6 );
    DoubleParameter stScale = new DoubleParameter("separation tessor viz scale", 1e-7, 1e-10, 1 );
    BooleanParameter drawSeparationTensor = new BooleanParameter("draw separation tensor", false );
    BooleanParameter drawStrainTensor = new BooleanParameter("draw strain tensor", false );
    BooleanParameter drawStressTensor = new BooleanParameter("draw stress tensor", false );
    BooleanParameter drawElementBoundaries = new BooleanParameter( "draw element boundaries", false );
    BooleanParameter drawBoundaryEdges = new BooleanParameter( "draw boundary edges", true );
    DoubleParameter transparency = new DoubleParameter("triangle transparency", 0.5, 0, 1 );
    JTextArea comments = new JTextArea("<comments>");
    BooleanParameter showCommentsAndParameters = new BooleanParameter("show comments and parameters", true );
    
    // simulation related parameters
    
    /** 
     * Irving presents a solution for inverting elements, thus this makes an interesting
     * alternative material (not part of this assignment)
     */
    BooleanParameter useIrving2004 = new BooleanParameter( "use Irving 2004 (otherwise StVK)", false );
    BooleanParameter implicit = new BooleanParameter( "Implicit integration", true );
    IntParameter newtonIterations = new IntParameter( "Newton root solve iterations", 1, 1, 20 );
    IntParameter cgIterations = new IntParameter( "CG solve iterations", 100, 20, 300 );
    BooleanParameter doCollisionDetection = new BooleanParameter( "do collisions", true ) ;
    BooleanParameter useg = new BooleanParameter( "use gravity", true );
    DoubleParameter g = new DoubleParameter( "gravity", 98, 0.01, 1000 );
    DoubleParameter mouseSpringk = new DoubleParameter( "mouse spring k", 1000, 1, 1e5 );
    DoubleParameter PoissonRatio  = new DoubleParameter( "Poisson Ratio", 0.3, 0, .4999 );
    DoubleParameter YoungModulus = new DoubleParameter( "YoungModulus", 50000, 1, 1e10 );
    DoubleParameter toughness = new DoubleParameter("material toughness", 1e5, 1, 1e8 );
    DoubleParameter RayleighAlpha = new DoubleParameter("Raleigh alpha", 1e-1, 1e-3, 1e3 );
    DoubleParameter RayleighBeta = new DoubleParameter("Raleigh beta", 1e-2, 1e-3, 1e3 );
    BooleanParameter useRAlpha = new BooleanParameter("use Raleigh alpha", true );
    BooleanParameter useRBeta = new BooleanParameter("use Raleigh beta", true );
    
    /** coefficient for viscous resolution of collision */
	DoubleParameter collisionViscousCoefficient = new DoubleParameter( "collision viscous coefficient", 5e7, 1, 5e8 );
    /** relative normal velocity threshold for using damping based collision */
	DoubleParameter relNormalVelThresh = new DoubleParameter( "relative normal velocity threshold", 100, 1e-3, 1e3 );
	/** Could be area weighted... was (2.0 * area + 50); */
	DoubleParameter collisionRepulsion = new DoubleParameter( "collision repulsion (buggy)", 0, 1e-3, 1e3 ); 
    
    
    /**
     * Creates an empty particle system
     */
    public FEMSystem() {
    	mouseSpring = new MouseSpring();
    }
    
    /**
     * Resets the positions of all particles to their initial states.
     * Probably don't want to use this after fracture events as the
     * original fracture state will not be restored
     */
    public void resetParticles() {
    	collisionList.clear();
        for ( Particle p : particles ) {
            p.reset();
        }
        for ( FEMTriangle tri : femSprings ) {
        	tri.reset(); // try to get rid of NaNs
        }
        time = 0;
    }
    
    /**
     * Deletes all particles, and as such removes all springs too.
     */
    public void clear() {      
    	collisionList.clear();
        particles.clear();
        tractionParticles.clear();
        collidableEdges.clear();
        femSprings.clear();
        name = "";
    }    
    
    public double time = 0;
    
    private double stepSize;
    

    /**
     * Identifies the boundary edges of the current set of FEM triangles.
     * The method loops over all triangles adding edges to a set, and then 
     * removing those edges if the edge is found again in an adjacent triangle. 
     * This is needed on loading a new system.  You'll want to do something 
     * more efficient on fixing up boundaries after fracture events.
     */
    void identifyBoundaries() {
    	collidableEdges.clear();
		final HashSet<Edge> boundaries = new HashSet<Edge>();
		boundaries.clear();				
		for ( FEMTriangle t : femSprings ) {
			Edge e1 = new Edge( t.A, t.B, t );
			if ( ! boundaries.remove( e1 ) ) boundaries.add( e1 );
			Edge e2 = new Edge( t.B, t.C, t );
			if ( ! boundaries.remove( e2 ) ) boundaries.add( e2 );
			Edge e3 = new Edge( t.C, t.A, t );
			if ( ! boundaries.remove( e3 ) ) boundaries.add( e3 );
		}
		collidableEdges.addAll( boundaries ); 
    }
    

    
	/**
	 * Comptues product with A = M - h (alpha M + beta K) - h^2 K
	 * But note there is an extra contribution for collision processing
	 */
	@Override
    public void mult(Vector v, Vector Av) {

		// TODO: Objective 2: implicit integration matrix multiply	
		
		int N = particles.size();
		double alpha = RayleighAlpha.getValue();
		double beta = RayleighBeta.getValue();
		double h = stepSize;
		Vector Kv = new DenseVector(2*N);
		int i = 0;
		for(Particle p : particles)
		{
			Av.set(i, v.get(i)*(p.mass*(1-alpha*h)));
			Av.set(i+1, v.get(i+1)*(p.mass*(1-alpha*h)));
			p.dx.x = v.get(i);
			p.dx.y = v.get(i+1);
        	p.df.set(0,0);
			i= i+2;
		}
		for(FEMTriangle tri : femSprings)
		{
			tri.computeForceDifferentials();
		}
		i = 0;
		double tmp = beta*h+h*h;
		for(Particle p : particles)
		{
			Kv.set(i, tmp*(p.df.x));
			Kv.set(i+1, tmp*(p.df.y));
			i = i+2;
		}
		Av.add(-1, Kv);
		
		for (Collision c : collisionList) {
			c.compute_dvDotResponse(Av, v, stepSize);
		}
	}

	@Override
	public void filter(Vector v) {
		int i = 0;
		for ( Particle p : particles ) {
			if ( p.pinned ){
            	v.set( i+0, 0 );
            	v.set( i+1, 0 );
            }
			i += 2;
        }
	}

	
	boolean isHingeJoint(Particle p) 
	{
		int triangleSize = p.tris.size();
		ArrayList<Particle> connectedP = new ArrayList<Particle>();
		Particle pp = null;
		for(FEMTriangle tri : p.tris)
		{
			pp = tri.A;
			if(pp.equals(p))
			{
				Particle pp2 = tri.B;
				if(!connectedP.contains(pp2))
				{
					connectedP.add(pp2);
				}
				Particle pp3 = tri.C;
				if(!connectedP.contains(pp3))
				{
					connectedP.add(pp3);
				}
			}
			pp = tri.B;
			if(pp.equals(p))
			{
				Particle pp2 = tri.A;
				if(!connectedP.contains(pp2))
				{
					connectedP.add(pp2);
				}
				Particle pp3 = tri.C;
				if(!connectedP.contains(pp3))
				{
					connectedP.add(pp3);
				}
			}
			pp = tri.C;
			if(pp.equals(p))
			{
				Particle pp2 = tri.B;
				if(!connectedP.contains(pp2))
				{
					connectedP.add(pp2);
				}
				Particle pp3 = tri.A;
				if(!connectedP.contains(pp3))
				{
					connectedP.add(pp3);
				}
			}
		}
		int particlesShareEdge = connectedP.size();
		if(triangleSize+1<particlesShareEdge)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	public void restoreParticle(Particle theP, Particle newP)
	{
		//has hinge joint, remove the added one
        for(FEMTriangle tri : newP.tris)
        {
			if(tri.A.equals(newP))
    		{
    			tri.A = theP;
    			tri.Ai = theP.addTriangle(tri);
    		}
    		else if(tri.B.equals(newP))
    		{
    			tri.B = theP;
    			tri.Bi = theP.addTriangle(tri);
    		}
    		else if(tri.C.equals(newP))
    		{
    			tri.C = theP;
    			tri.Ci = theP.addTriangle(tri);
    		}
    		else
    		{
    			System.out.println("Something is wrong, one of the particle in triangle must be equal");
    		}
			particles.remove(newP);
        }
	}

    
    /**
     * Process the fracture based on particles using the separation tensors.
     * The largest eigenvector that exceeds the toughness determines the 
     * line of separation. 
     */
    void processFracture() {
        // compute the separation tensors at each node
        for ( Particle p : particles ) {
        	if ( p.tris.size() == 0 ) continue;
        	p.computeSeparationTensor();
        }
        
        double t = toughness.getValue();
        ArrayList<Particle> pThatConsidered = new ArrayList<Particle>();
        boolean notYetFinish = true;
        while(notYetFinish)
        {
            Particle theP = null;
            Vector2d theV = new Vector2d();
        	for(Particle p : particles )
            {
            	if(!p.pinned)
            	{
            		double bigev = Math.max(p.separationTensor.ev1, p.separationTensor.ev2);
            		Vector2d bigv = p.separationTensor.ev1>p.separationTensor.ev2?
            				p.separationTensor.v1:p.separationTensor.v2;
            		if(bigev>t)
            		{
            			t = bigev;
            			if(!pThatConsidered.contains(p))
            			{
            				theP = p;
                			theV = bigv;
            			}        			
            		}
            	}
            }

            if(theP!=null)
            {
            	notYetFinish = false;
            	Particle newP = new Particle(theP);
                newP.index = particles.size();
                particles.add(newP);
                ArrayList<FEMTriangle> trisToRemove = new ArrayList<FEMTriangle>();
                for(FEMTriangle tri : theP.tris)
                {
                	Point2d a= tri.getCentroid();
                	Vector2d n = theV; 
                	Point2d p = theP.p;
                	Vector2d aminusp = new Vector2d(a.x-p.x, a.y-p.y);
                	double result = n.dot(aminusp);
                	if(result>0)
                	{
                		//do nothing keep triangle connect to it
                	}
                	else
                	{
                		if(tri.A.equals(theP))
                		{
                			tri.A = newP;
                			tri.Ai = newP.addTriangle(tri);
                		}
                		else if(tri.B.equals(theP))
                		{
                			tri.B = newP;
                			tri.Bi = newP.addTriangle(tri);
                		}
                		else if(tri.C.equals(theP))
                		{
                			tri.C = newP;
                			tri.Ci = newP.addTriangle(tri);
                		}
                		else
                		{
                			System.out.println("Something is wrong, one of the particle in triangle must be equal");
                		}
                		trisToRemove.add(tri);
                	}
                }
                for(FEMTriangle triToRemove : trisToRemove)
                {
                	theP.tris.remove(triToRemove);
                }
                if(newP.tris.isEmpty())
                {
                	particles.remove(newP);
                }
                
                if(theP.tris.isEmpty())
                {
                	particles.remove(theP);
                }
                identifyBoundaries();
                boolean particleRestored = false;
        		for(FEMTriangle theTri : newP.tris)
        		{
        			if(isHingeJoint(theTri.A)||isHingeJoint(theTri.B)|| isHingeJoint(theTri.C))
        			{
        				notYetFinish = true;
        				restoreParticle(theP, newP);
        				particleRestored = true;
        				pThatConsidered.add(theP);
        	            break;
        			}
        		}
        		
        		if(!particleRestored)
        		{
        			for(FEMTriangle theTri : theP.tris)
            		{
            			if(isHingeJoint(theTri.A)||isHingeJoint(theTri.B)|| isHingeJoint(theTri.C))
            			{
            				notYetFinish = true;
            				restoreParticle(theP, newP);
            				particleRestored = true;
            				pThatConsidered.add(theP);
            	            break;
            			}
            		}
        		}
        		identifyBoundaries();
            }
            else
            {
            	notYetFinish = false;
            }
        }
        
              
        // TODO: Objective 4: process fracture
        // Note that you can use identifyBoundaries() as a slow way to update the border edges
        // after modifying the topology.    
        
    }

    DenseVector xdot;
    DenseVector xz;
    DenseVector deltax;
    DenseVector deltaxdot;
    DenseVector f;
    DenseVector rhs;
    ConjugateGradientMTJ cgMTJ;
    
    /**
     * Initializes variables for backward Euler integration.
     * You may not actually need all these.  Rename them as you like
     */
    public void init() {
        int N = particles.size();
        if ( xdot == null || xdot.size() != 2*N ) {
        	xdot = new DenseVector( 2*N );
        	xz = new DenseVector( 2*N );
        	deltax = new DenseVector( 2*N );
        	deltaxdot = new DenseVector( 2*N );
        	f = new DenseVector( 2*N );
            rhs = new DenseVector( 2*N );
        	cgMTJ = new ConjugateGradientMTJ( 2*N );
            cgMTJ.setFilter(this);
        }
    }
    
    /**
     * Computes all forces acting on the degrees of freedom in the system.
     */
    private void computeForces() {
    	// gravity
        for ( Particle p : particles ) {
            p.f.set( 0, useg.getValue() ? g.getValue() * p.mass : 0 );                      
        }
        
        for (Collision c : collisionList) {
        	c.applyForce( stepSize, implicit.getValue() );
        }
        
        if ( useMouseSpring ) {
        	mouseSpring.apply();
        }

        // TODO: Objective 1: apply the forces of the FEMSprings
        
        for(FEMTriangle tri : femSprings)
        {
        	tri.applyForce();
        }     
        
        // TODO: Objective 2: explicit parts of the Rayleigh damping computation are needed here
        // use the computeForceDifferentials method of the the FEMTriangles, but note that you 
        // must set the particle dx parameter first, and initialize the particle df to zero as
        // differentials will be accumulated for all triangles adjacent to the particle.
        for(int i = 0; i<particles.size()*2; i+=2)
        {
        	Particle p = particles.get(i/2);
        	p.dx.x = p.v.x*stepSize;
        	p.dx.y = p.v.y*stepSize;
        	p.df.set(0,0);
        }

        
        for(FEMTriangle tri : femSprings)
        {
        	tri.computeForceDifferentials();
        }  
    }
    
    
    /**
     * Advances time and updates the position of all particles
     * @param h 
     * @return true if update was successful
     */
    public boolean updateParticles( double h ) {
        boolean resultOK = true;
        Vector2d tmp = new Vector2d();
        stepSize = h;

        // set the global element properties
        double E = YoungModulus.getValue();
        double nu = PoissonRatio.getValue();
        FEMTriangle.mu = E/2/(1+nu);
        FEMTriangle.lambda = E*nu/(1+nu)/(1-2*nu);
        mouseSpring.k = mouseSpringk.getValue();
        
        collisionList.clear();
        
        // ensure proper indexing for global deltaV vector
        for (int index = 0; index < particles.size(); index++) {
        	particles.get(index).index = index;
        }
          
        if ( doCollisionDetection.getValue() ) {
	        // process collision
        	// Note that this would be better as a loop over all boundary vertices
        	for (int i = 0; i < collidableEdges.size(); i++) {
        		FEMTriangle t1 = collidableEdges.get(i).triangle;
	        	for (int j = i + 1; j < collidableEdges.size(); j++) {
	        		FEMTriangle t2 = collidableEdges.get(j).triangle;
	        		double eps = 1e-10;
	        		Collision c = Collision.collisionDetect( t1, t2, eps );
	        		if ( c != null ) {
	        			c.coefficent_viscous = collisionViscousCoefficient.getValue();
	        			c.relNormalVelThres = relNormalVelThresh.getValue();
	        			c.coefficent_repulsive = collisionRepulsion.getValue(); 
	        			c.updateCollisionResponse();
	        			collisionList.add( c );
	        		}	        		
	        	}
	        }        	
        }
               		
        // Update the velocity of the particles as per symplectic Euler
        if ( ! implicit.getValue() ) {
            computeForces();
	        for ( Particle p : particles ) {
	            if ( p.pinned ) {            
	                p.f.set(0,0); // just to make sure!
	                p.v.set(0,0);
	            } else {
	                tmp.scale( h / p.mass, p.f );
	                p.v.add( tmp );            
	            }
	        }
        } else {
        	
        	// TODO: Objective 2: advance the state of the system with implicit integration
        	// it is fine to do only 1 linear solve rather than newton iterations (i.e.,
        	// you can igore the newtonIterations parameter)
        	
        	init(); // initialize useful members you want to use here based on the number of DOFs
            getVelocity(xdot);
        	computeForces();
            int N = particles.size();
            double alpha = RayleighAlpha.getValue();
            double beta = RayleighBeta.getValue();
            
            rhs = new DenseVector(xdot);
            for(int i = 0; i<particles.size()*2; i+=2)
            {
            	Particle p = particles.get(i/2);
            	rhs.set(i, h*alpha*p.mass*rhs.get(i));
            	rhs.set(i+1, h*alpha*p.mass*rhs.get(i+1));
            }
            
            double hb = h+beta;
            DenseVector hbKxd = new DenseVector(2*N);
            getForce(f);
            DenseVector hf = new DenseVector(f); 
            hf = hf.scale(h);
            for(int i = 0; i<particles.size()*2; i+=2)
            {
            	Particle p = particles.get(i/2);
            	hbKxd.set(i, p.df.x*hb);
            	hbKxd.set(i+1, p.df.y*hb);
            }
            
            rhs = (DenseVector)(rhs.add(hbKxd).add(hf));
            deltaxdot = new DenseVector(2*N);
            for(int i = 0 ; i<deltaxdot.size(); i++)
            {
            	deltaxdot.set(i,0); 
            }
            cgMTJ.solve(this, rhs, deltaxdot, cgIterations.getValue());
             setVelocity((DenseVector)(xdot.add(deltaxdot)));         	
        }
              
        // Finally, update the positions using the velocity at the next time step
        for ( Particle p : particles ) {
            if ( p.pinned ) continue;
            // symplectic Euler 
            tmp.scale( h, p.v );
            p.p.add( tmp );
            p.f.set(0,0);
        }
        
        processFracture();
                        
        time = time + h;
        return resultOK;
    }
    
    /**
     * Fills in the provided vector with the particle velocities.
     * @param xd
     */
    private void getVelocity( DenseVector xd ) {
    	int j = 0;
        for ( Particle p : particles ) {
            if( p.pinned ) {
                xd.set( j+0, 0 );
                xd.set( j+1, 0 );
            } else {
                xd.set( j+0, p.v.x );
                xd.set( j+1, p.v.y );
            }
            j += 2;
        }       
    }

    private void setVelocity( DenseVector xdot ) {
    	int j = 0;
        for ( Particle p : particles ) {
            if( p.pinned ) {
                p.v.set(0,0);
            } else {
                p.v.x = xdot.get(j+0);
                p.v.y = xdot.get(j+1);
            }
            j += 2;
        }
    }
    
    private void getForce( DenseVector f ) {
    	int j = 0;
        for ( Particle p : particles ) {
        	f.set( j+0, p.f.x );
        	f.set( j+1, p.f.y );
        	j += 2;
        }
    }
    
    /**
     * Fills the provided vector with the current positions of the particles.
     * @param x
     */
    private void getPosition( DenseVector x ) {
    	int j = 0;
    	for ( Particle p: particles ) {
            x.set( j+0, p.p.x );
            x.set( j+1, p.p.y );
            j += 2;
    	}	
    }
    
    public void applyTempVelocities() {
    	for(Particle p : particles)	{
    		p.v.set(p.vTmp);
    	}
    }
    
    public void updateTempVelocities() {
    	for(Particle p : particles) {
    		p.vTmp.set(p.v);
    	}
    }
    /**
     * Creates a new particle and adds it to the system
     * @param x
     * @param y
     * @param vx
     * @param vy
     * @return the new particle
     */
    public Particle createParticle( double x, double y, double vx, double vy ) {
        Particle p = new Particle( x, y, vx, vy );
        particles.add( p );
        return p;
    }
            
    @Override
    public void init(GLAutoDrawable drawable) {
        // do nothing
    }

    @Override
    public void display(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2();

        // draw the interaction spring
        if ( useMouseSpring ) {
        	mouseSpring.display( drawable );        	
        }

        // draw the boundaries of the mesh (i.e., used in collision detection
        if ( drawBoundaryEdges.getValue() ) {
	        gl.glColor4d( 0,0,0, 1 );
	        gl.glLineWidth( 2 );
	        gl.glBegin( GL.GL_LINES );
	        for ( Edge e : collidableEdges ) {
	        	gl.glVertex2d( e.p1.p.x, e.p1.p.y );
	        	gl.glVertex2d( e.p2.p.x, e.p2.p.y );
	        }
	        gl.glEnd();
        }
        
        // draw the triangles, and stress
        gl.glLineWidth( 1 );
        float alpha = transparency.getFloatValue();
        
        if ( drawElementBoundaries.getValue() ) {
        	for ( FEMTriangle f : femSprings ) {
        		f.displayElementBoundaries(drawable, alpha);
        	}
        }
        
        double s1 = strainEigVecScale.getValue();
        double s2 = stressEigVecScale.getValue();
        for ( FEMTriangle f : femSprings ) {
           f.display(drawable, alpha);
           if ( drawStrainTensor.getValue() ) {
        	   f.displayStrain( drawable, s1 );
           }
           if ( drawStressTensor.getValue() ) {
            	f.displayStress( drawable, s2 );
            }
        }

        // draw the separation tensors
        double sts = stScale.getValue();        
        if ( drawSeparationTensor.getValue() ) {
        	for ( Particle p : particles ) {
        		p.drawSeparationTensor(drawable, sts);
        	}
        }
        
        // draw the particles
        if ( drawParticles.getValue() ) {
            gl.glPointSize( pointSize.getFloatValue() );
            gl.glBegin( GL.GL_POINTS );
            for ( Particle p : particles ) {
                // transparency is used to get smooth edges on the particles
                alpha = 1;//  0.75;
                if ( p.pinned ) {
                    gl.glColor4d( 1, 0, 0, alpha );
                } else {
                	gl.glColor4d( 0,0,0, alpha );//gl.glColor4d( 0, 0.95,0, alpha );
                }
                gl.glVertex2d( p.p.x, p.p.y );
            }
            gl.glEnd();
        }
        
        if ( drawCollisions.getValue() ) {
        	for ( Collision c : collisionList ) {
        		c.display( drawable, 1.0 );
        	}
        }
        if ( drawCollisionBondary.getValue() ) {
        	for ( Collision c : collisionList ) {
        		c.drawCollisionBoundary(drawable);
        	}
        }
        
        
    }    
    
    
    public JPanel getControls() {
        VerticalFlowPanel vfp = new VerticalFlowPanel();
        
        VerticalFlowPanel vfp0 = new VerticalFlowPanel();
        vfp0.setBorder( new TitledBorder("Viewing Parameters" ) );
        
        HorizontalFlowPanel hfp0 = new HorizontalFlowPanel();
        hfp0.add( drawParticles.getControls() );
        hfp0.add( pointSize.getSliderControls(false) );
        vfp0.add( hfp0.getPanel() );
        
        vfp0.add( drawCollisions.getControls() );
        vfp0.add( drawCollisionBondary.getControls() );
        
        HorizontalFlowPanel hfp1 = new HorizontalFlowPanel();
        hfp1.add( drawStrainTensor.getControls() );
        hfp1.add( strainEigVecScale.getSliderControls(true));
        vfp0.add( hfp1.getPanel() );
        
        HorizontalFlowPanel hfp2 = new HorizontalFlowPanel();
        hfp2.add( drawStressTensor.getControls() );
        hfp2.add( stressEigVecScale.getSliderControls(true));
        vfp0.add( hfp2.getPanel() );
        
        HorizontalFlowPanel hfp3 = new HorizontalFlowPanel();
        hfp3.add( drawSeparationTensor.getControls() );
        hfp3.add( stScale.getSliderControls(true));
        vfp0.add( hfp3.getPanel() );
        
        vfp0.add( drawElementBoundaries.getControls() );
        vfp0.add( drawBoundaryEdges.getControls() );
        vfp0.add( transparency.getSliderControls(false) );
        vfp0.add( comments );
        vfp0.add( showCommentsAndParameters.getControls() );
        CollapsiblePanel cp0 = new CollapsiblePanel( vfp0.getPanel() );
        cp0.collapse();
        vfp.add( cp0 );
                
        VerticalFlowPanel vfp1 = new VerticalFlowPanel();
        vfp1.setBorder( new TitledBorder("Simulation Parameters"));
        vfp1.add( doCollisionDetection.getControls() );
        vfp1.add( implicit.getControls() );
        vfp1.add( newtonIterations.getControls() );
        vfp1.add( cgIterations.getControls() );
        
        HorizontalFlowPanel hfpg = new HorizontalFlowPanel();
        hfpg.add( useg.getControls() );
        hfpg.add( g.getSliderControls(true) );
        vfp1.add( hfpg.getPanel() );

        vfp1.add( YoungModulus.getSliderControls(true));
        vfp1.add( PoissonRatio.getSliderControls(false));
        vfp1.add( mouseSpringk.getSliderControls(true));
        vfp1.add( toughness.getSliderControls(true));
        
        HorizontalFlowPanel hfp4 = new HorizontalFlowPanel();
        hfp4.add( useRAlpha.getControls() );
        hfp4.add( RayleighAlpha.getSliderControls(true));
        vfp1.add( hfp4.getPanel() );
        
        HorizontalFlowPanel hfp5 = new HorizontalFlowPanel();
        hfp5.add( useRBeta.getControls() );
        hfp5.add( RayleighBeta.getSliderControls(true));
        vfp1.add( hfp5.getPanel() );
        
        vfp1.add( useIrving2004.getControls() );
        vfp1.add( collisionViscousCoefficient.getSliderControls(true) );
        vfp1.add( collisionRepulsion.getSliderControls(true) );
        vfp1.add( relNormalVelThresh.getSliderControls(true) );
        
        CollapsiblePanel cp1 = new CollapsiblePanel( vfp1.getPanel() );
        cp1.collapse();
        vfp.add( cp1 );
        
        return vfp.getPanel();        
    }
    
    @Override
    public String toString() {
    	DecimalFormat df = new DecimalFormat("0.000");
        String s = "particles = " + particles.size() + "\n" + "time = " + df.format(time) + "\n";
        s += comments.getText() + "\n";                
        return s;
    }
    
}
