package comp559.a2ccd;

import java.awt.Font;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.VerticalFlowPanel;

/**
 * Implementation of a robust collision detection
 * @author kry
 */
public class RobustCCD {
	        
    /** number of iterations in the last CCD processing loop, to keep an eye on how tricky the problem is */
    int iters;
    
    /**
     * Creates the new continuous collision detection and response object
     */
    public RobustCCD() {
        // do nothing
    }
    
    /** Might want to turn off collisions when testing? */
    BooleanParameter collision = new BooleanParameter( "apply collision impulses", true );

    /** Ignore this parameter unless you want to explore a Jacobi type resolution of collision */
    BooleanParameter useJacobi = new BooleanParameter( "use Jacobi", false );

    /** Use this as the maximum number of iterations, feel free to modify default, or the maximum! */
    IntParameter maxIterations = new IntParameter("maximum iterations", 60, 30, 300 );

    BooleanParameter repulsion = new BooleanParameter( "apply repulsion impulses", true );

    DoubleParameter restitutionValue = new DoubleParameter( "restitution", .0001, 0, 1 );

    DoubleParameter minDist = new DoubleParameter( "min distance (H)", 2, 0.1, 10 );

    public JPanel getControls() {
    	VerticalFlowPanel vfp = new VerticalFlowPanel();
    	vfp.setBorder( new TitledBorder("Robust CCD Controls"));
        ((TitledBorder) vfp.getPanel().getBorder()).setTitleFont(new Font("Tahoma", Font.BOLD, 18));
    	vfp.add( collision.getControls() );
    	vfp.add( useJacobi.getControls() );
    	vfp.add(maxIterations.getSliderControls());
    	vfp.add( repulsion.getControls() );
    	vfp.add( restitutionValue.getSliderControls(false) );
    	vfp.add( minDist.getSliderControls(true));    	
    	return vfp.getPanel();
    }
    
    /**
     * Try to deal with contacts before they happen
     * @param h
     * @param system
     */
    public void applyRepulsion( double h, ParticleSystem system ) {
    	if ( ! repulsion.getValue() ) return;
    	
    	// TODO: apply repulsion forces
    	// use minDist.getValue() as the thickness
    	// use your spring stiffness for the repulsion stifness, or create 
    	// new parameters and set their value!
    }
    
    /**
     * Checks all collisions in interval t to t+h
     * @param h
     * @param system 
     * @return true if all collisions resolved
     */
    public boolean check( double h, ParticleSystem system ) {        
    	if ( ! collision.getValue() ) return true; // pretend everything is OK!
    	boolean foundCollision = false;
    	int max = maxIterations.getValue();
    	int iter = 0;
    	do 
    	{
	    	for(Particle particle : system.particles)
	    	{
	    		for(Spring spring : system.springs)
	    		{
	    			double t = findT(h, spring.A, spring.B, particle);
    				if(t>=0)
    				{
    					double alpha = findAlpha(t, spring.A, spring.B, particle);
    					System.out.println("alpha is " + alpha);
    					process();
    					foundCollision = true;
    				}
    			}
    		}
	    	iter++;
    	}while(foundCollision && iter<max);
	    	
    	// TODO: find collisions and apply impulses to resolve them
    	// use maxIterations.getValue() as max iteraitons before giving up
    	// use restitutionValue.getValue() for computing the impulse
        
        return true;
    }
    
    public double findT(double h, Particle A, Particle B, Particle C)
    {
		if(A.equals(C)||B.equals(C)) {
			return -1;
		}
		else
		{
			double ax = A.p.x;
			double ay = A.p.y;
			double bx = B.p.x;
			double by = B.p.y;
			double cx = C.p.x;
			double cy = C.p.y;
			double aax = A.v.x;
			double aay = A.v.y;
			double bbx = B.v.x;
			double bby = B.v.y;
			double ccx = C.v.x;
			double ccy = C.v.y;
			
			double c = ax*by+bx*cy+ay*cx-cx*by-cy*ax-ay*bx;
			double b = aax*by+bby*ax+bbx*cy+ccy*bx+aay*cx+
					ccx*ay-ccx*by-bby*cx-aay*bx-bbx*ay-ccy*ax-aax*cy;
			double a = aax*bby+bbx*ccy+aay*ccx-ccx*bby-ccy*aax-aay*bbx;
			double delta = b*b-4*a*c;
			double t = -1;
			if(a == 0)
			{
				double tmpt = -c/b;
				if(tmpt>0&&tmpt<=h)
				{
					t = tmpt;
				}
			}
			else if(delta>0)
			{

				double t1 = (-b+Math.sqrt(delta))/(2*a);
				double t2 = (-b-Math.sqrt(delta))/(2*a);
				if(t1>0&&t1<=h&&t2>0&&t2<=h)
				{
					if(t1>t2)
					{
						t = t2;
					}
					else 
					{
						t = t1;
					}
				}
				else if(t1>0&&t1<=h)
				{
					t= t1;
				}
				else if(t2>0&&t2<=h)
				{
					t = t2;
				}
				else
				{
					//do nothing
				}
			}
	    	return t;
		}

    }
    
    public double findAlpha(double t, Particle A, Particle B, Particle C)
    {
    	double ax = A.p.x;
		double ay = A.p.y;
		double bx = B.p.x;
		double by = B.p.y;
		double cx = C.p.x;
		double cy = C.p.y;
		double aax = A.v.x;
		double aay = A.v.y;
		double bbx = B.v.x;
		double bby = B.v.y;
		double ccx = C.v.x;
		double ccy = C.v.y;
    	double alpha = 0;
		double axx = ax+t*aax;
		double ayy = ay+t*aay;
		double bxx = bx+t*bbx;
		double byy = by+t*bby;
		double cxx = cx+t*ccx;
		double cyy = cy+t*ccy;
		alpha = Math.sqrt((Math.pow((cxx-bxx),2)+Math.pow((cyy-byy),2))/(Math.pow((axx-bxx),2)+Math.pow((ayy-byy),2)));
		return alpha;
    }
    	
    public void process()
    {
    	//To-do
    }
    
}
