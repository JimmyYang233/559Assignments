package comp559.a2ccd;

import java.awt.Font;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

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
	    			List<Double> ts = findT(h, spring.A, spring.B, particle);
	    			for(double t : ts)
	    			{
    					double alpha = findAlpha(t, spring.A, spring.B, particle);
    					if(alpha>=0&&alpha<=1)
    					{
    						System.out.println("alpha is " + alpha);
        					System.out.println("t is " + t);
        					process(spring.A, spring.B, particle, alpha, t);
        					foundCollision = true;
    					}					
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
    
    public List<Double> findT(double h, Particle A, Particle B, Particle C)
    {
    	List<Double> ts = new ArrayList<Double>();
		if(A.equals(C)||B.equals(C)) {
			return ts;
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
			if(a == 0)
			{
				double tmpt = -c/b;
				if(tmpt>0&&tmpt<=h)
				{
					ts.add(tmpt);
				}
			}
			else if(delta>0)
			{

				double t1 = (-b+Math.sqrt(delta))/(2*a);
				double t2 = (-b-Math.sqrt(delta))/(2*a);
				if(t1>0&&t1<=h)
				{
					ts.add(t1);
				}
				if(t2>0&&t2<=h)
				{
					ts.add(t2);
				}
			}
	    	return ts;
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
    	
    public void process(Particle A, Particle B , Particle C, double alpha, double t)
    {
    	Point2d pa = A.p;
    	Point2d pb = B.p;
    	Vector2d va = A.v;
    	Vector2d vb = B.v;
    	Vector2d vc = C.v;
    	Vector2d n = new Vector2d(pb.y+vb.y*t-pa.y-va.y*t, pb.x+vb.x*t-pa.x-va.x*t);
    	n.normalize();
    	//System.out.println("n is " + n);
    	Vector2d valpha = new Vector2d(alpha*va.x+(1-alpha)*vb.x, alpha*(va.y)+(1-alpha)*vb.y);
    	Vector2d vminus = new Vector2d(n.x*(vc.x-valpha.x), n.y*(vc.y-valpha.y));
    	System.out.println("vminus is " + vminus);
    	double ma = A.mass;
    	double mb = B.mass;
    	double mc = C.mass;
    	double e = restitutionValue.getValue();
    	Vector2d j = new Vector2d(-(1+e)*vminus.x/(1/mc+alpha*alpha/ma+(1-alpha)*(1-alpha)/mb), 
    			-(1+e)*vminus.y/(1/mc+alpha*alpha/ma+(1-alpha)*(1-alpha)/mb));
    	System.out.println("j is " + j);
    	A.v.add(new Vector2d(-alpha*j.x/ma, -alpha*j.y/ma));
    	B.v.add(new Vector2d(-(1-alpha)*j.x/mb, -(1-alpha)*j.y/mb));
    	//C.p.add(j);
    	//System.out.println(C.v);
    	C.v.add(new Vector2d(j.x/mc, j.y/mc));
    	//System.out.println(C.v);
    }
    
}
