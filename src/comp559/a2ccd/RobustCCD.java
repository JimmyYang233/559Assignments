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
    
    public Point2d findClosestPoint(Point2d pa, Point2d pb, Point2d pc)
    {
    	Vector2d a_to_p = new Vector2d(pc.x-pa.x, pc.y-pa.y);
    	Vector2d a_to_b = new Vector2d(pb.x-pa.x, pb.y-pa.y);
    	double atb2 = a_to_b.x*a_to_b.x+a_to_b.y*a_to_b.y;
    	double atpdotatb = a_to_p.dot(a_to_b);
    	double t = atpdotatb/atb2;
    	return new Point2d(pa.x+a_to_b.x*t, pa.y+a_to_b.y*t);
    }
    
    public double findAlphaWithClosestPoint(Point2d A, Point2d B, Point2d C)
    {
    	double alpha = 0;
    	Vector2d b_to_p = new Vector2d(C.x-B.x, C.y-B.y);
    	Vector2d a_to_b = new Vector2d(A.x-B.x, A.y-B.y);
    	double atb2 = a_to_b.x*a_to_b.x+a_to_b.y*a_to_b.y;
    	double atpdotatb = b_to_p.dot(a_to_b);
    	alpha= atpdotatb/atb2;
		return alpha;
    }
    
    public double getVreln(Vector2d va, Vector2d vb, Vector2d vc, Vector2d n, double alpha)
    {
    	Vector2d valpha = new Vector2d(alpha*va.x+(1-alpha)*vb.x, alpha*(va.y)+(1-alpha)*vb.y);
    	double vminus = n.dot(new Vector2d(vc.x-valpha.x, vc.y-valpha.y));
    	return vminus;
    }
    /**
     * Try to deal with contacts before they happen
     * @param h
     * @param system
     */
    public void applyRepulsion( double h, ParticleSystem system ) {
    	if ( ! repulsion.getValue() ) return;
    	double H = minDist.getValue();
    	for(Particle particle : system.particles)
    	{
    		for(Spring spring : system.springs)
    		{
    			if(particle!=spring.A&&particle!=spring.B)
    			{
    				double k = spring.ks;
					Point2d pa = spring.A.p;
			    	Point2d pb = spring.B.p;
    				Point2d pc = particle.p;
        			Point2d cp = findClosestPoint(pa, pb, pc);
    				double alpha = findAlphaWithClosestPoint(pa, pb, cp);
    				double distance = Math.sqrt(Math.pow(pc.x-cp.x,2) + Math.pow(pc.y-cp.y, 2));
    				if(alpha>=0&&alpha<=1&&H>=distance)
    				{
    			    	Vector2d ac = new Vector2d(pc.x-pa.x, pc.y-pa.y);
    			    	double x = pb.y-pa.y;
    			    	double y = pb.x-pa.x;
    			    	Vector2d n = new Vector2d(x, -y);
    			    	if(n.dot(ac)<0)
    			    	{
    			    		n = new Vector2d(-(x), y);
    			    	}
    			    	n.normalize();
    					double vminus = getVreln(spring.A.v,spring.B.v,particle.v,n,alpha);		
    					double d = H-distance;
    					if(vminus<(.1d/h))	
    					{
    						//System.out.println("V is " + vminus);
    						double I = -Math.min(h*k*d, particle.mass*(.1d/h-vminus));
    						//System.out.println("I is " + I);
    						//System.out.println("VB before " + spring.B.v);
    						applyImpulse(spring.A, spring.B, particle, alpha, n, I);
    						//System.out.println("VB after " + spring.B.v);
    					}
    				}	
    			} 							
			}
		}
    	// TODO: apply repulsion forces
    	// use minDist.getValue() as the thickness
    	// use your spring stiffness for the repulsion stiffness, or create 
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
		iters = 0;
    	boolean foundCollision = false;
    	int max = maxIterations.getValue();
    	do 
    	{
    		foundCollision = false;

	    	for(Particle particle : system.particles)
	    	{
	    		for(Spring spring : system.springs)
	    		{
	    			if(spring.A!=particle&&spring.B!=particle)
	    			{
		    			List<Double> ts = findT(h, spring.A, spring.B, particle);
		    			for(double t : ts)
		    			{
	    					double alpha = findAlpha(t, spring.A, spring.B, particle);
	    					if(alpha>=0&&alpha<=1)
	    					{	
	    						iters++;
	        					process(spring.A, spring.B, particle, alpha, t);
	        					foundCollision = true;
	        					break;
	    					}					
		    			}
	    			}
    			}
    		}
	    	if(iters>=max)
	    	{
	    		return false;
	    	}
    	}while(foundCollision);
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
			if(a==0)
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
    	Vector2d b_to_p = new Vector2d(cxx-bxx, cyy-byy);
    	Vector2d a_to_b = new Vector2d(axx-bxx, ayy-byy);
    	double atb2 = a_to_b.x*a_to_b.x+a_to_b.y*a_to_b.y;
    	double atpdotatb = b_to_p.dot(a_to_b);
    	alpha= atpdotatb/atb2;
		//alpha = Math.sqrt((Math.pow((cxx-bxx),2)+Math.pow((cyy-byy),2))/(Math.pow((axx-bxx),2)+Math.pow((ayy-byy),2)));
		return alpha;
    }
    	
    public void process(Particle A, Particle B , Particle C, double alpha, double t)
    {
    	Vector2d n = getNormal(A,B,C,t);
    	double vminus = getVreln(A,B,C,alpha,t);
    	//System.out.println("vminus is " + vminus);
    	double ma = A.pinned?1.0/0.0:A.mass;
    	double mb = B.pinned?1.0/0.0:B.mass;
    	double mc = C.pinned?1.0/0.0:C.mass;
    	double e = restitutionValue.getValue();
    	double j = -(1+e)*vminus/(1/mc+alpha*alpha/ma+(1-alpha)*(1-alpha)/mb);
    	applyImpulse(A,B,C,alpha,n, j);
    }  
    
    public Vector2d getNormal(Particle A, Particle B, Particle C, double t)
    {
    	Point2d pa = A.p;
    	Point2d pb = B.p;

    	Vector2d va = A.v;
    	Vector2d vb = B.v;
    	double x = pb.y+vb.y*t-pa.y-va.y*t;
    	double y = pb.x+vb.x*t-pa.x-va.x*t;
    	Vector2d ac = new Vector2d(C.p.x+C.v.x*t-A.p.x-A.v.x*t, C.p.y+C.v.y*t-A.p.y-A.v.y*t);
    	Vector2d n = new Vector2d(-(x), y);
    	if(n.dot(ac)<0)
    	{
    		n = new Vector2d(x, -y);
    	}
    	n.normalize();
    	return n;
    }
    
    public double getVreln(Particle A, Particle B, Particle C, double alpha, double t)
    {
    	Vector2d va = A.v;
    	Vector2d vb = B.v;
    	Vector2d vc = C.v;
    	Vector2d n = getNormal(A,B,C,t);
    	Vector2d valpha = new Vector2d(alpha*va.x+(1-alpha)*vb.x, alpha*(va.y)+(1-alpha)*vb.y);
    	double vminus = n.dot(new Vector2d(vc.x-valpha.x, vc.y-valpha.y));
    	return vminus;
    }
    
    public void applyImpulse(Particle A, Particle B, Particle C, double alpha, Vector2d n, double j)
    {
    	//System.out.println("j is " + j);
    	double ma = A.pinned?1.0/0.0:A.mass;
    	double mb = B.pinned?1.0/0.0:B.mass;
    	double mc = C.pinned?1.0/0.0:C.mass;

		//System.out.println("A before " + A.v);
    	A.v.add(new Vector2d(-alpha*j*n.x/ma, -alpha*j*n.y/ma));
    	//System.out.println("A after " + A.v);
		//System.out.println("B before " + B.v);
    	B.v.add(new Vector2d(-(1-alpha)*j*n.x/mb, -(1-alpha)*j*n.y/mb));
    	//System.out.println("B after " + B.v);
    	//System.out.println("C before " + C.v);
    	C.v.add(new Vector2d(j*n.x/mc, j*n.y/mc));
    	//System.out.println("C after " + C.v);
    	//C.p.add(new Vector2d(j*n.x, j*n.y));

    }
}
