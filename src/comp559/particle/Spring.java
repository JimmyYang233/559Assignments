package comp559.particle;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import no.uib.cipr.matrix.DenseMatrix;
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
    
    public static double p = 1;
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
    	double distance = p1.p.distance(p2.p);
    	Vector2d vaMinusvb = new Vector2d(p1.v.x-p2.v.x, p1.v.y-p2.v.y);
    	Vector2d aMinusb = new Vector2d(p1.p.x-p2.p.x, p1.p.y-p2.p.y);
    	double dot = vaMinusvb.dot(aMinusb);
    	double mult = -(k*(distance-l0)+c*dot/distance)/distance;
    	Vector2d fA = new Vector2d(aMinusb);
    	fA.scale(mult);
    	Vector2d fB = new Vector2d(fA);
    	fB.scale(-1);
    	p1.addForce(fA);
    	p2.addForce(fB);
        // DONE: Objective 1, FINISH THIS CODE!
        
    }
   
    /** TODO: the functions below are for the backwards Euler solver */
    
    /**
     * Computes the force and adds it to the appropriate components of the force vector.
     * (This function is something you might use for a backward Euler integrator)
     * @param f
     */
    public void addForce( Vector f ) {
    	double distance = p1.p.distance(p2.p);
    	Vector2d vaMinusvb = new Vector2d(p1.v.x-p2.v.x, p1.v.y-p2.v.y);
    	Vector2d aMinusb = new Vector2d(p1.p.x-p2.p.x, p1.p.y-p2.p.y);
    	double dot = vaMinusvb.dot(aMinusb);
    	double mult = -(k*(distance-l0)+c*dot/distance)/distance;
    	Vector2d fA = new Vector2d(aMinusb);
    	fA.scale(mult);
    	Vector2d fB = new Vector2d(fA);
    	fB.scale(-1);
    	//fA.y = fA.y+p1.mass*9.8;
    	//fB.y = fB.y+p2.mass*9.8;
    	p1.addForce(fA);
    	p2.addForce(fB);
    	
    	int ax = 2*p1.index;
    	int ay = 2*p1.index+1;
    	int bx = 2*p2.index;
    	int by = 2*p2.index+1;
    	f.add(ax,fA.x);
    	f.add(ay,fA.y);
    	f.add(bx, fB.x);
    	f.add(by, fB.y);
        // TODO: Objective 8, FINISH THIS CODE for backward Euler method (probably very simlar to what you did above)
        
    }
    
    /**
     * Adds this springs contribution to the stiffness matrix
     * @param dfdx
     */
    public void addDfdx( Matrix dfdx ) {
        // TODO: Objective 8, FINISH THIS CODE... necessary for backward euler integration
    	double dfaxdax = getDfaxdax(p1.p, p2.p, p1.v, p2.v);
    	double dfaxday = getDfaxday(p1.p,p2.p,p1.v,p2.v);
    	double dfaydax = getDfaydax(p1.p,p2.p,p1.v,p2.v);
    	double dfayday = getDfayday(p1.p,p2.p,p1.v,p2.v);
    	double dfbxdbx = getDfaxdax(p2.p,p1.p,p2.v,p1.v);
        double dfbxdby = getDfaxday(p2.p,p1.p,p2.v,p1.v);
        double dfbydbx = getDfaydax(p2.p,p1.p,p2.v,p1.v);
        double dfbydby = getDfayday(p2.p,p1.p,p2.v,p1.v);
        double dfbxdax = -dfaxdax;
        double dfbxday = -dfaxday;
        double dfbydax = -dfaydax;
        double dfbyday = -dfayday;
        double dfaxdbx = -dfbxdbx;
        double dfaxdby = -dfbxdby;
        double dfaydbx = -dfbydbx;
        double dfaydby = -dfbydby;
        int ax = 2*p1.index;
        int ay = 2*p1.index+1;
        int bx = 2*p2.index;
        int by = 2*p2.index+1;
        dfdx.add(ax, ax, dfaxdax);
        dfdx.add(ax, ay, dfaxday);
        dfdx.add(ax, bx, dfaxdbx);
        dfdx.add(ax, by, dfaxdby);
        dfdx.add(ay, ax, dfaydax);
        dfdx.add(ay, ay, dfayday);
        dfdx.add(ay, bx, dfaydbx);
        dfdx.add(ay, by, dfaydby);
        dfdx.add(bx, ax, dfbxdax);
        dfdx.add(bx, ay, dfbxday);
        dfdx.add(bx, bx, dfbxdbx);
        dfdx.add(bx, by, dfbxdby);
        dfdx.add(by, ax, dfbydax);
        dfdx.add(by, ay, dfbyday);
        dfdx.add(by, bx, dfbydbx);
        dfdx.add(by, by, dfbydby);     
    }
    
    public double getDfaxdax(Point2d A, Point2d B, Vector2d VA, Vector2d VB) {
    	double l = A.distance(B);
    	Vector2d VAminusVB = new Vector2d(VA.x-VB.x, VA.y-VB.y);
    	Vector2d AminusB = new Vector2d(A.x-B.x, A.y-B.y);
    	double lx = AminusB.x;
    	double ly = AminusB.y;
    	double VdotP = VAminusVB.dot(AminusB);
    	double kSide = -k*(1-l0/l+lx*lx*(l0/(l*l*l)));
    	double cSide = -c*((VA.x-VB.x)*lx/(l*l)+VdotP*(1/(l*l)-2*lx*lx/(l*l*l*l)));
    	return kSide+cSide;
    }
    
    public double getDfaxday(Point2d A, Point2d B, Vector2d VA, Vector2d VB) {
    	double l = A.distance(B);
    	Vector2d VAminusVB = new Vector2d(VA.x-VB.x, VA.y-VB.y);
    	Vector2d AminusB = new Vector2d(A.x-B.x, A.y-B.y);
    	double lx = AminusB.x;
    	double ly = AminusB.y;
    	double VdotP = VAminusVB.dot(AminusB);
    	double kSide = -k*l0*lx*ly/(l*l*l);
    	double cSide = -c*((VA.y-VB.y)*lx/(l*l)+VdotP*(-2*lx*ly/(l*l*l*l)));
    	return kSide+cSide;
    }
    
    public double getDfaydax(Point2d A, Point2d B, Vector2d VA, Vector2d VB) {
    	double l = A.distance(B);
    	Vector2d VAminusVB = new Vector2d(VA.x-VB.x, VA.y-VB.y);
    	Vector2d AminusB = new Vector2d(A.x-B.x, A.y-B.y);
    	double lx = AminusB.x;
    	double ly = AminusB.y;
    	double VdotP = VAminusVB.dot(AminusB);
    	double kSide = -k*l0*lx*ly/(l*l*l);
    	double cSide = -c*((VA.x-VB.x)*ly/(l*l)+VdotP*(-2*lx*ly/(l*l*l*l)));
    	return kSide+cSide;
    }
    
    public double getDfayday(Point2d A, Point2d B, Vector2d VA, Vector2d VB) {
    	double l = A.distance(B);
    	Vector2d VAminusVB = new Vector2d(VA.x-VB.x, VA.y-VB.y);
    	Vector2d AminusB = new Vector2d(A.x-B.x, A.y-B.y);
    	double lx = AminusB.x;
    	double ly = AminusB.y;
    	double VdotP = VAminusVB.dot(AminusB);
    	double kSide = -k*(1-l0/l+ly*ly*(l0/(l*l*l)));
    	double cSide = -c*((VA.y-VB.y)*ly/(l*l)+VdotP*(1/(l*l)-2*ly*ly/(l*l*l*l)));
    	return kSide+cSide;
    }
 
    /**
     * Adds this springs damping contribution to the implicit damping matrix
     * @param dfdv
     */
    public void addDfdv( Matrix dfdv ) {
    	double dfaxdvax = getDfaxdvax(p1.p,p2.p);
    	double dfaxdvay = getDfaxdvay(p1.p,p2.p);
    	double dfaydvax = getDfaydvax(p1.p,p2.p);
    	double dfaydvay = getDfaydvay(p1.p,p2.p);
    	double dfbxdvbx = getDfaxdvax(p2.p,p1.p);
    	double dfbxdvby = getDfaxdvay(p2.p,p1.p);
    	double dfbydvbx = getDfaydvax(p2.p,p1.p);
    	double dfbydvby = getDfaydvay(p2.p,p1.p);
    	double dfbxdvax = -dfaxdvax;
    	double dfbxdvay = -dfaxdvay;
    	double dfbydvax = -dfaydvax;
    	double dfbydvay = -dfaydvay;
    	double dfaxdvbx = -dfbxdvbx;
    	double dfaxdvby = -dfbxdvby;
    	double dfaydvbx = -dfbydvbx;
    	double dfaydvby = -dfbydvby;
        int ax = 2*p1.index;
        int ay = 2*p1.index+1;
        int bx = 2*p2.index;
        int by = 2*p2.index+1;
        dfdv.add(ax, ax, dfaxdvax);
        dfdv.add(ax, ay, dfaxdvay);
        dfdv.add(ax, bx, dfaxdvbx);
        dfdv.add(ax, by, dfaxdvby);
        dfdv.add(ay, ax, dfaydvax);
        dfdv.add(ay, ay, dfaydvay);
        dfdv.add(ay, bx, dfaydvbx);
        dfdv.add(ay, by, dfaydvby);
        dfdv.add(bx, ax, dfbxdvax);
        dfdv.add(bx, ay, dfbxdvay);
        dfdv.add(bx, bx, dfbxdvbx);
        dfdv.add(bx, by, dfbxdvby);
        dfdv.add(by, ax, dfbydvax);
        dfdv.add(by, ay, dfbydvay);
        dfdv.add(by, bx, dfbydvbx);
        dfdv.add(by, by, dfbydvby); 
        // TODO: Objective 8, FINISH THIS CODE... necessary for backward Euler integration
        
    }
    
    public double getDfaxdvax(Point2d A, Point2d B)
    {
    	double l = A.distance(B);
    	double ans = -c*((A.x-B.x)*(A.x-B.x))/(l*l)-p;
    	return ans;
    }
    
    public double getDfaxdvay(Point2d A, Point2d B)
    {
    	double l = A.distance(B);
    	double ans = -c*((A.x-B.x)*(A.y-B.y))/(l*l);
    	return ans;
    }
    
    public double getDfaydvax(Point2d A, Point2d B)
    {
    	return getDfaxdvay(A,B);
    }
    
    public double getDfaydvay(Point2d A, Point2d B)
    {
    	double l = A.distance(B);
    	double ans = -c*((A.y-B.y)*(A.y-B.y))/(l*l)-p;
    	return ans;
    }
    
}
