package fractureW20;

import javax.vecmath.Color3f;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

public class FEMTriangle {

	Particle A, B, C;
	
	/** Colour for drawing, but also determines the mass for triangles loaded from an image */
	Color3f colour = new Color3f();
	
	/** Poisson ratio */
	static double mu = 0.3;
	/** Lame parameter */
	static double lambda = 1000;

	double area;
	
	/** used for computing lumped mass at particles */
	double density;
	
	/** deformed shape matrix */
	Matrix2d Ds = new Matrix2d();
	/** material space shape matrix */
	Matrix2d Dm = new Matrix2d();
	/** material space shape matrix inverse */
	Matrix2d Dminv = new Matrix2d();
	/** deformation gradient, Ds Dm^{-1} */
	Matrix2d F = new Matrix2d();
	/** Green strain tensor, 0.5 ( F^T F - I )*/
	Matrix2d E = new Matrix2d();
	/** First Piola-Kirchoff stress tensor, F (2 mu E + lambda tr(E) I) */
	Matrix2d P = new Matrix2d();
	/** Stress for visualizaiton, 2 mu E + lambda tr(E) I */
	Matrix2d stress = new Matrix2d();
	/** Forces */
	Matrix2d H = new Matrix2d();
	/** rotation matrix for SVD co-rotated elasticity */
	Matrix2d R = new Matrix2d();
	
	void reset() {
		Ds.zero();
		F.zero();
		E.zero();
		P.zero();
		H.zero();
		stress.zero();
	}
	

	/** index of this tri in each particles */
	int Ai, Bi, Ci;
	
	public FEMTriangle( Particle A, Particle B, Particle C, Color3f colour ) {
		this.A = A;
		this.B = B;
		this.C = C;
		this.colour.set(colour);
		Ai = A.addTriangle( this );
		Bi = B.addTriangle( this );
		Ci = C.addTriangle( this );
		computeAreaAndDm();
	}
	
	/**
	 * Computes the area, and material space matrices for initialization of this element
	 */
	public void computeAreaAndDm() {
		Dm.a = A.p0.x - C.p0.x; Dm.b = B.p0.x - C.p0.x;
		Dm.c = A.p0.y - C.p0.y; Dm.d = B.p0.y - C.p0.y;
		Dminv.inverse(Dm);
		area = 0.5 * Dm.det();	
	}
	
	
	/**
	 * Compute the correct E value, make sure you have the correc F before.
	 */
	public void computeGST() 
	{
		Matrix2d FT = new Matrix2d(F);
		FT.transpose();
		FT.mult(F);
		Matrix2d negI = new Matrix2d(-1,0,0,-1);
		E = FT;
		E.add(negI);
		E.scale(0.5);
	}
	
	/**
	 * Compute the correct stress value
	 */	
	public void computeStress()
	{
		Matrix2d twoMuE = new Matrix2d();
		twoMuE.scale(2*mu, E);
		Matrix2d lamtrEI = new Matrix2d();
		Matrix2d I = new Matrix2d(1,0,0,1);
		lamtrEI.scale(lambda*E.trace(), I);
		stress.add(twoMuE, lamtrEI);
	}
	
	public static Matrix2d m(Vector2d a)
	{
		double aLength = Math.sqrt(a.x*a.x+a.y*a.y);
		Matrix2d ans = new Matrix2d(a.x*a.x, a.x*a.y,a.y*a.x,a.y*a.y);
		ans.scale(1.0/aLength);
		return ans;	
	}
	
	/**
	 * Computes and accumulates the elastic force of this triangle.
	 * This should modify the f member in the three particles of this
	 * finite element.
	 * 
	 * This is also a good place to compute the tensile and compressive stress, 
	 * along with updating the fminus and fplus in each particle to allow for
	 * fracture computation later.
	 */
	public void applyForce() {
		
		// TODO: Objective 1: Compute STVK forces
		Ds.a = A.p.x - C.p.x; Ds.b = B.p.x - C.p.x;
		Ds.c = A.p.y - C.p.y; Ds.d = B.p.y - C.p.y;
		F = Ds;
		F.mult(Dminv);
		computeGST();
		computeStress();
		
		P = new Matrix2d(F);
		P.mult(stress);
		H.transpose(Dminv);
		H.mult(P, H);
		H.scale(-area);
		A.f.x += H.a;
		A.f.y += H.c;
		B.f.x += H.b;
		B.f.y += H.d;
		C.f.x += (-H.a-H.b);
		C.f.y += (-H.c-H.d);
		
		
		Matrix2d tensileStress = new Matrix2d();
		Matrix2d compressiveStress = new Matrix2d();
				
		// TODO: Objective 3: Compute tensile and compressive stress 
		// code is provided to set vectors in the fPlus and fMinus lists of the adjacent particles 
				
		stress.evd();
		
		double ev1 = stress.ev1;
		double ev2 = stress.ev2;
		
		Vector2d v1 = stress.v1;
		Vector2d v2 = stress.v2;
		
		Matrix2d m1 = m(v1);
		Matrix2d m2 = m(v2);
		
		Matrix2d m1Max = new Matrix2d(m1);
		Matrix2d m1Min = new Matrix2d(m1);
		Matrix2d m2Max = new Matrix2d(m2);
		Matrix2d m2Min = new Matrix2d(m2);
		
		m1Max.scale(Math.max(0, ev1));
		m2Max.scale(Math.max(0, ev2));
		m1Min.scale(Math.min(0, ev1));
		m2Min.scale(Math.min(0, ev2));
		
		tensileStress.add(m1Max);
		tensileStress.add(m2Max);
		
		compressiveStress.add(m1Min);
		compressiveStress.add(m2Min);
		
		
		P.mult( F, compressiveStress );
		H.scale( -area, P );
		H.multTrans( Dminv );
		A.fminus.get(Ai).set( H.a, H.c ); 
		B.fminus.get(Bi).set( H.b, H.d );
		C.fminus.get(Ci).set( -H.a-H.b, -H.c-H.d ); 

		P.mult( F, tensileStress );
		H.scale( -area, P );
		H.multTrans( Dminv );
		A.fplus.get(Ai).set( H.a, H.c ); 
		B.fplus.get(Bi).set( H.b, H.d );
		C.fplus.get(Ci).set( -H.a-H.b, -H.c-H.d ); 		
	}
	
	Matrix2d dDs = new Matrix2d();	
	Matrix2d dF = new Matrix2d();
	Matrix2d dFTF = new Matrix2d();
	Matrix2d FTdF = new Matrix2d();
	Matrix2d dE = new Matrix2d();
	Matrix2d dStress = new Matrix2d();
	Matrix2d dP = new Matrix2d();
	Matrix2d tmp = new Matrix2d();
	
	/**
	 * Compute the correct dE value,  make sure you have the correct F and dF before;
	 */
	public void computedGST()
	{
		dFTF = new Matrix2d(dF);
		dFTF.transpose();
		dFTF.mult(F);
		FTdF = new Matrix2d(F);
		FTdF.transpose();
		FTdF.mult(dF);
		dE = new Matrix2d(dFTF);
		dE.add(FTdF);
		dE.scale(0.5);
	}
	
	/**
	 * Compute correct dStress value
	 */
	public void computedStress()
	{
		Matrix2d twoMudE = new Matrix2d();
		twoMudE.scale(2*mu, dE);
		Matrix2d lamtrdEI = new Matrix2d();
		Matrix2d I = new Matrix2d(1,0,0,1);
		lamtrdEI.scale(lambda*dE.trace(), I);
		dStress.add(twoMudE, lamtrdEI);
	}
	
	/**
	 * Use position differential set in particles, to compute and
	 * accumulate force differentials in particles.
	 * 
	 * reads  Particle.dx
	 * writes Particle.df
	 */
	public void computeForceDifferentials( ) {
		
		// TODO: Objective 2: compute the STVK force differentials for implicit integration
		Ds.a = A.p.x - C.p.x; Ds.b = B.p.x - C.p.x;
		Ds.c = A.p.y - C.p.y; Ds.d = B.p.y - C.p.y;
		dDs.a = A.dx.x - C.dx.x; dDs.b = B.dx.x - C.dx.x;
		dDs.c = A.dx.y - C.dx.y; dDs.d = B.dx.y - C.dx.y;
		F = new Matrix2d(Ds);
		F.mult(Dminv);
		dF = new Matrix2d(dDs);
		dF.mult(Dminv);		
		computeGST();
		computedGST();
		computeStress();
		computedStress();
		
		dP = new Matrix2d(dF);
		dP.mult(stress);
		tmp = new Matrix2d(F);
		tmp.mult(dStress);
		dP.add(tmp);
		H.transpose(Dminv);
		H.mult(dP,H);
		H.scale(-area);
		A.df.x += H.a;
		A.df.y += H.c;
		B.df.x += H.b;
		B.df.y += H.d;
		C.df.x += (-H.a-H.b);
		C.df.y += (-H.c-H.d);
	}
	
	/**
	 * Draws the triangle with transparency alpha 
	 * @param drawable
	 * @param alpha transparency
	 */
	public void display( GLAutoDrawable drawable, float alpha ) {
		GL2 gl = drawable.getGL().getGL2();
		gl.glColor4f( colour.x, colour.y, colour.z, alpha );
		gl.glBegin( GL.GL_TRIANGLES );
		gl.glVertex2d( A.p.x, A.p.y );
		gl.glVertex2d( B.p.x, B.p.y );
		gl.glVertex2d( C.p.x, C.p.y );
		gl.glEnd();
	}
	
	/** 
	 * Draws the element boundaries
	 * @param drawable
	 * @param alpha
	 */
	public void displayElementBoundaries( GLAutoDrawable drawable, float alpha ) {
		GL2 gl = drawable.getGL().getGL2();		
		gl.glColor4f( 0,0,0, alpha );
		gl.glBegin( GL.GL_LINE_LOOP );
		gl.glVertex2d( A.p.x, A.p.y );
		gl.glVertex2d( B.p.x, B.p.y );
		gl.glVertex2d( C.p.x, C.p.y );
		gl.glEnd();		
	}
	
	/**
	 * Draws the strain tensor of this element at its center.
	 * @param drawable
	 * @param s
	 */
	public void displayStrain( GLAutoDrawable drawable, double s) {
		displayTensor( drawable, E, s );
	}
	
	/**
	 * Draws the stress tensor of this element at its center.
	 * @param drawable
	 * @param s
	 */
	public void displayStress( GLAutoDrawable drawable, double s) {
		displayTensor( drawable, stress, s );
	}

	/**
	 * Draws the tensor of this element at its center.
	 * @param drawable
	 * @param s
	 */
	public void displayTensor( GLAutoDrawable drawable, Matrix2d T, double s ) {
		GL2 gl = drawable.getGL().getGL2();
		Vector2d p = new Vector2d();
		p.add( A.p );
		p.add( B.p );
		p.add( C.p );
		p.scale(1.0/3.0);

		T.evd(); // may have already been called, so could be not so efficient (but just for viz, so don't worry?)
		double s1 = s * T.ev1;
		double s2 = s * T.ev2;
		
		gl.glBegin( GL.GL_LINES );
		if ( T.ev1 < 0 ) {
			gl.glColor3f(0.75f,0,0);
		} else {
			gl.glColor3f(0,0.75f,0);
		}
		gl.glVertex2d( p.x - T.v1.x * s1, p.y - T.v1.y * s1 );
		gl.glVertex2d( p.x + T.v1.x * s1, p.y + T.v1.y * s1 );
		if ( T.ev2 < 0 ) {
			gl.glColor3f(0.75f,0,0);
		} else {
			gl.glColor3f(0,0.75f,0);
		}
		gl.glVertex2d( p.x - T.v2.x * s2, p.y - T.v2.y * s2 );
		gl.glVertex2d( p.x + T.v2.x * s2, p.y + T.v2.y * s2 );
		gl.glEnd();		
	}
	
	/**
	 * Checks if a point is inside the triangle, with small positive epsilon allowing 
	 * for some overlap before returning true.  Parameter coords is set to the 
	 * barycentric coordinates if it is not null.
	 */
	public boolean isInside( Point2d p, double eps, Point3d coords ) {
		final Vector2d v0 = new Vector2d();
		final Vector2d v1 = new Vector2d();
		final Vector2d v2 = new Vector2d();
		v0.sub(B.p, A.p);
		v1.sub(C.p, A.p);
		v2.sub(p, A.p);	
	    double d00 = v0.dot(v0);
	    double d01 = v0.dot(v1);
	    double d11 = v1.dot(v1);
	    double d20 = v2.dot(v0);
	    double d21 = v2.dot(v1);
	    double denom = d00 * d11 - d01 * d01;
	    double beta = (d11 * d20 - d01 * d21) / denom; // beta
	    double gamma = (d00 * d21 - d01 * d20) / denom; // gamma
	    double alpha = 1 - beta - gamma;
	    if ( coords != null ) {
	    	coords.set( alpha, beta, gamma );
	    }
	    if ( alpha > 1+eps || alpha < -eps || 
	    	 beta > 1+eps || beta < -eps ||
	    	 gamma > 1+eps || gamma < -eps ) {
	    	return false;
	    }
	    return true;
	}
	
	/**
	 * Used in collision detection
	 */
	public boolean isSegmentOnTraingle( Point2d start, Point2d end, double eps ) {	
		Vector2d segment = new Vector2d(start);
		segment.sub(end);				
		Vector2d edge0 = new Vector2d(A.p);
		edge0.sub(B.p);
		Vector2d edge1 = new Vector2d(B.p);
		edge1.sub(C.p);
		Vector2d edge2 = new Vector2d(C.p);
		edge2.sub(A.p);
				
		// First test whether the segment is parallel to any of the three sides
		double det0 = segment.x * edge0.y - edge0.x * segment.y;
		double det1 = segment.x * edge1.y - edge1.x * segment.y;
		double det2 = segment.x * edge2.y - edge2.x * segment.y;
		
		double segmentLength = segment.length();
		
		Vector2d parallelEdge = null;
		Point2d init = null;
		// It is better to avoid in place normalization and use length instead.
		if ( Math.abs(det0) <= eps * segmentLength * edge0.length() ) {
			parallelEdge = edge0;
			init = A.p;
		} else if ( Math.abs(det1) <= eps * segmentLength * edge1.length() ) {
			parallelEdge = edge1;
			init = B.p;
		} else if ( Math.abs(det2) <= eps * segmentLength * edge2.length() ) {
			parallelEdge = edge2;
			init = C.p;
		}
		
		if ( parallelEdge == null ) return false;
		
		double parallelEdgeLength = parallelEdge.length();
		
		if ( parallelEdgeLength < eps ) return false;
		
		// check if start solves the equation of line formed by the edge.
		if ( Math.abs(parallelEdge.x) > eps * parallelEdgeLength ) {
			double slope = parallelEdge.y / parallelEdge.x;
			double intercept = init.y - slope * init.x;
			if ( Math.abs(start.x * slope  + intercept - start.y) > eps * segmentLength ) {
				return false;
			}
		} else {
			double slope = parallelEdge.x / parallelEdge.y;
			double intercept = init.x - slope * init.y;
			if ( Math.abs(start.y * slope  + intercept - start.x) > eps * segmentLength ) {
				return false;
			}
		}	
		return true;
	}
	
}