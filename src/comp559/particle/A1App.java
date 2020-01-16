package comp559.particle;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.io.File;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.List;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;
import com.jogamp.opengl.util.gl2.GLUT;

import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.JTextArea;
import javax.vecmath.Vector2d;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.FileSelect;
import mintools.swing.HorizontalFlowPanel;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.Interactor;
import mintools.viewer.SceneGraphNode;


/**
 * Provided code for particle system simulator.
 * This class provides the mouse interface for clicking and dragging particles, and the 
 * code to draw the system.  When the simulator is running system.advanceTime is called
 * to numerically integrate the system forward.
 * @author kry
 */
public class A1App implements SceneGraphNode, Interactor {

    private EasyViewer ev;
    
    private ParticleSystem system;
    
    private double maxDist = 150;
    
    private double minDist = 50;

    private double grabThresh = 10;

    /**
     * Entry point for application
     * @param args
     */
    public static void main(String[] args) {
        new A1App();        
    }
    
    /**
     * Creates the application / scene instance
     */
    public A1App() {
        system = new ParticleSystem();
        system.integrator = forwardEuler;
        ev = new EasyViewer( "COMP 559 W2011 - A1 Particle System", this, new Dimension(640,360), new Dimension(640,640) );
        ev.addInteractor(this);
    }
     
    @Override
    public void init(GLAutoDrawable drawable) {
        GL gl = drawable.getGL();
        gl.glEnable( GL.GL_BLEND );
        gl.glBlendFunc( GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA );
        gl.glEnable( GL.GL_LINE_SMOOTH );
        gl.glEnable( GL2.GL_POINT_SMOOTH );
        gl.glDisable( GL2.GL_LIGHTING );
        system.init(drawable);
    }
        
    @Override
    public void display(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2();
        EasyViewer.beginOverlay(drawable);
        
        if ( run.getValue() ) {
            for ( int i = 0; i < substeps.getValue(); i++ ) {
                system.advanceTime( stepsize.getValue() / substeps.getValue() );                
            }
        }
        
        system.display( drawable );
        
        if ( mouseDown ) {
            if ( ! grabbed ) {
                if ( ! run.getValue() ) {
                	// check particle pair line
                	if ( p1 != null && p2 != null ) { 
                		final Vector2d v = new Vector2d();
	                	v.sub( p1.p, p2.p );
	                	v.normalize();
	                	double d = Math.abs( v.x*(p1.p.y - ycurrent) - v.y*(p1.p.x - xcurrent) );
	                	closeToParticlePairLine = d < grabThresh;
                	}
                	if ( closeToParticlePairLine ) {
                		gl.glColor4d(0,1,1,.5);
                        gl.glLineWidth(3f);
                        gl.glBegin( GL.GL_LINES );
                        gl.glVertex2d( p1.p.x, p1.p.y );
                        gl.glVertex2d( p2.p.x, p2.p.y );
                        gl.glEnd();
                	} else {
	                    gl.glPointSize( 5f );
	                    gl.glLineWidth( 2f );
	                    if ( ! run.getValue() ) {                      
	                        drawLineToParticle( drawable, xcurrent, ycurrent, p1, d1 );
	                        drawLineToParticle( drawable, xcurrent, ycurrent, p2, d2 );
	                    }
                	}
                }
            } else {
                gl.glPointSize( 15f );
                gl.glColor4d(0,1,0,0.95);
                gl.glBegin( GL.GL_POINTS );
                gl.glVertex2d( p1.p.x, p1.p.y );
                gl.glEnd();        
            }
        } else {
            if ( mouseInWindow ) {
                findCloseParticles( xcurrent, ycurrent );
                if ( p1 != null && d1 < grabThresh ) {
                    gl.glPointSize( 15f );
                    gl.glColor4d(0,1,0,0.95);
                    gl.glBegin( GL.GL_POINTS );
                    gl.glVertex2d( p1.p.x, p1.p.y );
                    gl.glEnd();        
                } else if ( p1 != null && p2 != null ) {
                	final Vector2d v = new Vector2d();
                	v.sub( p1.p, p2.p );
                	v.normalize();
                	double d = Math.abs( v.x*(p1.p.y - ycurrent) - v.y*(p1.p.x - xcurrent) );
                	closeToParticlePairLine = d < grabThresh;
                	if ( closeToParticlePairLine ) {
                        gl.glColor4d(0,1,1,.5);
                        gl.glLineWidth(3f);
                        gl.glBegin( GL.GL_LINES );
                        gl.glVertex2d( p1.p.x, p1.p.y );
                        gl.glVertex2d( p2.p.x, p2.p.y );
                        gl.glEnd();
                    }
                }
            }
        }
	        
        String text = system.toString() + "\n" + 
                      "h = " + stepsize.getValue() + "\n" +
                      "substeps = " + substeps.getValue() + "\n" +
                      "computeTime = " + system.computeTime;     
        gl.glColor3f(1, 1, 1);
        EasyViewer.printTextLines( drawable, text, 10, 20, 13, GLUT.BITMAP_8_BY_13 );
        EasyViewer.endOverlay(drawable);    

        if ( run.getValue() || stepped ) {
            stepped = false;        
            if ( record.getValue() ) {
                // write the frame
                File file = new File( "stills/" + dumpName + format.format(nextFrameNum) + ".png" );                                             
                nextFrameNum++;
                file = new File(file.getAbsolutePath().trim());
                ev.snapshot(drawable, file);
            }
        }
    }

    private BooleanParameter record = new BooleanParameter( "record (press ENTER in canvas to toggle)", false );
    
    /** 
     * boolean to signal that the system was stepped and that a 
     * frame should be recorded if recording is enabled
     */
    private boolean stepped = false;
        
    private String dumpName = "dump";
    
    private int nextFrameNum = 0;
    
    private NumberFormat format = new DecimalFormat("00000");
    
    /**
     * draws a line from the given point to the given particle
     * @param drawable
     * @param x
     * @param y
     * @param p
     * @param d
     */
    private void drawLineToParticle( GLAutoDrawable drawable, double x, double y, Particle p, double d ) {

        if ( p == null ) return;
        if ( d > maxDist ) return;
        
        GL2 gl = drawable.getGL().getGL2();
        
        double col = d < minDist ? 1 : (maxDist-d) / (maxDist-minDist);
        gl.glColor4d( 1-col,0,col,0.75f);
        gl.glBegin(GL.GL_LINES);
        gl.glVertex2d( x, y );
        gl.glVertex2d( p.p.x, p.p.y );
        gl.glEnd();    
    }
    
    private BooleanParameter run = new BooleanParameter( "simulate", false );
    private DoubleParameter stepsize = new DoubleParameter( "step size", 0.05, 1e-5, 1 );
    private IntParameter substeps = new IntParameter( "sub steps", 1, 1, 100);
    
    @Override
    public JPanel getControls() {
    	VerticalFlowPanel vfp = new VerticalFlowPanel();
    	
        JTextArea ta = new JTextArea(
        		"   SPACE   - start/stop simulation \n" +
        		"   ENTER   - start/stop recording to stills \n" +
        		"   S       - step simulation \n" +
        		"   UP/DOWN - increase decrease sub-steps \n" +
         		"   Z       - zero all velocities \n" +
        		"   R       - reset to initial conditions \n" +
        		"   C       - clear all particles \n" +
        		"   DEL     - remove closest paticle to mouse " +
        		"   1 	    - use explicit forward Euler \n " +
        		"   2 	    - use explicit midpoint \n " +
        		"   3 	    - use explicit modified midpoint \n " +
        		"   4 	    - use explicit symplectic Euler \n " +
         		"   5 	    - use explicit RK4 \n " +
         		"   6 	    - use implicit Backward Euler \n " +
         		"   ESC     - quit \n\n" +
        		"   Mouse Controls \n" +
        		"     - left click to create particles \n" +
        		"     - left click highlighted particle to pin/unpin \n" +
        		"     - left drag to move a highlighted particle \n" +
        		"     - left click highlighted spring to create or destroy \n" );
        ta.setEditable(false);
        vfp.add( ta );
        
    	HorizontalFlowPanel hrfp = new HorizontalFlowPanel();
        
    	JButton res1 = new JButton("640x360");
    	hrfp.add( res1);
        res1.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                ev.glCanvas.setSize( 640, 360 );
                ev.frame.setSize( ev.frame.getPreferredSize() );
            }
        });       
    	 JButton res2 = new JButton("1280x720");
    	 hrfp.add( res2);
         res2.addActionListener( new ActionListener() {
             @Override
             public void actionPerformed(ActionEvent e) {                
                 ev.glCanvas.setSize( 1280, 720 );
                 ev.frame.setSize( ev.frame.getPreferredSize() );
             }
         });   
         vfp.add( hrfp.getPanel());
        
         HorizontalFlowPanel hfp = new HorizontalFlowPanel();
        JButton create1 = new JButton("test system 1");
        hfp.add( create1 );
        create1.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                system.createSystem(1);
            }
        });
        
        JButton create2 = new JButton("test system 2");
        hfp.add( create2 );
        create2.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                system.createSystem(2);
            }
        });
        
        JButton create3 = new JButton("test system 3");
        hfp.add( create3 );
        create3.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                system.createSystem(3);
            }
        });
        
        JButton save = new JButton("SAVE");
        hfp.add( save );
        save.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            	// note, xml show here as the extension, but you might change this
            	// to something else if you choose a binary or text format.
            	File f = FileSelect.select(".xml", "particle system", "save", ".", true );
            	if ( f != null ) {
            		// TODO: Bonus, do something with this file!
            	}
            }
        });

        
        JButton load= new JButton("LOAD");
        hfp.add( load );
        load.addActionListener( new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
            	// note, xml show here as the extension, but you might change this
            	// to something else if you choose a binary or text format.

            	File f = FileSelect.select(".xml", "particle system", "load", ".", true );
            	if ( f != null ) {
            		// TODO: Bonus, do something with this file!
            	}
            }
        });

        
        vfp.add( hfp.getPanel() );
        
        vfp.add( record.getControls() );
        vfp.add( run.getControls() );
        vfp.add( stepsize.getSliderControls(true) );
        vfp.add( substeps.getControls() );
        vfp.add( system.getControls() );
        
       
        
        return vfp.getPanel();
    }
    
    private Particle p1 = null;
    private Particle p2 = null;
    private double d1 = 0;
    private double d2 = 0;
    
    /**
     * Finds the two closest particles for showing potential spring connections
     * @param x 
     * @param y 
     */
    private void findCloseParticles( int x, int y ) {
        List<Particle> particles = system.getParticles();
        p1 = null;
        p2 = null;
        d1 = 0;
        d2 = 0;
        if ( particles.size() > 0 ) {
            for ( Particle p : particles ) {
                double d = p.distance( x, y );
                if ( p1 == null || d < d1 ) {
                    p2 = p1; d2 = d1; p1 = p; d1 = d;
                } else if ( p2 == null || d < d2 ) {
                    p2 = p; d2 = d;
                }
            }      
        }
    }  
    
    private int xdown = 0;
    private int ydown = 0;
    private int xcurrent = 0;
    private int ycurrent = 0;
    private boolean mouseDown = false;
    private boolean mouseInWindow = false;
    private boolean grabbed = false;
    private boolean wasPinned = false;
    private boolean closeToParticlePairLine = false;
    
    @Override
    public void attach(Component component) {
        component.addMouseMotionListener( new MouseMotionListener() {
            @Override
            public void mouseDragged(MouseEvent e) {
                xcurrent = e.getPoint().x;
                ycurrent = e.getPoint().y;
                if ( grabbed ) {
                    p1.p.set( xcurrent, ycurrent );
                    p1.v.set( 0, 0 ); 
                    if ( ! run.getValue() ) {
                        p1.p0.set( p1.p );
                        p1.v0.set( p1.v );
                        for ( Spring s : p1.springs ) {
                            s.recomputeRestLength();
                        }
                    }
                } else {
                    findCloseParticles(xcurrent, ycurrent);
                }
            }
            @Override
            public void mouseMoved(MouseEvent e) {
                xcurrent = e.getPoint().x;
                ycurrent = e.getPoint().y;
            }
        } );
        component.addMouseListener( new MouseListener() {
            @Override
            public void mouseClicked(MouseEvent e) {
                // do nothing
                mouseInWindow = true;
            }
            @Override
            public void mouseEntered(MouseEvent e) {
                mouseInWindow = true;
            }
            @Override
            public void mouseExited(MouseEvent e) {
                // clear the potential spring lines we're drawing
                mouseInWindow = false;
            }
            @Override
            public void mousePressed(MouseEvent e) {
                mouseInWindow = true;
                xdown = e.getPoint().x;
                ydown = e.getPoint().y;
                xcurrent = xdown;
                ycurrent = ydown;
                mouseDown = true;
                findCloseParticles(xcurrent, ycurrent);
                if ( p1 != null && d1 < grabThresh ) {
                    wasPinned = p1.pinned;
                    p1.pinned = true;                    
                    grabbed = true;
                    p1.p.set( xcurrent, ycurrent );
                    p1.v.set( 0, 0 ); 
                }
            }
            @Override
            public void mouseReleased(MouseEvent e) {
                mouseDown = false;
                
                	if ( ! grabbed && ! run.getValue() ) {
	                    double x = e.getPoint().x;
	                    double y = e.getPoint().y;
	                    // were we within the threshold of a spring?
	                    if ( closeToParticlePairLine ) {
	                    	if ( !system.removeSpring( p1, p2 ) ) {
	                			system.createSpring( p1, p2 );
	                		}
	                    } else {
		                    Particle p = system.createParticle( x, y, 0, 0 );
		                    if ( p1 != null && d1 < maxDist ) {
		                        system.createSpring( p, p1 );
		                    }
		                    if ( p2 != null && d2 < maxDist ) {
		                        system.createSpring( p, p2 );
		                    }  
	                    }
	                } else if ( grabbed && p1 != null ) {
	                	p1.pinned = ! wasPinned;
	                } 
                
                grabbed = false;
            }
        } );
        component.addKeyListener( new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                if ( e.getKeyCode() == KeyEvent.VK_SPACE ) {
                    run.setValue( ! run.getValue() ); 
                } else if ( e.getKeyCode() == KeyEvent.VK_S ) {
                    for ( int i = 0; i < substeps.getValue(); i++ ) {
                        system.advanceTime( stepsize.getValue() / substeps.getValue() );                
                    }
                    stepped = true;
                } else if ( e.getKeyCode() == KeyEvent.VK_R ) {
                    system.resetParticles();                    
                } else if ( e.getKeyCode() == KeyEvent.VK_C ) {                   
                    system.clearParticles();
                    p1 = null;
                    p2 = null;
                } else if ( e.getKeyCode() == KeyEvent.VK_1 ) {
                    system.explicit.setValue(true);
                    system.integrator = forwardEuler;                    
                } else if ( e.getKeyCode() == KeyEvent.VK_2 ) {
                    system.explicit.setValue(true);
                    system.integrator = midpoint;
                } else if ( e.getKeyCode() == KeyEvent.VK_3 ) {
                    system.explicit.setValue(true);
                    system.integrator = modifiedMidpoint;
                } else if ( e.getKeyCode() == KeyEvent.VK_4 ) {
                    system.explicit.setValue(true);
                    system.integrator = symplecticEuler;
                } else if ( e.getKeyCode() == KeyEvent.VK_5 ) {
                    system.explicit.setValue(true);
                    system.integrator = rk4;
                } else if ( e.getKeyCode() == KeyEvent.VK_6 ) {
                    system.explicit.setValue(false);                    
                } else if ( e.getKeyCode() == KeyEvent.VK_ESCAPE ) {
                    ev.stop();
                } else if ( e.getKeyCode() == KeyEvent.VK_DELETE ) {
                	findCloseParticles( xcurrent, ycurrent );
                    if ( p1 != null && d1 < grabThresh ) {
                		system.remove( p1 );
                	}
                } else if ( e.getKeyCode() == KeyEvent.VK_Z ) {
                	for ( Particle p : system.getParticles() ) {
                		p.v.set(0,0);
                	}
                } else if ( e.getKeyCode() == KeyEvent.VK_ENTER ) {
                    record.setValue( ! record.getValue() );
                } else if ( e.getKeyCode() == KeyEvent.VK_UP ) {
                    substeps.setValue( substeps.getValue() + 1 );
                } else if ( e.getKeyCode() == KeyEvent.VK_DOWN ) {
                    substeps.setValue( substeps.getValue() - 1 );
                }
            }
        } );
    }
    
    private ForwardEuler forwardEuler = new ForwardEuler();    
    private Midpoint midpoint = new Midpoint();
    private ModifiedMidpoint modifiedMidpoint = new ModifiedMidpoint();
    private RK4 rk4 = new RK4();
    private SymplecticEuler symplecticEuler = new SymplecticEuler();
    
}
