package botlab;

import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import javax.swing.*;

import april.jcam.*;
import april.util.*;
import april.jmat.*;
import april.vis.*;
import april.image.*;
import april.jmat.geom.*;

import botlab.lcmtypes.*;

import lcm.lcm.*;

public class RobotGUI implements LCMSubscriber
{

    JFrame jf = new JFrame("RobotGUI");

    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);

    ParameterGUI pg = new ParameterGUI();

	LCM lcm;
	bot_status_t bot_status;

    RobotGUI()
    {
        this.lcm =  LCM.getSingleton();
        lcm.subscribe("6_POSE",this);

        jf.setLayout(new BorderLayout());
        jf.add(vc, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);

        jf.setSize(800,600);
        jf.setVisible(true);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        vl.cameraManager.uiLookAt(new double[] {-0.21867, -1.71540, 10.96653 },
                                  new double[] {-0.21867, -1.71540, 0.00000 },
                                  new double[] { 0.00000,   1.00000, 0.00000 }, true);

        VisWorld.Buffer vb = vw.getBuffer("Ground");
        vb.addBack(new VisChain(LinAlg.translate(0,0,-0.025),new VzBox(30,30,0.05,new VzMesh.Style(Color.darkGray))));
        vb.swap();

    }

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_POSE"))
			{
				bot_status = new bot_status_t(dins);
                drawRobot();
                drawCovariance();
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

    public void drawRobot()
    {
        double[] xyt = new double[3];
        xyt[0] = bot_status.xyt[0];
        xyt[1] = bot_status.xyt[1];
        xyt[2] = bot_status.xyt[2];

        double wheelRadius = 0.04;
        VzBox base = new VzBox(0.20,0.25,0.10, new VzMesh.Style(Color.red));
        VisObject vo_base = new VisChain(LinAlg.translate(0,0.08,0.12),base);

        VzBox cameraBase = new VzBox(0.06,0.02,0.05, new VzMesh.Style(Color.white));
        VisObject vo_cameraBase = new VisChain(LinAlg.translate(0,0,0.19),cameraBase);

        VzCylinder wheels = new VzCylinder(wheelRadius,0.01, new VzMesh.Style(Color.white));
        VisObject vo_wheels = new VisChain(LinAlg.rotateY(Math.PI/2),LinAlg.translate(-wheelRadius,0,0.10),wheels,LinAlg.translate(0,0,-0.20),wheels);

        double castorRad = 0.03;
        VzCylinder castor = new VzCylinder(castorRad,0.02, new VzMesh.Style(Color.black));
        VisObject vo_castor = new VisChain(LinAlg.rotateY(Math.PI/2), LinAlg.translate(-castorRad,0.16,0),castor);

        VisChain pandaBot = new VisChain();

        pandaBot.add(vo_base,vo_cameraBase,vo_wheels,vo_castor);

        VisWorld.Buffer vb = vw.getBuffer("Robot");
        vb.addBack(new VzAxes());
        //vb.addBack(new VisChain(LinAlg.translate(xyt[0],xyt[1],0), LinAlg.rotateZ(xyt[2]-Math.PI/2),new VzTriangle(0.25,0.4,0.4,new VzMesh.Style(Color.GREEN))));
        vb.addBack(new VisChain(LinAlg.translate(xyt[0],xyt[1],0), LinAlg.rotateZ(xyt[2]-Math.PI/2),pandaBot));

        vb.swap();
    }

    public void drawCovariance()
    {
        //double[] eigTheta = computerEigenValues();
        VisWorld.Buffer vb = vw.getBuffer("Covariance Ellipse");
        double[][] cov22 = new double[][]{{bot_status.cov[0][0], bot_status.cov[0][1]},
                                          {bot_status.cov[1][0], bot_status.cov[1][1]}};

        vb.addBack(new VisChain(LinAlg.translate(0,0,0.01),new VzEllipse(new double[]{bot_status.xyt[0],bot_status.xyt[1]}, cov22, new VzMesh.Style(Color.black))));
        vb.swap();
    }
    //returns two eigen values (first one larger) and theta of max eigen vector
    /*double[] computeEigenValues()
    {
        double[][] cov = bot_status.cov;
        double a = cov[0][0];
        double b = cov[0][1];
        double d = cov[1][1];
        double e1 = (a+d)/2 + (Math.sqrt((a+d)*(a+d) - 4*(a*d-b*b)))/2;
        double e2 = (a+d)/2 - (Math.sqrt((a+d)*(a+d) - 4*(a*d-b*b)))/2;
        double theta = Math.atan2(2*b/(a-d));
        if(e1>e2)
            return(new double[]{e1,e2,theta});
        else
            return(new double[]{e2,e1,theta});
    }*/

	public static void main(String[] args) throws Exception
	{
		RobotGUI pl = new RobotGUI();

        while(true)
        {
            Thread.sleep(1000);
        }


	}


}
