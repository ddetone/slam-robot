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

public class Pimu extends VisEventAdapter implements LCMSubscriber 
{


	static VisWorld vw = new VisWorld();
    static VisLayer vl  = new VisLayer(vw);
    static VisCanvas vc = new VisCanvas(vl);
	static JFrame jf = new JFrame("Gyro Visualization");  
	  
    //CalibrateGyro cg = new CalibrateGyro();
	
	public static int[] prev_integrator = new int[8];
	public static int[] sum_angvel = new int[8];	
	public static int[] integrator = new int[8];
	public static double prev_time,time;
	public static int num_calibs;

	LCM lcm = LCM.getSingleton();
	pimu_t gyros;
	boolean calibrating;
	boolean calibdone;
	int clickcount;	
	
	Pimu()
    {
    	//Jfram and VisLayer initialization
		jf.setLayout(new BorderLayout());
		jf.add(vc, BorderLayout.CENTER);    
		vl.cameraManager.uiLookAt(
		        new double[] {-1, -1, 1 },
		        new double[] { 0,  0, 0.00000 },
		        new double[] { 0.13802,  0.40084, 0.90569 }, true);

		jf.setSize(800, 800);
		jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		jf.setVisible(true);
		vl.addEventHandler(this);   

    	//Initialize previous variables to zero
    	num_calibs = 0;
    	prev_time=0;
    	calibrating=false;
    	calibdone=false;
    	clickcount = 0;
    	for (int i=0; i<8; i++)
    	{
			prev_integrator[i] = 0;
			sum_angvel[i] = 0;
		}

		//Create singleton and subscribe to LCM	
		lcm.subscribe("PIMU", this);
		
    }
    
	public boolean keyTyped(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
    {
		char c = e.getKeyChar();        
		if(c =='c')
		{	
			System.out.printf("C\n");
			if ((clickcount % 2) == 0)
			{	
				System.out.printf("CALIBRATING\n");
				calibdone = false;
				calibrating = true;
			}
			else
			{
				System.out.printf("CALIBDONE\n");
				calibdone = true;
				calibrating = false;			
			}
		}
		clickcount++;
		return false;	
    }   
     
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
	
		try
		{
			if(channel.equals("PIMU"))
			{
			
				gyros = new pimu_t(dins);

				if (!calibrating && !calibdone)
				{
					updateData();
					System.out.printf("NOT CALIBRATED\n");
					for (int i=0; i<8; i++)
						sum_angvel[i] = 0;
					updateGUI();						
				}
				else if (calibdone && !calibrating)
				{
					updateData();
					updateGUI();
				}				
				else if (calibrating  && !calibdone)
				{
					System.out.printf("numcalibs:%d\n",num_calibs);
					num_calibs++;
					updateData();
					getAvgs();
				}

				
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
		
		
	}
	
	public void getAvgs()
	{	
		double timediff = time-prev_time;
		for (int i=0; i<8; i++)
		{
			double diff = (integrator[i]-prev_integrator[i])/timediff;
			sum_angvel[i] += diff;
		}
	}
	
    public void updateData()
    {
    	for (int i=0; i<8; i++)
			integrator[i] = gyros.integrator[i];
			
		time = gyros.utime_pimu;
    }	

	synchronized void updateGUI()
    {  
    	
    	//Intialize an Array of colors for bars
    	ArrayList<VzMesh.Style> meshColors = new ArrayList<VzMesh.Style>();
		for (int i=0; i<8; i++)
		{
			if (i%2 == 1)
				meshColors.add(i,new VzMesh.Style(Color.gray));
			else
				meshColors.add(i,new VzMesh.Style(Color.blue));
		}    
    	
		//Create Bars
     	double timediff = time-prev_time;
     	double[] sub = new double[8]; 	
    	ArrayList<VisObject> bars = new ArrayList<VisObject>();
    	for (int i=0; i<8; i++)
    	{
    		double diff = (integrator[i]-prev_integrator[i])/timediff;
    		if (num_calibs==0)
    		{
    			System.out.printf("NUM_Calibs is zero\n");
    			num_calibs++;
    		}
    		sub[i] = (sum_angvel[i]/num_calibs);
    		//System.out.printf("It:%d,Sum:%d,NumCalib:%d,Sub:%f\n",i,sum_angvel[i],num_calibs,sub[i]);

    		diff = diff - sub[i];

    		bars.add(i,	new VisChain(LinAlg.translate((0.05*i),0.0,0.001*diff),
						new VzBox(0.05,0.05,Math.abs(diff*0.002), meshColors.get(i))));
    	}
    		//System.out.println();    
    	//Create Buffer and swap
		VisWorld.Buffer vb = vw.getBuffer("chart");
		for (int i=0; i<8; i++)
		{
			vb.addBack(bars.get(i));
		}
    	vb.swap();
    	
    	//Update prev
    	prev_time = time;
		for (int i=0; i<8; i++)
			prev_integrator[i] = integrator[i];    	
    }
      	
	public static void main(String[] args) throws Exception
	{
		Pimu pg = new Pimu();
	
		while(true)
		{
            Thread.sleep(1000);
        }
        	

	} 
	
	public void printAngles()
	{
		double timediff = time-prev_time;
		System.out.printf("timediff:%f\n",timediff);
		for (int i=0; i<8; i++)
		{
			System.out.printf("integratordiff%d is:%f\n",i,
					(integrator[i]-prev_integrator[i])/timediff);	
		}
		System.out.printf("\n");	
	} 	 	
}	
    	
    	
    	
