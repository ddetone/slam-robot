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

public class Pimu implements LCMSubscriber
{


	VisWorld vw = new VisWorld();
    VisLayer vl  = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);
	
	public static int[] prev_integrator = new int[8];
	public static int[] integrator = new int[8];
	public static double prev_time,time;
	public statis int num_msgs;
	
	LCM lcm;
	pimu_t gyros;	
	
	Pimu()
    {
    	//Jfram and VisLayer initialization
		JFrame jf = new JFrame();
		jf.setLayout(new BorderLayout());
		jf.add(vc, BorderLayout.CENTER);    
		vl.cameraManager.uiLookAt(
		        new double[] {-1, -1, 1 },
		        new double[] { 0,  0, 0.00000 },
		        new double[] { 0.13802,  0.40084, 0.90569 }, true);

		jf.setSize(800, 800);
		jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		jf.setVisible(true);    
    
    	//Initialize previous variables to zero
    	num_msgs = 0;
    	prev_time=0;
    	for (int i=0; i<8; i++)
			prev_integrator[i] = 0;
			
		//Create singleton and subscribe to LCM	
       	this.lcm = LCM.getSingleton();
		lcm.subscribe("PIMU", this);
    }
    
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("PIMU"))
			{
				gyros = new pimu_t(dins);
				updateData();
				//printAngles();
				update();
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
		
		
	}
	
    public void updateData()
    {
    	for (int i=0; i<8; i++)
			integrator[i] = gyros.integrator[i];
			
		time = gyros.utime_pimu;
    }	

	synchronized void update()
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
    	ArrayList<VisObject> bars = new ArrayList<VisObject>();
    	for (int i=0; i<8; i++)
    	{
    		double diff = (integrator[i]-prev_integrator[i])/timediff;
    		bars.add(i,	new VisChain(LinAlg.translate((0.05*i),0.0,0.001*diff),
						new VzBox(0.05,0.05,Math.abs(diff*0.002), meshColors.get(i))));
    	}
    
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

		/* Wait 1 sec to subscribe to PIMU */
		while(true){
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
    	
    	
    	
