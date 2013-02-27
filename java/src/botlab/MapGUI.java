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

public class MapGUI implements LCMSubscriber
{
	JFrame jf = new JFrame("RobotGUI");

    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);

    ParameterGUI pg = new ParameterGUI();

	LCM lcm;
	map_t map;

    MapGUI()
    {
    	// Determine which slider values we want
		pg.addDoubleSlider("decay_dist", "decay_dist (marginalizing)", -10.0, 10.0, MapBuilder.DEFAULT_DECAY_DIST);
		pg.addDoubleSlider("decay_rate", "decay_rate (marginalizing)", -10.0, 10.0, MapBuilder.DEFAULT_DECAY_RATE);
		pg.addDoubleSlider("inverse_knowledge_dist", "inverse_knowledge_dist (unused)", -10.0, 10.0, MapBuilder.DEFAULT_INV_KNOWLEDGE_DIST);
		pg.addDoubleSlider("inverse_cost_decay", "inverse_cost_decay", -10.0,10.0, MapBuilder.DEFAULT_INV_COST_DECAY);
		
		
		
		pg.addListener(new ParameterListener() {
			public void parameterChanged(ParameterGUI pg, String name)
			{
				if(name == "decay_dist" || name == "decay_rate" || name == "inverse_knowledge_dist" || name == "inverse_cost_decay"){
					parameter_t p = new parameter_t();
					p.name = name;
					p.value = pg.gd(name);
					lcm.publish("6_PARAM",p);
				}
			}
		});

        this.lcm =  LCM.getSingleton();
        lcm.subscribe("6_MAP",this);

        jf.setLayout(new BorderLayout());
        jf.add(vc, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);

        jf.setSize(800,600);
        jf.setVisible(true);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        //vl.cameraManager.uiLookAt(new double[] {-0.21867, -1.71540, 10.96653 },
        //                          new double[] {-0.21867, -1.71540, 0.00000 },
        //                          new double[] { 0.00000,   1.00000, 0.00000 }, true);

        vl.cameraManager.uiLookAt(new double[] {-2.19919, -6.22111, 4.10494 },
                                  new double[] {-0.02769, 3.86996,  0.00000 },
                                  new double[] {0.07774, 0.36127,  0.92922 }, true);

        VisWorld.Buffer vb = vw.getBuffer("Ground");
        
        vb.swap();
    }

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_MAP"))
			{
				map = new map_t(dins);
				drawMap();
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public void drawMap()
    {
    	VisWorld.Buffer vb = vw.getBuffer("Robot");
        for(int i = 0; i < map.size; ++i){
        	for(int j = 0; j < map.size; ++j){
        		VzBox mapBox = new VzBox(map.scale,map.scale,(map.cost[i][j]/map.max)*map.scale*5, new VzMesh.Style(Color.red));
        		VisObject vo_mapBox = new VisChain(LinAlg.translate(i*map.scale,j*map.scale,0.0),mapBox);
        		vb.addBack(vo_mapBox);
        	}
        }

        vb.swap();
    }

	public static void main(String[] args) throws Exception
	{
		MapGUI mgui = new MapGUI();

        while(true)
        {
            Thread.sleep(1000);
        }


	}


}
