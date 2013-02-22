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

public class PandaPublisher 
{
	
	LCM lcm = LCM.getSingleton();
	bot_status_t bot_status;


	public void run()
	{
		int x = 0, y = 0, t = 0;

		while(true)
		{
			bot_status_t bot_status = new bot_status_t();
			bot_status.utime = TimeUtil.utime();
			bot_status.xyt[0] = x++;
			bot_status.xyt[1] = y++;
			bot_status.xyt[2] = t++;

			lcm.publish("6_POSE",bot_status);
			try{
				Thread.sleep(33);
			}catch(InterruptedException e)
			{}
		}
	}

	public static void main(String[] args)
	{
		PandaPublisher pp = new PandaPublisher();
	
		pp.run();
		//lcm.subscribe("6_POSE", pl);
									
	}


}
