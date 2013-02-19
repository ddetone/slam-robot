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

public class PandaListener implements LCMSubscriber
{
	
	LCM lcm = LCM.getSingleton();
	bot_status_t bot_status;

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_POSE"))
			{
				bot_status = new bot_status_t(dins);
				System.out.println("utime:" + bot_status.utime);
				System.out.println("X:" + bot_status.xyt[0]);
				System.out.println("Y:" + bot_status.xyt[1]);
				System.out.println("T:" + bot_status.xyt[2]);
				System.out.println();

			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public static void main(String[] args)
	{
		PandaListener pl = new PandaListener();

		/* Subscribe to 6_POSE */
		LCM.getSingleton().subscribe("6_POSE", pl);
								  //      LCM.getSingleton().subscribe("BALLS_LIST", gui);
									
	}


}
