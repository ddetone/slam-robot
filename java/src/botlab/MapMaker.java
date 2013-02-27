package botlab;

import april.graph.*;
import lcm.lcm.*;
import april.jmat.*;
import april.util.*;
import java.io.*;

import botlab.lcmtypes.*;

public class MapMaker implements LCMSubscriber
{

	LCM lcm;
	Graph g;

	MapMaker(boolean showGUI){
		g = new Graph();
		lcm = LCM.getSingleton();

		lcm.subscribe("6_POSE",this);
		lcm.subscribe("6_FEATURES",this);
	}


	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_POSE"))
			{
				bot_status_t bot = new bot_status_t(dins);
			}
			if(channel.equals("6_FEATURES"))
			{
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}
	
}
