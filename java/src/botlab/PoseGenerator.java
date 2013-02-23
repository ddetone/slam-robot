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

public class PoseGenerator implements LCMSubscriber
{
	
	static LCM lcm = LCM.getSingleton();
	motor_feedback_t motors;

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("MOTOR_FEEDBACK"))
			{
				motors = new motor_feedback_t(dins);
				//generatePose();
//				System.out.println("utime:" + bot_status.utime);
//				System.out.println("X:" + bot_status.xyt[0]);
//				System.out.println("Y:" + bot_status.xyt[1]);
//				System.out.println("T:" + bot_status.xyt[2]);
//				System.out.println();

			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public static void main(String[] args)
	{
		PoseGenerator pl = new PoseGenerator();

		/* Subscribe to 6_POSE */
		lcm.subscribe("6_POSE", pl);
		while(true);
	}


}
