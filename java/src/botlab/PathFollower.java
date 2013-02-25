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


public class PathFollower implements LCMSubscriber
{

	LCM lcm;
	bot_status_t bot_status;

	//Robot is actively following if isFollow = true else if does not move
	static boolean isFollow = false;
	static double[] currXYT = new double[3];
	static double[] destXYT = new double[3];

	double Kp_turn = 0.7;
	double Kp = 1;

	PathFollower()
	{
		this.lcm =  LCM.getSingleton();
		lcm.subscribe("6_POSE",this);
		lcm.subscribe("6_WAYPOINT",this);
	}

	//goToPoint will follow a straight line path to the x,y coordinate from whereever it is
	//it should pulbish an LCM message saying it reached destination?
	void goToPoint(double x, double y)
	{
		destXYT[0] = x;
		destXYT[1] = y;
		isFollow = true;
	}

	//to stop the robot just in case
	void stop()
	{
		setMotorCommand(0.0F, 0.0F);
		isFollow = false;
	}

	void moveRobot()
	{
		double x_c = currXYT[0], y_c = currXYT[1];
		double x_d = destXYT[0], y_d = destXYT[1];

		double destAngle = Math.atan2((y_d - y_c),(x_d - x_c));
		double errorAngle = destAngle - currXYT[2];

		if(errorAngle > Math.PI)errorAngle-=2*Math.PI;
		else if(errorAngle < -Math.PI)errorAngle+=2*Math.PI;

		double errorDist = LinAlg.distance(new double[]{x_c,y_c}, new double[]{x_d, y_d});

		double left, right;

		if(Math.abs(errorAngle) > Math.toRadians(10))
		{
			right =  Kp_turn * errorAngle;
			left  = -Kp_turn * errorAngle;
			System.out.println("angle error:" + Math.toDegrees(errorAngle) 
					+ "  dest Angle:" + Math.toDegrees(destAngle) 
					+ "  curr Angle:" + Math.toDegrees(currXYT[2]));
		}
		else
		{
			System.out.println("distance error:" + errorDist);
			if(errorDist > 0.10)//10cm
			{
				right = 0.5;
				left  = 0.5;
			}
			else if(errorDist > 0.03)
			{
				right = 0.5 * errorDist;
				left  = 0.5 * errorDist;
			}
			else{
				stop();
				return;
			}


			double delta = Kp * errorAngle;
			if(delta > 0)
				left -= delta;
			else
				right -= delta;

		}

		setMotorCommand(left, right);


	}

	void setMotorCommand(double left, double right)
	{
		diff_drive_t motor = new diff_drive_t();

		motor.left_enabled = false;
		motor.right_enabled = false;

		motor.utime = TimeUtil.utime();

		left = LinAlg.clamp(left, -0.5, 0.5);
		right = LinAlg.clamp(right, -0.5, 0.5);

		motor.left = (float)left;
		motor.right = (float)right;

		lcm.publish("DIFF_DRIVE", motor);
	}

	//goToPoint(double x, double y, double theta)
	//{
	//}

	//Needs to subscribe to 6_POSE to know where it is
	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_POSE"))
			{
				bot_status = new bot_status_t(dins);
				currXYT[0] = bot_status.xyt[0];
				currXYT[1] = bot_status.xyt[1];
				currXYT[2] = bot_status.xyt[2];
				if(isFollow)
					moveRobot();
			}
			else if(channel.equals("6_WAYPOINT"))
			{
				xyt_t dest = new xyt_t(dins);
				goToPoint(dest.xyt[0],dest.xyt[1]);
			}

		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public static void main(String[] args) throws Exception
	{
		PathFollower pl = new PathFollower();

		//System.out.println(Math.atan2(12.0,0.0));
		while(true)
		{
			Thread.sleep(1000);
		}


	}


}
