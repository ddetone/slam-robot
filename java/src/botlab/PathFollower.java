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
import botlab.util.*;

import lcm.lcm.*;


public class PathFollower implements LCMSubscriber
{

	final boolean verbose = false;
	final boolean verbose2 = true;
	LCM lcm;
	bot_status_t bot_status;

	
	//Robot is actively following if isFollow = true else if does not move
	static boolean isFollow = false;
	static double[] currXYT = new double[3];
	static double[] currDotXYT = new double[3];
	static double[] destXYT = new double[3];

	static final double MAX_SPEED = 0.35;
	
	double Kp_turn = 0.7;
	double Kp = 1;
	double Kd_turn = 0.001;
	double Kd = 0.001;
	
	//The PID controller for finer turning
	double[] KPID = new double[]{1, 0.001, 0.5};
	PidController pidAngle = new PidController(KPID[0], KPID[1], KPID[2]);


	PathFollower()
	{
		this.lcm =  LCM.getSingleton();
		lcm.subscribe("6_POSE",this);
		lcm.subscribe("6_WAYPOINT",this);
	}

	//goToPoint will follow a straight line path to the x,y coordinate from whereever it is
	//it should pulbish an LCM message saying it reached destination?
	void goToPoint(double[] xyt)
	{
		destXYT[0] = xyt[0];
		destXYT[1] = xyt[1];
		destXYT[2] = xyt[2];
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

		//This errorAngle is the orientation of the robot its needs to go forward on
		double destAngle = Math.atan2((y_d - y_c),(x_d - x_c));
		double errorAngle = destAngle - currXYT[2];

		//This error angle is the final orientation of the robot when it reached its destination. This will only be used when the robot is within 3cm of the final robot orientation
		double errorAngleFinal = destXYT[2] - currXYT[2];

		//errorAngle = Math	(Math.Pi * 2);
		while(errorAngle > Math.PI)errorAngle -= 2 * Math.PI;
		while(errorAngle < -Math.PI)errorAngle += 2 * Math.PI;

		double errorDist = LinAlg.distance(new double[]{x_c,y_c}, new double[]{x_d, y_d});
		double left, right;

		if(Math.abs(errorAngle) > Math.toRadians(10))//First Orient the robot to face moving direction 
		{
			right =  Kp_turn * errorAngle - Kd_turn * currDotXYT[2];
			left  = -Kp_turn * errorAngle + Kd_turn * currDotXYT[2];

			if(verbose2)
				System.out.println("Initial Angle Orientation");

			if(verbose)System.out.println("angle error:" + Math.toDegrees(errorAngle) 
					+ "  dest Angle:" + Math.toDegrees(destAngle) 
					+ "  curr Angle:" + Math.toDegrees(currXYT[2]));
		}
		else //now in correct orientation, start moving forward
		{

			if(verbose2)
				System.out.println("Moving Forward");

			if(verbose)System.out.println("distance error:" + errorDist);
			//Far from robot go straight(full speed)
			if(errorDist > 0.10)//10cm
			{
				right = MAX_SPEED;
				left  = MAX_SPEED;
			}
			else if(errorDist > 0.03)//closing in on destination(start slowing down)
			{
				right = MAX_SPEED * errorDist;
				left  = MAX_SPEED * errorDist;
			}
			//Now instead of stopping(when the robot is 3 cm within its goal), we will turn to the angle specified in xyt
			else{
				if(errorAngleFinal < Math.toRadians(1))
				{
					if(verbose2)
						System.out.println("Final PID");

					double pid = pidAngle.getOutput(destXYT[2]-currXYT[2]);
					right = pid;
					left = -pid;
				}
				else{
					
					if(verbose2)
						System.out.println("STOP-----------------------------------------!!!!!!");
					//Reset Controller for next run
					pidAngle.resetController();
					stop();
					return;
				}
			}

			double delta = Kp * errorAngle - Kd * currDotXYT[2];
			if(delta > 0)
				left -= delta;
			else
				right -= delta;

		}

		setMotorCommand(left, right);
	}
/*
	void rotateBot(int rotations)
	{
		//double curangle = currXYT[2];

		double total_radians = rotations * (2*Math.PI);

		while (true) //counterclockwise is positive
		{
			if (currXYT[2] >= total_radians)
			{			
				stop();
				return;
			}
			setMotorCommand(0.0F,0.5F);
		}

	}
*/
	void setMotorCommand(double left, double right)
	{
		diff_drive_t motor = new diff_drive_t();

		motor.left_enabled = false;
		motor.right_enabled = false;

		motor.utime = TimeUtil.utime();

		left = LinAlg.clamp(left, -MAX_SPEED, MAX_SPEED);
		right = LinAlg.clamp(right, -MAX_SPEED, MAX_SPEED);

		motor.left = (float)left;
		motor.right = (float)right;

		lcm.publish("6_DIFF_DRIVE", motor);
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
				currDotXYT[0] = bot_status.xyt_dot[0];
				currDotXYT[1] = bot_status.xyt_dot[1];
				currDotXYT[2] = bot_status.xyt_dot[2];
				
				if(isFollow)
					moveRobot();
			}
			else if(channel.equals("6_WAYPOINT"))
			{
				xyt_t dest = new xyt_t(dins);
				goToPoint(dest.xyt);
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
