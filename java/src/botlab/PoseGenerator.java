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
	//This class would publish pose messages periadically

	LCM lcm;
	motor_feedback_t motors;

    public static int[] encoder_curr = new int[2];
    public static int[] encoder_prev = new int[2];

    //pose global
    //public static double poseG[] = new double[3];
    public static bot_status_t bot = new bot_status_t();

    //Determined emperically
    static final double metersPerTick = 0.000194;
    static final double base = 18.8/100.0;

    static boolean first = true;

    PoseGenerator()
    {
        this.lcm = LCM.getSingleton();
		lcm.subscribe("MOTOR_FEEDBACK", this);
    }

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("MOTOR_FEEDBACK"))
			{
				motors = new motor_feedback_t(dins);
				generatePose();

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

    public void generatePose()
    {

        encoder_curr[0] = motors.encoders[0];
        encoder_curr[1] = motors.encoders[1];

        if(first)
        {
            encoder_prev[0] = encoder_curr[0];
            encoder_prev[1] = encoder_curr[1];
            first = false;
        }

        double encoder_delta[] = new double[]{-(encoder_curr[0] - encoder_prev[0]), (encoder_curr[1] - encoder_prev[1])};

        double x_T = (encoder_delta[1] + encoder_delta[0]) / 2 * metersPerTick;
        double y_T = 0;
        double t_T = (encoder_delta[1] - encoder_delta[0]) / base * metersPerTick;

        encoder_prev[0] = encoder_curr[0];
        encoder_prev[1] = encoder_curr[1];


        //pose global
        double x_A = bot.xyt[0];
        double y_A = bot.xyt[1];
        double t_A = bot.xyt[2];

        double ca = Math.cos(t_A);
        double sa = Math.sin(t_A);

        double x_B = x_T*ca - y_T*sa + x_A;
        double y_B = x_T*sa + y_T*ca + y_A;
        double t_B = t_A + t_T;


        bot.xyt[0] = x_B;
        bot.xyt[1] = y_B;
        bot.xyt[2] = t_B;

        bot.utime = TimeUtil.utime();

        lcm.publish("6_POSE",bot);
        try{
            Thread.sleep(33);
        }catch(InterruptedException e)
        {}


    }


	public static void main(String[] args) throws Exception
	{
		PoseGenerator pg = new PoseGenerator();

		/* Subscribe to 6_POSE */
		while(true){
            Thread.sleep(1000);
        }
	}


}
