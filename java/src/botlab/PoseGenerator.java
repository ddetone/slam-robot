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

    //covariance
    double[][] sigmaB = new double[3][3];
    double[][] sigmaA = new double[3][3];
    double[][] sigmaT = new double[3][3];

    PoseGenerator()
    {
        this.lcm = LCM.getSingleton();
		lcm.subscribe("MOTOR_FEEDBACK", this);

        //initial uncertainty
        sigmaA = new double[][]{{0.00001,0,0},
                                {0,0.00001,0},
                                {0,0,0.00001}};
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


        double[] xyt_T = new double[3];
        xyt_T[0] = (encoder_delta[1] + encoder_delta[0]) / 2 * metersPerTick;
        xyt_T[1] = 0;
        xyt_T[2] = (encoder_delta[1] - encoder_delta[0]) / base * metersPerTick;

        encoder_prev[0] = encoder_curr[0];
        encoder_prev[1] = encoder_curr[1];


        //pose prev global
        double[] xyt_A = new double[3];
        xyt_A[0] = bot.xyt[0];
        xyt_A[1] = bot.xyt[1];
        xyt_A[2] = bot.xyt[2];

        double ca = Math.cos(xyt_A[2]);
        double sa = Math.sin(xyt_A[2]);

        double[] xyt_B = new double[3];
        xyt_B[0] = xyt_T[0]*ca - xyt_T[1]*sa + xyt_A[0];
        xyt_B[1] = xyt_T[0]*sa + xyt_T[1]*ca + xyt_A[1];
        xyt_B[2] = xyt_A[2] + xyt_T[2];

        computeCov(xyt_A, xyt_B, xyt_T);

        bot.xyt[0] = xyt_B[0];
        bot.xyt[1] = xyt_B[1];
        bot.xyt[2] = xyt_B[2];

        bot.utime = TimeUtil.utime();

        lcm.publish("6_POSE",bot);
        try{
            Thread.sleep(33);
        }catch(InterruptedException e)
        {}


    }


    public void computeCov(double[] xyt_A, double[] xyt_B, double[] xyt_T)
    {
        double alpha = 0.1;
        double beta  = 0.01;

        double sigmaT_xx = (alpha*xyt_T[0])*(alpha*xyt_T[0]);
        double sigmaT_tt = (alpha*xyt_T[2])*(alpha*xyt_T[2]);

        sigmaT = new double[][]{{sigmaT_xx, 0   , 0        },
                                {0        , beta, 0        },
                                {0        , 0   , sigmaT_tt}};

        double sa = Math.sin(xyt_A[2]);
        double ca = Math.cos(xyt_A[2]);
        double xt = xyt_T[0];
        double yt = xyt_T[1];

        double[][] Ja = new double[][]{{1, 0,-xt*sa-yt*ca },
                                        {0, 1, xt*ca-yt*sa },
                                        {0, 0, 1           }};

        double[][] Jt = new double[][]{{ca,-sa, 0 },
                                        {sa, ca, 0 },
                                        {0 , 0 , 1 }};

        double[][] Jt_T = LinAlg.transpose(Jt);
        double[][] Ja_T = LinAlg.transpose(Ja);

        sigmaB = LinAlg.add(LinAlg.matrixABC(Ja,sigmaA,Ja_T) , LinAlg.matrixABC(Jt,sigmaT,Jt_T));

        sigmaA = LinAlg.copy(sigmaB);

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
