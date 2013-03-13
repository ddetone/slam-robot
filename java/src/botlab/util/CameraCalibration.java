package botlab.util;

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
import april.jmat.geom.*;


public class CameraCalibration extends CordinateDescent
{
	
    public CameraCalibration(double[] _p, double[] _dp, double _tolerance)
    {
        super(_p,_dp,_tolerance);
    }
    
	static ArrayList<double[]> points = new ArrayList<double[]>();
	static ArrayList<double[]> pixels = new ArrayList<double[]>();

	static double[][] pointsN;
	static double[][] pixelsN;

    static double height = 0.215;
    
	//params = f cx cy theta
	public double error(double[] parameters)
	{
		double f = parameters[0];
		double cx = parameters[1];
		double cy = parameters[2];
		double th = parameters[3];

		//double[][] ext = LinAlg.rotateX(th);

		//double[][] intr = new double[][]{{f,0,cx,0},
		//								 {0,f,cy,0},
		//								 {0,0, 1,0}};
		
		
		double[][] pixelsPred = transformPoint2Pixel(f, cx, cy, th, pointsN);		
			
		normalize(pixelsPred);

		double error = 0;

		for(int i = 0; i < pixelsN[0].length; i++)
		{
			double[] vec1 = new double[]{pixelsN[0][i], pixelsN[1][i], pixelsN[2][i]};
			double[] vec2 = new double[]{pixelsPred[0][i], pixelsPred[1][i], pixelsPred[2][i]};

			error += LinAlg.distance(vec1,vec2);
		}

		return error;

	}


	public double[][] transformPoint2Pixel(double f, double cx, double cy, double th, double[][] pointsN)
	{
		double[][] ext = LinAlg.rotateX(th);

		double[][] intr = new double[][]{{f,0,cx,0},
	                                     {0,f,cy,0},
                                         {0,0, 1,0}};


       return(LinAlg.matrixABC(intr,ext,pointsN));

	}

	public void normalize(double[][] px)
	{
		for(int i = 0; i < px[0].length; i++)
		{
			px[0][i] = px[0][i] / px[2][i];
			px[1][i] = px[1][i] / px[2][i];
			px[2][i] = px[2][i] / px[2][i];
		}
	}

	public static void main(String[] args)
	{
		
		points.add(new double[]{2,0});
		points.add(new double[]{3,0});
		points.add(new double[]{4,0});
		points.add(new double[]{5,0});
		points.add(new double[]{2,1});
		points.add(new double[]{4,1});
		points.add(new double[]{3,2});
		points.add(new double[]{3,0.5});
		points.add(new double[]{2,-1});
		points.add(new double[]{4,-1});
		points.add(new double[]{3,-2});
		points.add(new double[]{3,-0.5});

		pixels.add(new double[]{620,689});
		pixels.add(new double[]{623,610});
		pixels.add(new double[]{623,571});
		pixels.add(new double[]{623,548});
		pixels.add(new double[]{958,695});
		pixels.add(new double[]{785,573});
		pixels.add(new double[]{1061,618});
		pixels.add(new double[]{731,611});
		pixels.add(new double[]{297,690});
		pixels.add(new double[]{455,571});
		pixels.add(new double[]{161,608});
		pixels.add(new double[]{511,609});

		assert(pixels.size() == points.size());

		for(int i = 0; i < points.size(); i++){
			points.get(i)[0] *= 0.3048;
			points.get(i)[1] *= 0.3048;
		}
		
		int n = pixels.size();

		pointsN = new double[4][n];
		pixelsN = new double[3][n];

		for(int i = 0 ; i < n ; i++)
		{
			pointsN[0][i] = points.get(i)[1];
			pointsN[1][i] = height;
			pointsN[2][i] = points.get(i)[0];
			pointsN[3][i] = 1;

			pixelsN[0][i] = pixels.get(i)[0];
			pixelsN[1][i] = pixels.get(i)[1];
			pixelsN[2][i] = 1;
		}

		//Parameters f cx cy theta
		double[] p = new double[]{660,620,440,0};
		double[] dp = new double[]{1,1,1,0.01};
		double tolerance = 0.000001;

		CameraCalibration cd = new CameraCalibration(p,dp,tolerance);

//		cd.n = n;

		double[] params = cd.getBestParams();
		
		System.out.println("Final Parameters");
		LinAlg.print(params);

		System.out.println("Pixels Predict");
		double[][] pixPred = cd.transformPoint2Pixel(params[0],params[1],params[2],params[3], pointsN);
		cd.normalize(pixPred);
		LinAlg.print(pixPred);

		
		System.out.println("Pixels Orig");
		LinAlg.print(pixelsN);

		System.out.println("Error");
		System.out.println(cd.error(params));
	}
}
