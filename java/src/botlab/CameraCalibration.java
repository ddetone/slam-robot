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
import april.jmat.geom.*;


public class CameraCalibration
{
	
	public static double[][] getJacobian(double f, double cx, double cy, double phi, double Tx, double Ty, double Tz, double X, double Y, double Z){
		
		double[][] J = {{Tx + X,
				Tz + Z*Math.cos(phi) - Y*Math.sin(phi),
				0,
				-Z*cx*Math.sin(phi) - Y*cx*Math.cos(phi), 
				f, 
				0, 
				cx},
				{Ty + Y*Math.cos(phi) + Z*Math.sin(phi),
				0,
				Tz + Z*Math.cos(phi) - Y*Math.sin(phi),
				Z*(f*Math.cos(phi) - cy*Math.sin(phi)) - Y*(cy*Math.cos(phi) + f*Math.sin(phi)),
				0,
				f,
				cy},
				{0,
				0,
				0,
				-Y*Math.cos(phi) - Z*Math.sin(phi), 
				0, 
				0,
				1}};
		return J;
		
	}


	public static void main(String args[]){
		Matrix A = new Matrix(36,7);
		Matrix b = new Matrix(36,1);
		Matrix x = new Matrix(7,1);
		
		ArrayList<double[]> points = new ArrayList<double[]>();
		ArrayList<double[]> pixels = new ArrayList<double[]>();
		
		points.add(new double[]{0,0,2});
		points.add(new double[]{0,0,3});
		points.add(new double[]{0,0,4});
		points.add(new double[]{0,0,5});
		points.add(new double[]{1,0,2});
		points.add(new double[]{1,0,4});
		points.add(new double[]{2,0,3});
		points.add(new double[]{0.5,0,3});
		points.add(new double[]{-1,0,2});
		points.add(new double[]{-1,0,4});
		points.add(new double[]{-2,0,5});
		points.add(new double[]{-0.5,0,3});
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
		int numCorrespondences = pixels.size();
		final int numIterations = 10000;
		
		for(int i = 0; i < points.size(); i++){
			points.get(i)[0] *= 0.3048;
			//points.get(i)[1] *= 0.3048;
			points.get(i)[2] *= 0.3048;
		}

		//set x
		//for(int i = 0; i < 7; i++){
		//	x.set(i,0,);
		//}
		x.set(0,0,0.015);
		x.set(3,0,-0.1);
		x.set(6,0,0.21);
		
		//set b
		for(int i = 0; i < numCorrespondences; i++){
			b.set(3 * i, 0, new double[][]{	{pixels.get(i)[0]},
							{pixels.get(i)[1]},
							{1}});
		}
		
		
		try{
			for(int i = 0; i < numIterations; i++){
				System.out.println(i + "th time in loop");
				for(int j = 0; j < numCorrespondences; j++){
					A.set(3 * j, 0, getJacobian(x.get(0,0),
								x.get(1,0),
								x.get(2,0),
								x.get(3,0),
								x.get(4,0),
								x.get(5,0),
								x.get(6,0),
								points.get(j)[0],
								points.get(j)[1],
								points.get(j)[2]));
				}
				Matrix At = A.transpose();
				Matrix AtA = At.times(A);
				Matrix AtAinv = AtA.inverse();
				Matrix Atb = At.times(b);
				x = AtAinv.times(Atb);
			}
		}catch(RuntimeException e){
		}
		
		x.print();
		
	}
	
	
	
	
	
	
	
}
