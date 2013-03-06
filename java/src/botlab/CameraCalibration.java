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


	public static double[][] J_Cross(double cx, double cy, double x, double y){
		
		double cw = 1;
		
		double[][] jacobian = new double[][]{
			{     0,     0,   0, -cw*x, -cw*y, -cw,  cy*x,  cy*y,  cy},
			{  cw*x,  cw*y,  cw,     0,     0,   0, -cx*x, -cx*y, -cx},
			{ -cy*x, -cy*y, -cy,  cx*x,  cx*y,  cx,     0,     0,   0}};

		return jacobian;
		
	}


	public static void main(String args[]){
		double Cirx = 3.37025048258259540e+02; // larger -> moves right
		double Ciry = 2.4675008449138741e+02; // larger -> moves down
		int numCorrespondences = 12;
		Matrix A = new Matrix(9,9);
		Matrix J = new Matrix(3 * numCorrespondences,9);
		Matrix h = new Matrix(9,1);
		Matrix H = new Matrix(3,3);
		
		ArrayList<double[]> points = new ArrayList<double[]>();
		ArrayList<double[]> pixels = new ArrayList<double[]>();
		
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
		points.add(new double[]{5,-2});
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
			// not necessary since should be all 0s.
			//points.get(i)[2] *= 0.3048;
			pixels.get(i)[0] -= Cirx;
			pixels.get(i)[1] -= Ciry;
		}
		
		for(int i = 0; i < numCorrespondences; i++){
			J.set(3 * i, 0, J_Cross(points.get(i)[0], points.get(i)[1], pixels.get(i)[0], pixels.get(i)[1]));
		}
		
		A = J.transpose().times(J);

		SingularValueDecomposition svd = new SingularValueDecomposition(A);
		System.out.println(svd.getV().getRowDimension());
		System.out.println(svd.getV().getColumnDimension());
		double[][] htemp = svd.getV().copyArray(0, 8, 9, 1);
		H = new Matrix(new double[][]{
			{htemp[0][0], htemp[1][0], htemp[2][0]},
			{htemp[3][0], htemp[4][0], htemp[5][0]},
			{htemp[6][0], htemp[7][0], htemp[8][0]}});
		
		
		H.print();
		for(int i = 0; i < numCorrespondences; i++){
			Matrix test = Matrix.columnMatrix(new double[]{pixels.get(i)[0], pixels.get(i)[1], 1});
			test = H.times(test);
			Matrix test1 = Matrix.columnMatrix(new double[]{test.get(0,0) / test.get(2,0), test.get(1,0) / test.get(2,0)});
			Matrix point = Matrix.columnMatrix(new double[]{points.get(i)[0], points.get(i)[1]});
			//test1.print();
			//point.print();
			point.minusEquals(test1);
			//point.print();
			System.out.println(Math.sqrt(point.transpose().times(point).get(0,0)));
			//System.out.println();
			//System.out.println();
			//System.out.println();
			//System.exit(1);
		}
		
	}
	
	
	
	
	
	
	
}
