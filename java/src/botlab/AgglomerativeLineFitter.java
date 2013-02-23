package botlab;

import javax.swing.*;
import java.awt.*;

import java.util.*;
import java.io.*;

import april.jmat.*;
import april.vis.*;
import april.util.*;
import april.lcmtypes.*;
import botlab.lcmtypes.*;

import lcm.lcm.*;

public class AgglomerativeLineFitter
{

	double threshold;
	double numSteps;
	double pointSpreadMax;
	double cornerAngleThresh;

	ArrayList<double []> points = null;
	ArrayList<double[]> lines = null;
	ArrayList<ArrayList<double[]> > lineSegs = new ArrayList<ArrayList <double[]> >();
	ArrayList<double []> corners = null;
	double longestSeg[] = new double[2];

	public AgglomerativeLineFitter(double _threshold, double _numSteps, double _pointSpreadMax, double _cornerAngleThresh)
	{
		//threshold = 0.00005;
		//numSteps = 15;
		//pointSpreadMax = 0.9;
		threshold = _threshold;
		numSteps = _numSteps;
		pointSpreadMax = _pointSpreadMax;
		cornerAngleThresh = _cornerAngleThresh;

		points = new ArrayList<double []>();
	}

	public void setPoints(ArrayList<double []> _points){
		points = _points;
	}
	
	public ArrayList<ArrayList<double []> > getLineSegs(){
		return lineSegs;
	}

	public ArrayList<double []> getLines(){
		return lines;
	}

	public double[] getLongestSeg(){
		return longestSeg;
	}

	public void plotLineSegs(VisWorld.Buffer vb){
		for(int i = 0; i < lineSegs.size(); i++){
			vb.addBack(new VisChain(LinAlg.translate(0,0,5),new VzLines(new VisVertexData(lineSegs.get(i)), 
					VzLines.LINES, new VzLines.Style(Color.red, 4))));
		}
	}

	public void plotCorners(VisWorld.Buffer vb){
		vb.addBack(new VisChain(LinAlg.translate(0,0,5),new VzPoints(new VisVertexData(corners),
					new VzPoints.Style(Color.yellow, 5))));
	}
	
	public void findSegs(){
		ArrayList<int[]> segments = new ArrayList<int[]>();
		lineSegs.clear();

		for(int i = 0 ; i < points.size() - 1; i++){
			Matrix a = Matrix.columnMatrix(points.get(i));
			Matrix b = Matrix.columnMatrix(points.get(i + 1));
			Matrix BA = b.minus(a);
			double magBA = BA.getColumn(0).dotProduct(BA.getColumn(0));

			// pointSpreadMax is squared to avoid calulating a sqrt()
			if(magBA < pointSpreadMax * pointSpreadMax){
				segments.add(new int[]{i, i + 1});
			}

		}

		boolean changed = true;
		int numIterations = 0;
		while(changed && (numIterations++ < numSteps)){
			changed = false;
			for(int j = 0; j < segments.size() - 1; j++){
				
				int first = segments.get(j)[0];
				int last = segments.get(j+1)[1];
				if(segments.get(j)[1] == segments.get(j+1)[0]){
					ArrayList<double[]> checkPoints = new ArrayList<double[]>();
					for(int k = first; k <= last; k++){
						checkPoints.add(points.get(k));
					}
				
					MultiGaussian errorCheck = new MultiGaussian(checkPoints);
					Matrix cov = new Matrix(errorCheck.getCovariance());
					cov.timesEquals(checkPoints.size());
					double theta = (0.5 * (Math.atan2(2 * cov.getRow(0).get(1), cov.getRow(0).get(0) - cov.getRow(1).get(1)))) + (Math.PI / 2.0);
				
					double[] normVals = new double[]{Math.cos(theta), Math.sin(theta)};
					Matrix normal = Matrix.columnMatrix(normVals);

					Matrix error = normal.transpose();
					error = error.times(cov);
					error = error.times(normal);
				
					if(error.getRow(0).get(0) < threshold){
						segments.get(j)[1] = last;
						segments.remove(j+1);
						changed = true;
					}
				}
				
			}
			
		}
		
		// accept any segments greater than 3 points in length
		// Make a list of (x,y, theta)
		lines = new ArrayList<double[]>();
		double maxMagHalf = Double.MIN_VALUE;

		for(int j = 0; j < segments.size(); j++){
			if(segments.get(j)[1] - segments.get(j)[0] > 5){

				int first = segments.get(j)[0];
				int last = segments.get(j)[1];
				
				ArrayList<double[]> checkPoints = new ArrayList<double[]>();
				for(int k = first; k <= last; k++){
					checkPoints.add(points.get(k));
				}
				
				MultiGaussian errorCheck = new MultiGaussian(checkPoints);
				Matrix cov = new Matrix(errorCheck.getCovariance());
				double theta = (0.5 * (Math.atan2(2.0 * cov.getRow(0).get(1), cov.getRow(0).get(0) - cov.getRow(1).get(1))));
				
				double[] lineVals = new double[]{Math.cos(theta), Math.sin(theta)};
				Matrix line = Matrix.columnMatrix(lineVals);
				
				Matrix firstPoint = Matrix.columnMatrix(points.get(first));
				Matrix lastPoint = Matrix.columnMatrix(points.get(last));
				
				Matrix magVec = lastPoint.minus(firstPoint);
				double magHalf = magVec.getColumn(0).dotProduct(magVec.getColumn(0));
				magHalf = Math.sqrt(magHalf) / 2.0;

				line.timesEquals(magHalf);
				
				Matrix myu = Matrix.columnMatrix(errorCheck.getMean());
				
				if(magHalf > maxMagHalf){
					maxMagHalf = magHalf;
					longestSeg[0] = myu.get(0,0);
					longestSeg[1] = myu.get(1,0);
				}
				

				ArrayList<double[]> lineSeg = new ArrayList<double[]>();
				lineSeg.add(points.get(first));
				lineSeg.add(points.get(last));
				lineSegs.add(lineSeg);

				// Add line with one point associated to line (offset) and theta (line orientation)
				lines.add(new double[]{myu.get(0,0), myu.get(1,0), theta});

			}
		}

		
		corners = new ArrayList<double []>();
		// Consider the intersections of all valid lines
		for (int i = 0; i < lines.size(); i++) {
			for (int j = i + 1; j < lines.size(); j++) {
				// Decide if the lines form a valid "corner":
				double theta1 = lines.get(i)[2];
				double theta2 = lines.get(j)[2];
				double dotProduct = Math.abs(Math.cos(theta1) * Math.cos(theta2) + Math.sin(theta1) * Math.sin(theta2));

				if (Math.abs(dotProduct) < cornerAngleThresh) {
					// Consider corner in this case
					double[][] A_vals = new double[2][2];
					A_vals[0][0] = -Math.tan(theta1); //Math.cos(theta1);
					A_vals[0][1] = 1; //Math.sin(theta1);
					A_vals[1][0] = -Math.tan(theta2); //Math.cos(theta2);
					A_vals[1][1] = 1; //Math.sin(theta2);
					Matrix A = new Matrix(A_vals);
					
					double[] P_vals = new double[2];
					P_vals[0] = lines.get(i)[1] - Math.tan(theta1)*lines.get(i)[0];
					P_vals[1] = lines.get(j)[1] - Math.tan(theta2)*lines.get(j)[0];
					Matrix P = Matrix.columnMatrix(P_vals);
					
					Matrix A_inv = A.inverse();
					
					Matrix intersection = A_inv.times(P);
					corners.add(new double[] {intersection.get(0,0), intersection.get(1,0)});

				}
			}
		}
	
	
	}

}
