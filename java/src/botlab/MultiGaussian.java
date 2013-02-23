package botlab;

import java.util.*;
import java.awt.*;
import javax.swing.*;

import april.jmat.*;
import april.vis.*;
import april.util.*;

/** Represents a multi-variate Gaussian distribution. This is a
 *  template for EECS568; you should fill in the methods below.
 **/
public class MultiGaussian
{
    /** The mean **/
    double u[];

    /** The covariance **/
    double P[][];

    /** Create a new MultiGaussian object with covariance P and mean u.
        (This should be trivial)
    **/
    public MultiGaussian(double P[][], double u[])
    {
        // You shouldn't need to modify this.
        this.P = LinAlg.copy(P);
        this.u = LinAlg.copy(u);
    }

    /** Create a new MultiGaussian object that estimates the covariance and mean
        from the provided samples. This should implement your algorithm from A. **/
    public MultiGaussian(ArrayList<double[]> samples)
    {
    	// Compute mean
	Matrix mean = new Matrix(samples.get(0).length, 1);
    	for(double[] vec: samples){
    		mean.plusEquals(Matrix.columnMatrix(vec));
    	}
    	
    	// Divide mean by n
    	mean.timesEquals(1.0/((double)samples.size()));
    	
    	// Compute Covariance
	Matrix covariance = new Matrix(samples.get(0).length, samples.get(0).length);
    	for(double[] vec: samples){
    		Matrix value = Matrix.columnMatrix(vec);
    		value.minusEquals(mean);
    		
    		double[] A = value.getColumn(0).getDoubles();
    		covariance.plusEquals(Matrix.outerProduct(A, A));
    	}
    	
    	// Divide Covariance by n-1
    	double num_samples = Math.max(samples.size()-1, 1);
    	covariance.timesEquals(1.0/num_samples);
    	
    	u = mean.getColumn(0).getDoubles();
    	P = covariance.copyArray();
    }
    
    /** Return the covariance associated with this object. (Trivial). **/
    public double[][] getCovariance()
    {
        // You shouldn't need to modify this.
        return P;
    }

    /** Return the mean associated with this object. (Trivial). **/
    public double[] getMean()
    {
        // You shouldn't need to modify this.
        return u;
    }

    /** Draw a random sample from this distribution. This should implement your
        method from part C.
    **/
    public double[] sample(Random r)
    {
    	// Populate vector of original gaussian distribution
    	double[] original = new double[u.length];
    	for(int i=0; i<u.length; ++i){
    		original[i] = r.nextGaussian();
    	}
    	
    	// Compute the cholesky decomposition of our covariance
    	CholeskyDecomposition chol = new CholeskyDecomposition(new Matrix(P));
    	Matrix A = chol.getL();
    	
    	// Multiply the solution for A by the original distribution
    	Matrix result = A.times(Matrix.columnMatrix(original));
    	
    	// Add our mean to the result
    	result.plusEquals(Matrix.columnMatrix(u));
    	
    	// Return the result (which is a (d x 1) matrix)
    	return result.getColumn(0).getDoubles();
    }

    /** Given an observation from the distribution, compute the chi^2 value. This
        is given by (x-u)'inv(M)(x-u)
    **/
    public double chi2(double[] x)
    {
    	// Calculate (x-u)
        Matrix diff = Matrix.columnMatrix(x);
        diff.minusEquals(Matrix.columnMatrix(u));
        
        // calculate (x-u)'inv(M)(x-u)
        Matrix result = diff.transpose();
        result = result.times((new Matrix(P)).inverse());
        result = result.times(diff);
    	
        // Return the only index of the matrix
        return result.getRow(0).get(0);
    }

    /** Compute a set of points that, when plotted as a curve, would trace out an
        iso-probability contour corresponding to the specified chi^2 value. Generate
        points at one-degree spacings using your method from part D.
    **/
    public ArrayList<double[]> getContour(double chi2)
    {
    	ArrayList<double[]> result = new ArrayList<double[]>();
    	
    	for(double angle=0.0; angle<360.0; angle+=1.0){
		double angleRad = Math.toRadians(angle);
    		double[] normal_array = new double[]{ Math.cos(angleRad), Math.sin(angleRad) };
    		Matrix normal = Matrix.columnMatrix(normal_array);
    		
    		// Multiply normal' Inv(covariance) normal
    		Matrix value = normal.transpose();
    		value = value.times((new Matrix(P).copy(0,1,0,1)).inverse());
    		value = value.times(normal);
    		
    		// Alpha = srt(chi2 / value)
    		double alpha = Math.sqrt(chi2 / value.get(0,0));
    		
    		// Point = mean + alpha * normal
    		Matrix temp = Matrix.columnMatrix(u);
		Matrix point = temp.copy(0,1,0,0);
		point.plusEquals(normal.times(alpha));
    		double[] finalPoint = new double[2];
		finalPoint[0] = point.get(0,0);
		finalPoint[1] = point.get(1,0);
    		
    		// Add resultant point to our list
    		result.add(finalPoint);
    	}
    	
    	return result;
    }



    static public void renderGaussianProjection(VisWorld vw, double r, double theta){
	VisWorld.Buffer vb = vw.getBuffer("Gaussian Projection");
	
	double[][] wrCov = new double[1][1];
	wrCov[0][0] = 100;
	double[] noiseMyu = new double[1];
	noiseMyu[0] = 0;
	double[][] wthetaCov = new double[1][1];
	wthetaCov[0][0] = .5;

	MultiGaussian wr = new MultiGaussian(wrCov, noiseMyu);
	MultiGaussian wtheta = new MultiGaussian(wthetaCov, noiseMyu);

	double[] zVals = new double[2];
	zVals[0] = r;		// r
	zVals[1] = theta;	// theta


	Matrix z = Matrix.columnMatrix(zVals);


	double[] noiseVals = new double[2];
	Random rand = new Random();

	ArrayList<double[]> EuclideanPoints = new ArrayList<double[]>();

	for(int i = 0; i < 500; i++){
		Matrix x = z.copy();

		noiseVals[0] = wr.sample(rand)[0];
		noiseVals[1] = wtheta.sample(rand)[0];
		x.plusEqualsColumnVector(noiseVals);
		
		double[] euclid = new double[]{x.getRow(0).get(0) * Math.cos(x.getRow(1).get(0)), x.getRow(0).get(0) * Math.sin(x.getRow(1).get(0))};
		
		
		EuclideanPoints.add(euclid);		
	}
	MultiGaussian X = new MultiGaussian(EuclideanPoints);
	vb.addBack(new VzAxes());
	vb.addBack(new VzGrid());
	vb.addBack(new VzPoints(new VisVertexData(EuclideanPoints),
                            new VzPoints.Style(Color.orange, 4)));

	vb.addBack(new VzLines(new VisVertexData(X.getContour(3)),
                                            VzLines.LINE_LOOP,
                                            new VzLines.Style(Color.pink, 2)));
	vb.swap();
    }




    // XXX Example rendering code. You will not need to use or modify
    // this code. It merely exists to present example usage of Vis.
    //
    // This function manually creates an ellipse based on specfications for
    // the minor and major axis as well as an angle of rotation around the
    // origin, which happens to be the center of the ellipse.
    static public void renderEllipse(VisWorld vw, double myux, double myuy, double covx, double covy, double chi2)
    {
        // Render an ellipse with the given parameters
        VisWorld.Buffer vb = vw.getBuffer("laser-data");



	double[][] Cov = new double[2][2];
	Cov[0][0] = covx;
	Cov[0][1] = 0;
	Cov[1][0] = 0;
	Cov[1][1] = covy;
	double[] myu = new double[2];
	myu[0] = myux;
	myu[1] = myuy;
	MultiGaussian MG = new MultiGaussian(Cov, myu);
	
	Random r = new Random();
	ArrayList<double[]> samplesOut = new ArrayList<double[]>();
	ArrayList<double[]> samplesIn = new ArrayList<double[]>();

	for(int i = 0; i < 500; i++){
		double[] point = MG.sample(r);
		
		if(MG.chi2(point) < chi2){
			samplesIn.add(point);
		}
		else{
			samplesOut.add(point);
		}
	}

	vb.addBack(new VzAxes());
	vb.addBack(new VzGrid());
	vb.addBack(new VzLines(new VisVertexData(MG.getContour(chi2)),
                                            VzLines.LINE_LOOP,
                                            new VzLines.Style(Color.green, 2)));

	vb.addBack(new VzPoints(new VisVertexData(samplesIn),
                            new VzPoints.Style(Color.blue, 4)));
	vb.addBack(new VzPoints(new VisVertexData(samplesOut),
                            new VzPoints.Style(Color.red, 4)));

	double percent = (double)samplesIn.size() / (double)(samplesIn.size() + samplesOut.size()) * 100.0;
	vb.addBack(new VzText("<<scale=0.05>>Percent of points inside chi2: " + percent + "%"));
	
        vb.swap();
    }

    public static void main(String args[])
    {
        JFrame jf = new JFrame("MultiGaussian");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setSize(800,600);
        jf.setLayout(new BorderLayout());

        final VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);
        jf.add(vc, BorderLayout.CENTER);

        // XXX Example interactive vis rendering code. Comment this out
        // or delete it before making your final submission
        ParameterGUI pg = new ParameterGUI();
	pg.addDoubleSlider("r", "R", -50.0, 50.0, 0.0);
	pg.addDoubleSlider("theta", "Theta", -6.28318, 6.28318, 0.0);
	pg.addDoubleSlider("myux", "Myu (X)", -10.0, 10.0, 0.0);
	pg.addDoubleSlider("myuy", "Myu (Y)", -10.0, 10.0, 0.0);
	pg.addDoubleSlider("covx", "Covariance (X)", 0.0, 10.0, 1.0);
	pg.addDoubleSlider("covy", "Covariance (Y)", 0.0, 10.0, 1.0);
	pg.addDoubleSlider("chi2", "Chi Squared", 0.0, 10.0, 1.0);
        pg.addListener(new ParameterListener() {
            public void parameterChanged(ParameterGUI pg, String name) {
                renderEllipse(vw, pg.gd("myux"), pg.gd("myuy"), pg.gd("covx"), pg.gd("covy"), pg.gd("chi2"));
		//renderGaussianProjection(vw, pg.gd("r"), pg.gd("theta"));
            }
        });
        jf.add(pg, BorderLayout.SOUTH);
        renderEllipse(vw, pg.gd("myux"), pg.gd("myuy"), pg.gd("covx"), pg.gd("covy"), pg.gd("chi2"));
	//renderGaussianProjection(vw, pg.gd("r"), pg.gd("theta"));

        // Insert your test code here.



        jf.setVisible(true);
    }
}
