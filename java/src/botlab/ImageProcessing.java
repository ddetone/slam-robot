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

import botlab.lcmtypes.*;
import lcm.lcm.*;


public class ImageProcessing extends VisEventAdapter
{

	static final double DEFAULT_CALIBRATE_VAL = -2550;
	static final double DEFAULT_THRESH_VAL = 30;
	static final int DEFAULT_NUM_STEPS = 200;
	static final int DEFAULT_POINT_SPREAD = 26;
	static final double DEFAULT_CORNER_ANG_THRESH = 0.6;
	static final double DEFAULT_BLUE_THRESH = 285;
	static final double DEFAULT_HALF_BOX_THRESH = 130;
	static final double DEFAULT_GREEN_THRESH = 355;
	static final double DEFAULT_SAT_THRESH = 0.52;
	static final double DEFAULT_VALUE_THRESH = 0.62;

	static final boolean DEFAULT_DISP_LONG_LINE = false;
	static final boolean DEFAULT_DISP_BLUE_PIXELS = false;
	static final boolean DEFAULT_DISP_LINES = false;
	static final boolean DEFAULT_DISP_CORNERS = false;
	static final boolean DEFAULT_DISP_TRIANGLES = false;
	static final boolean DEFAULT_TRIANGLE_COST_MAP = false;

	double calibrateVal = DEFAULT_CALIBRATE_VAL;
	double threshold = DEFAULT_THRESH_VAL;
	int numSteps = DEFAULT_NUM_STEPS;
	int pointSpreadMax = DEFAULT_POINT_SPREAD;
	double cornerAngleThresh = DEFAULT_CORNER_ANG_THRESH;
	double blueThresh = DEFAULT_BLUE_THRESH;
	double halfBoxThresh = DEFAULT_HALF_BOX_THRESH;
	double greenThresh = DEFAULT_GREEN_THRESH;
	double saturationThresh = DEFAULT_SAT_THRESH;
	double valueThresh = DEFAULT_VALUE_THRESH;

	boolean dispLongLine  = DEFAULT_DISP_LONG_LINE;
	boolean dispBluePixels = DEFAULT_DISP_BLUE_PIXELS;
	boolean dispLines = DEFAULT_DISP_LINES;
	boolean dispCorners = DEFAULT_DISP_CORNERS;
	boolean dispTriangles = DEFAULT_DISP_TRIANGLES;
	boolean dispTriMap = DEFAULT_TRIANGLE_COST_MAP;

	public enum BLUE_STATE {PREBLUE, INBLUE, POSTBLUE};
	final int BLUE=230, GREEN=60, YELLOW=60;

	long timeOfImage = 0;

	LCM lcm;

	ImageSource is;

	JFrame jf; 
	JImage jim;

	ParameterGUI pg;

	int width = 0;
	int height = 0;
	
	final int searchHeight = 500;
	//WRONG NUMBER BELOW
	final double TRIANGLE_HEIGHT = 0.3;

	float pixelDists[] = null;
	float pixelThetas[] = null;
	int indexLookup[] = null;

	float hsvImage[][] = null;
	float checkHSV[] = new float[3];;
	int mouse[] = new int[2];

	int data[];

	bot_status_t lastPose;

        static VisWorld vw;
        static VisLayer vl;
        static VisCanvas vc;
	static VisWorld.Buffer vb;

	static AgglomerativeLineFitter alf = null;

	// homography matrix
	Matrix H = null;

	boolean dispGUI;

	public ImageProcessing(ImageSource _is, boolean _dispGUI)
	{
		is = _is;

		lcm = LCM.getSingleton();
		//lcm.subscribe("6_POSE", this);
		
		dispGUI = _dispGUI;

		if(dispGUI){
			vw = new VisWorld();
			vl  = new VisLayer(vw);
			vc = new VisCanvas(vl);
			pg  = new ParameterGUI();
			// Determine which slider values we want
			pg.addDoubleSlider("calib", "Calibrate Constant", -5000, -1, DEFAULT_CALIBRATE_VAL);
			pg.addDoubleSlider("greenThresh", "Green Hue Mean", 0, 360, DEFAULT_GREEN_THRESH);
			pg.addDoubleSlider("satThresh", "Saturation Mean", 0, 1, DEFAULT_SAT_THRESH);
			pg.addDoubleSlider("valThresh", "Value Mean", 0, 1, DEFAULT_VALUE_THRESH);
			pg.addDoubleSlider("halfBoxThresh", "Half Box Thresh", 0, 500, DEFAULT_HALF_BOX_THRESH);

			pg.addDoubleSlider("blueThresh", "Blue Thresh", 0, 360, DEFAULT_BLUE_THRESH);
			pg.addDoubleSlider("thresh", "Line Error Threshold", 0, 200,DEFAULT_THRESH_VAL);
			pg.addIntSlider("num_steps", "Line Fit Num Steps", 0,200,DEFAULT_NUM_STEPS);
			pg.addIntSlider("point_spread", "Line Point Spread Max", 0,100,DEFAULT_POINT_SPREAD);
			pg.addDoubleSlider("cornerAngleThresh", "Line Intersection (Corner) Angle Threshold", 0,1,DEFAULT_CORNER_ANG_THRESH);

			pg.addCheckBoxes("dispLongLine", "Show Longest Line Mean", DEFAULT_DISP_LONG_LINE);
			pg.addCheckBoxes("dispBluePixels", "Show Pixels", DEFAULT_DISP_BLUE_PIXELS);
			pg.addCheckBoxes("dispLines", "Show Lines", DEFAULT_DISP_LINES);
			pg.addCheckBoxes("dispCorners", "Show Corners", DEFAULT_DISP_CORNERS);
			pg.addCheckBoxes("dispTriangles", "Show Triangles", DEFAULT_DISP_TRIANGLES);
			pg.addCheckBoxes("dispTriMap", "Show Triangle Cost Map", DEFAULT_TRIANGLE_COST_MAP);
			
			
			pg.addListener(new ParameterListener() {
				public void parameterChanged(ParameterGUI pg, String name)
				{
					if(name == "valThresh"){
						valueThresh = pg.gd("valThresh");
					}
					if(name == "satThresh"){
						saturationThresh = pg.gd("satThresh");
					}
					if(name == "calib"){
						calibrateVal = pg.gd("calib");
						calculateLookupTable();
					}
					else if(name == "blueThresh")blueThresh = pg.gd("blueThresh");
					else if(name == "thresh"){
						threshold = pg.gd("thresh");
						alf.threshold = threshold;
					}
					else if(name == "num_steps"){
						numSteps = pg.gi("num_steps"); 
						alf.numSteps = numSteps;
					}
					else if(name == "point_spread"){
						pointSpreadMax = pg.gi("point_spread"); 
						alf.pointSpreadMax = pointSpreadMax;
					}
					else if(name == "cornerAngleThresh"){
						cornerAngleThresh = pg.gd("cornerAngleThresh");
						alf.cornerAngleThresh = cornerAngleThresh;
					}
					else if(name == "dispLongLine"){
						dispLongLine = pg.gb("dispLongLine");
					}
					else if(name == "dispBluePixels"){
						dispBluePixels = pg.gb("dispBluePixels");
					}
					else if(name == "dispLines"){
						dispLines = pg.gb("dispLines");
					}
					else if(name == "dispCorners"){
						dispCorners = pg.gb("dispCorners");
					}
					else if(name == "dispTriangles"){
						dispTriangles = pg.gb("dispTriangles");
					}
					else if(name == "greenThresh"){
						greenThresh = pg.gd("greenThresh");
					}
					else if(name == "halfBoxThresh"){
						halfBoxThresh = pg.gd("halfBoxThresh");
					}
					else if(name == "dispTriMap"){
						dispTriMap = pg.gb("dispTriMap");
					}
			}});
			
			jf = new JFrame("Image Processing");
			jim = new JImage();
			jim.setFit(true);
			vl.addEventHandler(this);
			vl.cameraManager.uiLookAt(new double[]{640.0, 480.0, 1000.0}, new double[]{640.0,480.0,0.0}, new double[]{0.0,1.0,0.0}, true);

			// Setup window layout
			jf.setLayout(new BorderLayout());
			jf.add(vc, BorderLayout.CENTER);
			vb = vw.getBuffer("ImageProcessing");
			jf.add(pg, BorderLayout.SOUTH);
			jf.setSize(1024, 768);
			//jf.setVisible(true);
			jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		}
	}

/*
	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_POSE"))
			{
				if(done){
					lastPose = new bot_status_t(dins);
				}
			}
		}catch(IOException e){
			e.printStackTrace();
		}
	}
*/

	public boolean mouseMoved(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
	{
		double[] _pixel = ray.intersectPlaneXY();
		int[] pixel = new int[]{(int)_pixel[0], (int)_pixel[1]};
		pixel[1] = height - pixel[1];
		mouse = pixel;
		if((pixel[1] >= height - searchHeight) && (pixel[1] > 0) && (pixel[1] < height) && (pixel[0] > 0) && (pixel[0] < width)) checkHSV = hsvImage[(pixel[1] - height + searchHeight) * width + pixel[0]];
		else checkHSV = new float[]{pixel[1],0F,0F};
		return false;
	}



	public void computeHomography(int numCorrespondences, ArrayList<double[]> realWorld, ArrayList<double[]> pixels){
		
		// Compute H
		Matrix j = new Matrix(3 * numCorrespondences, 9);
	
		for(int i = 0; i < numCorrespondences; i++){
			j.set(0 + 3 * i, 0, J_Cross( realWorld.get(i)[0], realWorld.get(i)[1], pixels.get(i)[0], pixels.get(i)[1]));
		}

		SingularValueDecomposition svd = new SingularValueDecomposition(j);
		System.out.println(svd.getV().getRowDimension());
		System.out.println(svd.getV().getColumnDimension());
		double[][] htemp = svd.getV().copyArray(0, 8, 9, 1);
		H = new Matrix(new double[][]{
			{htemp[0][0], htemp[1][0], htemp[2][0]},
			{htemp[3][0], htemp[4][0], htemp[5][0]},
			{htemp[6][0], htemp[7][0], htemp[8][0]}});
		
	}


	public static double[][] J_Cross(double x, double y, double cx, double cy){
		
		double cw = 1;
		
		double[][] jacobian = new double[][]{
			{     0,     0,   0, -cw*x, -cw*y, -cw,  cy*x,  cy*y,  cy},
			{  cw*x,  cw*y,  cw,     0,     0,   0, -cx*x, -cx*y, -cx},
			{ -cy*x, -cy*y, -cy,  cx*x,  cx*y,  cx,     0,     0,   0}};

		return jacobian;
		
	}
//*
	// x_start, y_start, x_end, y_end
	public void drawBox(int[] bounds, int color){
		// Display the detection, by drawing on the image
		if (bounds != null) {
			// draw the horizontal lines
			for (int y : new int[]{bounds[1], bounds[3]})
				for (int x = bounds[0]; x <=bounds[2]; x++) {
					data[y * width + x] = color; //Go Blue!
				}

			// draw the horizontal lines
			for (int x : new int[]{bounds[0], bounds[2]})
				for (int y = bounds[1]; y <=bounds[3]; y++) {
					data[y * width + x] = color; //Go Blue!
				}

		}
	}
//*/	

   
	public static float dist2d(float x1, float x2, float y1, float y2)
	{
		double X = x2-x1;
		double Y = y2-y1;
		return (float)Math.sqrt(X*X + Y*Y);
	}

	public int convertPix(int index)
	{	
		int val;
		
		//calculate x & y pos
		int x = index%width;
		int y = (index - x)/width;
		
		//x = 750;
		//y = 550;
		
		double B = 1.0 / calibrateVal;
		//double B = 1.0 / pg.gd("calib");
		double r = (double)pixelDists[y * width + x];
		
		double th_out = (double)pixelThetas[y * width + x];
		//double B = pg.gi("calib");	
		
		
		//need to shift x&y so that middle pixel is (0,0) 
		//x = x - width / 2;
		//y = y - height / 2;
		
		//System.out.printf("x:%d, y:%d, TH:%f, B:%f\n",x,y,th_out,B);
		//System.out.printf("before r:%f\n",r);
		r = r + B*r*r;
		//System.out.printf("after r:%f\n",r);
		
		
		int nx = (int)((r * Math.cos(th_out)) + (width / 2));
		int ny = (int)((r * Math.sin(th_out)) + (height / 2));
				
		//System.out.printf("nx:%f, ny:%f\n\n",nx,ny);
		
		
		//nx = nx + (int)(width/2);
		//ny = ny + (int)(height/2);
		
		//System.out.printf("nx:%f, ny:%f\n\n",nx,ny);

		if (nx < 0 || nx >= width || ny < 0 || ny >= height) val = -1;
		else val = ny * width + nx;
		
		return val;
	
	}
	
	
	public void calibrate()
	{
	
		int[] temp = new int[data.length];
		//Arrays.fill(temp, 0);

		int index;
		
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				index = indexLookup[i * width + j];
				//System.out.println("pixel: " + j + ", " + i + "  maps to pixel  " + index%width + ", "
				if(index != -1){
					if(temp[i * width + j] == 0) temp[i * width + j] = data[i * width + j];
					if(temp[index] == 0) temp[index] = data[index];
					data[i * width + j] = temp[index];
				}
			}	
		}
		
		return;  	
	
	}

	public void calculatePixelsState(){

		for(int i = 0; i < height; i++){
			for(int j = 0; j < width; j++){
				pixelDists[i * width + j] = dist2d(width/2, j, height/2, i);
				pixelThetas[i * width + j] = (float)Math.atan2(i - height/2, j - width/2);
			}
		}
	}

	public void calculateLookupTable(){
		for(int i = 0; i < height; i++){
			for(int j = 0; j < width; j++){
				indexLookup[i * width + j] = convertPix(i * width + j);
			}
		}
	}
	
		
	
	
	public void publish(Triangles triangles, ArrayList<ArrayList<double[]> > lineSegs){
		map_features_t features = new map_features_t();
		double f = 672.07;
		double cx = 620.82;
		double cy = 456.58;
		double h = 0.68898;
		features.nlineSegs = lineSegs.size();
		features.lineSegs = new double[features.nlineSegs][4];
		for(int i = 0; i < features.nlineSegs; i++){
			ArrayList<double[]> line = lineSegs.get(i);
			double[] lineStart = line.get(0);
			double[] lineEnd = line.get(1);
			features.lineSegs[i][0] = (lineStart[0] - cx) / (lineStart[1] - cy) * h * 0.3048;
			features.lineSegs[i][1] = f * h / (lineStart[1] - cy) * 0.3048;
			features.lineSegs[i][2] = (lineEnd[0] - cx) / (lineEnd[1] - cy) * h * 0.3048;
			features.lineSegs[i][3] = f * h / (lineEnd[1] - cy) * 0.3048;
		}

		
		features.ntriangles = triangles.numTriangles;
		features.triangles = new double[features.ntriangles][2];
		for(int i = 0; i < features.ntriangles; i++){
			double[] mean = triangles.getMean(i);
			features.triangles[i][0] = (mean[0] - cx) / (mean[1] - cy) * h * 0.3048;
			features.triangles[i][1] = f * h / (mean[1] - cy) * 0.3048;
		}
		if(lastPose == null)lastPose = new bot_status_t();
		//features.bot=lastPose;
		features.utime = timeOfImage;
		lcm.publish("6_FEATURES", features);
	}


	public class Triangles{
		ArrayList<double[]> means;
		ArrayList<int[]> boundingBoxes;
		
		int numTriangles = 0;

		Triangles(){
			boundingBoxes = new ArrayList<int[]>();;
			means = new ArrayList<double[]>();
		}

		public double[] getMean(int i){
			return means.get(i);
		}

		public void addTriangle(double[] _mean, int[] _bounds){
			numTriangles++;
			double tempMean[] = new double[2];
			int tempBoundingBox[] = new int[4];
			System.arraycopy( _mean, 0, tempMean, 0, _mean.length );
			means.add(tempMean);
			System.arraycopy( _bounds, 0, tempBoundingBox, 0, _bounds.length );
			boundingBoxes.add(tempBoundingBox);
		}
	}


	public boolean within(double val, double mean, double tolerance){
		if((val > mean - tolerance) && (val < mean + tolerance))return true;
		else return false;
	}



	public Triangles findTriangles(){
		
		UnionFind uf = new UnionFind(width * searchHeight);
		byte costMap[] = new byte[width * searchHeight];
		//pre compute first row of values for optimization of union find
		for(int i = 0; i < width; i++){
			float[] pixel = hsvImage[i];
			if(within(colorScore(i,height - searchHeight, GREEN), greenThresh, 10) && within(pixel[1], saturationThresh, .17) && within(pixel[2], valueThresh, .4))
				costMap[i] = 1;
			else costMap[i] = 0;
		}
		
		for(int j = height - searchHeight; j < height; j++){
			int tempj = j - height + searchHeight;
			for(int i = 0; i < width; i++){
				if(j != height - 1){
					float[] pixel = hsvImage[tempj * width + i];
					if(within(colorScore(i,j,GREEN), greenThresh, 10) && within(pixel[1], saturationThresh, .17) && within(pixel[2], valueThresh, .4))
						costMap[(tempj + 1) * width + i] = 1;
					else costMap[(tempj + 1) * width + i] = 0;
				}
				
				if(costMap[(tempj * width) + i] == 1){
					if(j != height - 1){
						if(costMap[(tempj + 1) * width + i] == 1){
							uf.connectNodes((tempj + 1) * width + i,
									tempj * width + i);
						}

					}
					if(i != width - 1){
						if(costMap[(tempj * width) + i + 1] == 1){
							uf.connectNodes((tempj * width) + i + 1,
									tempj * width + i);
						}
					}
				}
			}
		}
		
		
		HashMap<Integer, Cluster> hm = new HashMap(width*height);
		ArrayList<Integer> reps = new ArrayList<Integer>();
		// Go back through and reorganize clusters into a hash table.
		// Create a list of all valid, unique ids in the hash table
		for(int j = height - searchHeight; j < height; j++){
			int tempj = j - height + searchHeight;
			for(int i = 0; i < width; i++){
				if(costMap[tempj * width + i] == 1){
					int rep = uf.getRepresentative(tempj * width + i);
					if(!reps.contains(rep)) reps.add(new Integer(rep));

					Cluster cluster = hm.get(rep);

					if(cluster == null){
						cluster = new Cluster();
						cluster.addPoint(new int[]{i,j});
						hm.put(rep, cluster);
					}else{
						cluster.addPoint(new int[]{i, j});	
					}
				}
			}
		}



		// Characterize each cluster by its covariance and myu
		int numReps = reps.size();
		Triangles triangles = new Triangles();

		for(int j = 0; j < numReps; j++){
			Cluster cluster = hm.get(reps.get(j));
			//Matrix P = new Matrix(mg.getCovariance());
			//double[] eigens = eig(P.copyArray());
			
			//if(isCircular(eigens)
			//	 && (eigens[1] > minSizeThreshold) && (eigens[0] < maxSizeThreshold)
			//	&& (points.size() > 60)){
			if((Math.abs(cluster.areaBox() / 2.0 - cluster.points.size()) < halfBoxThresh) && 
				(cluster.points.size() > 60) && (cluster.aspectRatio() > .6)){
			//if(cluster.points.size() > 50){
					
				triangles.addTriangle(cluster.getMean(), 
						cluster.boundingBox());
				if(dispTriangles)drawBox(cluster.boundingBox(), 0xffff0000);
					
				
			}
		}
		
		if(dispTriMap){
			for(int j = height - searchHeight; j < height; j++){
				for(int i = 0; i < width; i++){
					double cost = costMap[(j - height + searchHeight) * width + i];
					if(cost == 1) data[j * width + i] = 0xff0000ff;
					else  data[j * width + i] = 0xff000000;
				}
			}
		}
		
		return triangles;
		
	}
	
	public class Cluster{

		ArrayList<int []> points;

		int minX = Integer.MAX_VALUE;
		int maxX = Integer.MIN_VALUE;
		int minY = Integer.MAX_VALUE;
		int maxY = Integer.MIN_VALUE;

		int numPoints = 0;
		long sumPoints[];

		Cluster(){
			points = new ArrayList<int []>();
			sumPoints = new long[2];
		}
		
		public void addPoint(int[] point){
			points.add(point);
			numPoints++;
			sumPoints[0] += (long)point[0];
			sumPoints[1] += (long)point[1];
			if(point[0] < minX) minX=point[0];
			if(point[1] < minY) minY=point[1];
			if(point[0] > maxX) maxX=point[0];
			if(point[1] > maxY) maxY=point[1];	
			return;
		}

		public int[] boundingBox(){
			return new int[]{minX, minY, maxX, maxY};
		}

		public int areaBox(){
			return (maxX - minX) * (maxY - minY);
		}

		public double aspectRatio(){
			return (double)Math.min(maxY - minY, maxX - minX) / (double)Math.max(maxY - minY, maxX - minX);
		}

		public ArrayList<int []> getPoints(){
			return new ArrayList<int[]>(points);
		}

		public double[] getMean(){
			return new double[]{(double)sumPoints[0] / (double)numPoints, (double)sumPoints[1] / (double)numPoints};
		}

		
	}


	public ArrayList<double []> findTape(){

		ArrayList<double []> maxes = new ArrayList<double []>();
		
		int numPostBlue = 0;

		//int maxBlue[] = new int[width];
		BLUE_STATE state[] = new BLUE_STATE[width];

		Arrays.fill(state, BLUE_STATE.PREBLUE);

		for(int j = height - 1; ((j >= height - searchHeight) && (numPostBlue < width)); j--){
			for(int i = 0; ((i < width) && (numPostBlue < width)); i += 9){
				if(state[i] != BLUE_STATE.POSTBLUE){
					if(colorScore(i, j, BLUE) > blueThresh){
					//if((data[j * width + i] & 0x000000ff) > blueThresh){
						//maxBlue[i] = j;
						state[i] = BLUE_STATE.POSTBLUE;
						numPostBlue += 1;
						maxes.add(new double[]{(double)i, (double)j});
					}else if(state[i] == BLUE_STATE.INBLUE){
						state[i] = BLUE_STATE.POSTBLUE;
						numPostBlue += 1;
						maxes.add(new double[]{(double)i, (double)j});
					}
				}
			}
		}

		maxes.add(new double[]{0,height - 0});
		return maxes;
		
	}

	public void convertToHSV(){
		for(int j = height - searchHeight; j < height; j++){
			int tempj = j - height + searchHeight;
			for(int i = 0; i < width; i++){
				//System.out.println((height - j) * width + i);
				int aRGB = data[j * width + i];
				Color.RGBtoHSB(((aRGB & 0x00FF0000) >> 16), ((aRGB & 0x0000FF00) >> 8), (aRGB & 0x000000FF), hsvImage[tempj * width + i]);
				float temp = hsvImage[tempj * width + i][0];
				hsvImage[tempj * width + i][0] = (temp - (int)temp) * 360;
			}
		}
	}


	public double colorScore(int i, int j, int color){
		
		//System.out.println((height - (j + 1)) * width + i);
		float[] pixel = hsvImage[(j - height + searchHeight) * width + i];
		int distFromColor = Math.max(color, (int)pixel[0]) - Math.min(color, (int)pixel[0]);
		return(double)(360 - distFromColor);
		
	}
	
	public void run()
	{
/*
		ArrayList<double[]> realWorld = new ArrayList<double[]>();
		realWorld.add(new double[]{1,.01});
		realWorld.add(new double[]{2,.01});
		realWorld.add(new double[]{3,.01});
		realWorld.add(new double[]{4,.01});
		realWorld.add(new double[]{5,.01});
		realWorld.add(new double[]{6,.01});
		realWorld.add(new double[]{7,.01});
		realWorld.add(new double[]{8,.01});
		realWorld.add(new double[]{9,.01});
		ArrayList<double[]> pixels = new ArrayList<double[]>();
		pixels.add(new double[]{1,185});
		pixels.add(new double[]{1,324});
		pixels.add(new double[]{1,378});
		pixels.add(new double[]{1,407});
		pixels.add(new double[]{1,425});
		pixels.add(new double[]{1,438});
		pixels.add(new double[]{1,447});
		pixels.add(new double[]{1,454});
		pixels.add(new double[]{1,459});
		
		computeHomography(9,pixels,realWorld);
		Matrix newPoint = Matrix.columnMatrix(new double[]{0,407,1});
		//H.print();
		//H.times(newPoint).print();
		//System.exit(1);
*/		
		is.start();
		ImageSourceFormat fmt = is.getCurrentFormat();

		// Initialize visualization environment now that we know the image dimensions
		width = fmt.width;
		height = fmt.height;
		hsvImage = new float[width * searchHeight][3];
		pixelDists = new float[height * width];
		pixelThetas = new float[height * width];
		indexLookup = new int[height * width];

		calculatePixelsState();
		calculateLookupTable();

		alf = new AgglomerativeLineFitter(threshold, numSteps, pointSpreadMax, cornerAngleThresh);

		while(true) {
			// read a frame
			byte buf[] = is.getFrame().data;
			if (buf == null)
				continue;

			timeOfImage = TimeUtil.utime();
			// Grab the image, and convert it to gray scale immediately
			BufferedImage im = ImageConvert.convertToImage(fmt.format, fmt.width, fmt.height, buf);
			
			data = ((DataBufferInt) (im.getRaster().getDataBuffer())).getData();
			
			calibrate();
			convertToHSV();

			ArrayList<double []> topBluePixels = findTape();
			Triangles triangles = findTriangles();
			

			Collections.sort(topBluePixels,new PointComparator());
			alf.setPoints(topBluePixels);
			alf.findSegs();

			VisChain chain = new VisChain(LinAlg.rotateX(Math.PI),LinAlg.translate(0,-height,-5));

			if(dispGUI && (dispTriangles)) chain.add(new VzPoints(new VisVertexData(triangles.means),new VzPoints.Style(Color.blue, 5)));
			if(dispGUI && (dispBluePixels)) chain.add(new VzPoints(new VisVertexData(topBluePixels),new VzPoints.Style(Color.green, 5)));
			if(dispGUI && (dispLines)) alf.plotLineSegs(vb, chain);
			if(dispGUI && (dispCorners)) alf.plotCorners(vb, chain);
			if(dispGUI)vb.addBack(new VzImage(im, VzImage.FLIP));
			if(dispGUI)vb.addBack(chain);
			if(dispGUI && (dispLongLine)){
				double dist = pixelToDistanceX(alf.getLongestSeg()[1], 0);
				vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.BOTTOM_LEFT,
						new VzText(VzText.ANCHOR.BOTTOM_LEFT, 
						"Longest Segment Mean:" + alf.getLongestSeg()[0] + ", " + alf.getLongestSeg()[1] + "\n" + "Distance to Target:" + dist)));
			}
			if(dispGUI){
				vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.BOTTOM_LEFT,
					new VzText(VzText.ANCHOR.BOTTOM_LEFT, "hue: " + checkHSV[0] + "\nsaturation: " + checkHSV[1] + "\nvalue: " + checkHSV[2] + "\nmouse:" + mouse[0] + ", " + mouse[1])));
				vb.swap();
			}
			
			publish(triangles, alf.getLineSegs());
					
		   
		}
	}

	public static double pixelToDistanceX(double pixelY, double BLAH){
		return (-413.1089 / (pixelY - 503.80119) - 0.30247) * 0.3048;
	}
	

	static class PointComparator implements Comparator<double[]>
	{
	     public int compare(double[] p1, double[] p2)
	     {
		 return (int)(p2[0] - p1[0]);
	     }
	}

	public static void main(String args[]) throws IOException
	{
		ArrayList<String> urls = ImageSource.getCameraURLs();

		String url = null;
		if (urls.size()==1)
			url = urls.get(0);

		if (args.length > 0)
			url = args[0];

		if (url == null) {
			System.out.printf("Cameras found:\n");
			for (String u : urls)
				System.out.printf("  %s\n", u);
			System.out.printf("Please specify one on the command line.\n");
			return;
		}

		ImageSource is = ImageSource.make(url);
		new ImageProcessing(is, false).run();
	}


}
