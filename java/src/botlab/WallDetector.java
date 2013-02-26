package botlab;

import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.image.*;
import javax.swing.*;

import april.jcam.*;
import april.util.*;
import april.jmat.*;
import april.vis.*;
import lcm.lcm.*;

import botlab.lcmtypes.*;

public class WallDetector extends VisEventAdapter
{

	static final double DEFAULT_CALIBRATE_VAL = -2550;
	static final double DEFAULT_THRESH_VAL = 90;
	static final int DEFAULT_NUM_STEPS = 200;
	static final int DEFAULT_POINT_SPREAD = 26;
	static final double DEFAULT_CORNER_ANG_THRESH = 0.6;
	static final double DEFAULT_BLUE_THRESH = 90;
	static final boolean DEFAULT_DISP_LONG_LINE = false;
	static final boolean DEFAULT_DISP_BLUE_PIXELS = false;
	static final boolean DEFAULT_DISP_LINES = false;
	static final boolean DEFAULT_DISP_CORNERS = false;

	double calibrateVal = DEFAULT_CALIBRATE_VAL;
	double threshold = DEFAULT_THRESH_VAL;
	int numSteps = DEFAULT_NUM_STEPS;
	int pointSpreadMax = DEFAULT_POINT_SPREAD;
	double cornerAngleThresh = DEFAULT_CORNER_ANG_THRESH;
	double blueThresh = DEFAULT_BLUE_THRESH;
	boolean dispLongLine  = DEFAULT_DISP_LONG_LINE;
	boolean dispBluePixels = DEFAULT_DISP_BLUE_PIXELS;
	boolean dispLines = DEFAULT_DISP_LINES;
	boolean dispCorners = DEFAULT_DISP_CORNERS;

	public enum BLUE_STATE {PREBLUE, INBLUE, POSTBLUE};

	static final int BLUE = 240;

	ImageSource is;

	JFrame jf = new JFrame("Wall Detector");
	JImage jim = new JImage();

	ParameterGUI pg = new ParameterGUI();

	int width = 0;
	int height = 0;

	float pixelDists[] = null;
	float pixelThetas[] = null;
	int indexLookup[] = null;

	float hsvImage[][] = null;

	int data[];

        static VisWorld vw = new VisWorld();
        static VisLayer vl  = new VisLayer(vw);
        static VisCanvas vc = new VisCanvas(vl);
	static VisWorld.Buffer vb;

	static AgglomerativeLineFitter alf = null;

	// homography matrix
	Matrix H = null;

    LCM lcm;


	public WallDetector(ImageSource _is)
	{
		is = _is;

        lcm = LCM.getSingleton();

		// Determine which slider values we want
		pg.addDoubleSlider("calib", "Calibrate Constant", -5000, -1, DEFAULT_CALIBRATE_VAL);
		pg.addDoubleSlider("blueThresh", "Blue Thresh", 0, 360, DEFAULT_BLUE_THRESH);
		pg.addDoubleSlider("thresh", "Error Threshold", 0, 200,DEFAULT_THRESH_VAL);
		pg.addIntSlider("num_steps", "Num Steps", 0,200,DEFAULT_NUM_STEPS);
		pg.addIntSlider("point_spread", "Point Spread Max", 0,100,DEFAULT_POINT_SPREAD);
		pg.addDoubleSlider("cornerAngleThresh", "Corner Angle Threshold", 0,1,DEFAULT_CORNER_ANG_THRESH);
		pg.addCheckBoxes("dispLongLine", "Show Longest Line Mean", DEFAULT_DISP_LONG_LINE);
		pg.addCheckBoxes("dispBluePixels", "Show Pixels", DEFAULT_DISP_BLUE_PIXELS);
		pg.addCheckBoxes("dispLines", "Show Lines", DEFAULT_DISP_LINES);
		pg.addCheckBoxes("dispCorners", "Show Corners", DEFAULT_DISP_CORNERS);


		pg.addListener(new ParameterListener() {
			public void parameterChanged(ParameterGUI pg, String name)
			{
				if(name == "calib"){
					calibrateVal = pg.gd("calib");
					calculateLookupTable();
				}
				if(name == "blueThresh")blueThresh = pg.gd("blueThresh");
				if(name == "thresh"){
					threshold = pg.gd("thresh");
					alf.threshold = threshold;
				}
				if(name == "num_steps"){
					numSteps = pg.gi("num_steps");
					alf.numSteps = numSteps;
				}
				if(name == "point_spread"){
					pointSpreadMax = pg.gi("point_spread");
					alf.pointSpreadMax = pointSpreadMax;
				}
				if(name == "cornerAngleThresh"){
					cornerAngleThresh = pg.gd("cornerAngleThresh");
					alf.cornerAngleThresh = cornerAngleThresh;
				}
				if(name == "dispLongLine"){
					dispLongLine = pg.gb("dispLongLine");
				}
				if(name == "dispBluePixels"){
					dispBluePixels = pg.gb("dispBluePixels");
				}
				if(name == "dispLines"){
					dispLines = pg.gb("dispLines");
				}
				if(name == "dispCorners"){
					dispCorners = pg.gb("dispCorners");
				}
			}
		});


		jim.setFit(true);
		vl.addEventHandler(this);
		vl.cameraManager.uiLookAt(new double[]{640.0, 480.0, 1000.0}, new double[]{640.0,480.0,0.0}, new double[]{0.0,1.0,0.0}, true);

		// Setup window layout
		jf.setLayout(new BorderLayout());
		jf.add(vc, BorderLayout.CENTER);
		vb = vw.getBuffer("WallDetector");
		jf.add(pg, BorderLayout.SOUTH);
		jf.setSize(1024, 768);
		jf.setVisible(true);
		jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
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

	public ArrayList<double []> findTape(){

		ArrayList<double []> maxes = new ArrayList<double []>();

		int numPostBlue = 0;

		//int maxBlue[] = new int[width];
		BLUE_STATE state[] = new BLUE_STATE[width];

		Arrays.fill(state, BLUE_STATE.PREBLUE);

		for(int j = height - 1; ((j >= height - 480) && (numPostBlue < width)); j--){
			for(int i = 0; ((i < width) && (numPostBlue < width)); i += 10){
				if(state[i] != BLUE_STATE.POSTBLUE){
					if(blueScore(i, j) > blueThresh){
					//if((data[j * width + i] & 0x000000ff) > blueThresh){
						//maxBlue[i] = j;
						state[i] = BLUE_STATE.POSTBLUE;
						numPostBlue += 1;
						maxes.add(new double[]{(double)i, height-(double)j});
					}else if(state[i] == BLUE_STATE.INBLUE){
						state[i] = BLUE_STATE.POSTBLUE;
						numPostBlue += 1;
						maxes.add(new double[]{(double)i, height-(double)j});
					}
				}
			}
		}

		maxes.add(new double[]{0,height - 0});
		return maxes;

	}

	public void convertToHSV(){
		for(int j = height - 1; j >= height - 480; j--){
			for(int i = 0; i < width; i++){
				//System.out.println((height - j) * width + i);
				int aRGB = data[j * width + i];
				Color.RGBtoHSB(((aRGB & 0x00FF0000) >> 16), ((aRGB & 0x0000FF00) >> 8), (aRGB & 0x000000FF), hsvImage[(height - (j + 1)) * width + i]);
				float temp = hsvImage[(height - (j + 1)) * width + i][0];
				hsvImage[(height - (j + 1)) * width + i][0] = (temp - (int)temp) * 360;
			}
		}
	}


	public double blueScore(int i, int j){

		//System.out.println((height - (j + 1)) * width + i);
		int distFromBlue = Math.max(BLUE, (int)hsvImage[(height - (j + 1)) * width + i][0]) - Math.min(BLUE, (int)hsvImage[(height - (j + 1)) * width + i][0]);
		return(double)(BLUE - distFromBlue);

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
		hsvImage = new float[width * 480][3];
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

			// Grab the image, and convert it to gray scale immediately
			BufferedImage im = ImageConvert.convertToImage(fmt.format, fmt.width, fmt.height, buf);

			data = ((DataBufferInt) (im.getRaster().getDataBuffer())).getData();

			calibrate();
			convertToHSV();

			ArrayList<double []> topBluePixels = findTape();
			Collections.sort(topBluePixels,new PointComparator());


			alf.setPoints(topBluePixels);
			alf.findSegs();

			if(dispBluePixels) vb.addBack(new VisChain(LinAlg.translate(0,0,5), new VzPoints(new VisVertexData(topBluePixels),
					new VzPoints.Style(Color.green, 5))));
			if(dispLines) alf.plotLineSegs(vb);
			if(dispCorners) alf.plotCorners(vb);
			vb.addBack(new VzImage(im, VzImage.FLIP));
			if(dispLongLine){
				double dist = (-413.1089/(alf.getLongestSeg()[1]-503.80119)-0.30247)*0.3048;
				vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.BOTTOM_LEFT,
						new VzText(VzText.ANCHOR.BOTTOM_LEFT,
						"Longest Segment Mean:" + alf.getLongestSeg()[0] + ", " + alf.getLongestSeg()[1] + "\n" + "Distance to Target:" + dist)));
			}
			vb.swap();



            //map_features_t map_features = new map_features_t();
            //map_features.utime = TimeUtil.utime();

            //ArrayList<ArrayList<double[]>> lineSegs = alf.getLineSegs();

            //This code publishes all the line segments but this is not what we need
            /*map_features.nlineSegs = lineSegs.size();
            for(int i = 0; i < lineSegs.size() ; i++)
            {
                double[] startPoint = lineSegs.get(i).get(0);
                double[] endPoint   = lineSegs.get(i).get(1);

                map_features.lineSegs[i][0] = startPoint[0];
                map_features.lineSegs[i][1] = startPoint[1];
                map_features.lineSegs[i][2] = endPoint[0];
                map_features.lineSegs[i][3] = endPoint[1];
            }*/

            //map_features.nlineSegs = 1;
            //map_features.lineSegs[1][0] = alf.getLongestSeg

            //lcm.publish("6_MAP_FEATURES", map_features);

		}
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
		new WallDetector(is).run();
	}


}
