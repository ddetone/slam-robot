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


public class WallDetector extends VisEventAdapter
{

	static final double DEFAULT_CALIBRATE_VAL = -2550;
	static final double DEFAULT_THRESH_VAL = 100;
	static final int DEFAULT_NUM_STEPS = 200;
	static final int DEFAULT_POINT_SPREAD = 50;
	static final double DEFAULT_CORNER_ANG_THRESH = 0.6;
	static final double DEFAULT_BLUE_THRESH = 100;

	double calibrateVal = DEFAULT_CALIBRATE_VAL;
	double threshold = DEFAULT_THRESH_VAL;
	int numSteps = DEFAULT_NUM_STEPS;
	int pointSpreadMax = DEFAULT_POINT_SPREAD;
	double cornerAngleThresh = DEFAULT_CORNER_ANG_THRESH;
	double blueThresh = DEFAULT_BLUE_THRESH;

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


	int data[];

        static VisWorld vw = new VisWorld();
        static VisLayer vl  = new VisLayer(vw);
        static VisCanvas vc = new VisCanvas(vl);
	static VisWorld.Buffer vb;

	static AgglomerativeLineFitter alf = null;


	public WallDetector(ImageSource _is)
	{
		is = _is;

		// Determine which slider values we want
		pg.addDoubleSlider("calib", "Calibrate Constant", -5000, -1, DEFAULT_CALIBRATE_VAL);
		pg.addDoubleSlider("blueThresh", "Blue Thresh", 0, 360, DEFAULT_BLUE_THRESH);
		pg.addDoubleSlider("thresh", "Error Threshold", 0, 200,DEFAULT_THRESH_VAL);
		pg.addIntSlider("num_steps", "Num Steps", 0,200,DEFAULT_NUM_STEPS);
		pg.addIntSlider("point_spread", "Point Spread Max", 0,100,DEFAULT_POINT_SPREAD);
		pg.addDoubleSlider("cornerAngleThresh", "Corner Angle Threshold", 0,1,DEFAULT_CORNER_ANG_THRESH);
		
		
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

		//int maxBlue[] = new int[width];
		BLUE_STATE state[] = new BLUE_STATE[width];

		Arrays.fill(state, BLUE_STATE.PREBLUE);

		for(int j = height - 1; j >= height/2; j--){
			for(int i = 0; i < width; i += 10){
				if(state[i] != BLUE_STATE.POSTBLUE){
					if(blueScore(data[j * width + i]) > blueThresh){
					//if((data[j * width + i] & 0x000000ff) > blueThresh){
						//maxBlue[i] = j;
						state[i] = BLUE_STATE.POSTBLUE;
						maxes.add(new double[]{(double)i, height-(double)j});
					}else if(state[i] == BLUE_STATE.INBLUE){
						state[i] = BLUE_STATE.POSTBLUE;
						maxes.add(new double[]{(double)i, height-(double)j});
					}
				}
			}
		}

		maxes.add(new double[]{0,height - 0});
		return maxes;
		
	}

	public double blueScore(int aRGB){
		
		float hsv[] = new float[3];
		Color.RGBtoHSB(((aRGB & 0x00FF0000) >> 16), ((aRGB & 0x0000FF00) >> 8), (aRGB & 0x000000FF), hsv);
		hsv[0] = (hsv[0] - (int)hsv[0]) * 360;
		int distFromBlue = Math.max(BLUE, (int)hsv[0]) - Math.min(BLUE, (int)hsv[0]);
		return(double)(BLUE - distFromBlue);
		
	}
	
	public void run()
	{

		//System.out.println(blueScore(0xff0000ff));
		//System.exit(1);		

		is.start();
		ImageSourceFormat fmt = is.getCurrentFormat();

		// Initialize visualization environment now that we know the image dimensions
		width = fmt.width;
		height = fmt.height;
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

			ArrayList<double []> topBluePixels = findTape();
			Collections.sort(topBluePixels,new PointComparator());
			/*for(int j = 0; j < height; j++){		
				for(int i = 0; i < width; i++){
					if(blueScore(data[j * width + i]) > blueThresh) data[j * width + i] = 0xffff0000;
				}
			}*/
			vb.addBack(new VisChain(LinAlg.translate(0,0,5), new VzPoints(new VisVertexData(topBluePixels),
					new VzPoints.Style(Color.green, 5))));
			alf.setPoints(topBluePixels);
			alf.findSegs();

			alf.plotLineSegs(vb);
			vb.addBack(new VzImage(im, VzImage.FLIP));
			vb.swap();
		   
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
