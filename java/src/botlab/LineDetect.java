package botlab;

import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.image.*;
import javax.swing.*;

import april.jcam.*;
import april.util.*;
import april.jmat.*;


public class LineDetect
{

	//static final int DEFAULT_GRAY_THRESHOLD = 235;
	ImageSource is;

	JFrame jf = new JFrame("Calibration");
	
	JImage jim = new JImage();

	ParameterGUI pg = new ParameterGUI();

	int width = 0;
	int height = 0;
	
	float pixelDists[] = null;
	float pixelThetas[] = null;

	public LineDetect(ImageSource _is)
	{
		is = _is;

		// Determine which slider values we want
		pg.addDoubleSlider("calib","Calibrate",-5000,-1,-500);

		jim.setFit(true);

		// Setup window layout
		jf.setLayout(new BorderLayout());
		jf.add(jim, BorderLayout.CENTER);
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
		
		double B = 1.0 / pg.gd("calib");
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
	
	
	public void calibrate(BufferedImage im)
	{
	
		int data[] = ((DataBufferInt) (im.getRaster().getDataBuffer())).getData();
	
		int[] temp = new int[data.length];
		Arrays.fill(temp, 0);

		int index;
		
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				index = convertPix(i * width + j);
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
	
	public void run()
	{
		is.start();
		ImageSourceFormat fmt = is.getCurrentFormat();

		// Initialize visualization environment now that we know the image dimensions
		width = fmt.width;
		height = fmt.height;
		pixelDists = new float[height * width];
		pixelThetas = new float[height * width];

		for(int i = 0; i < height; i++){
			for(int j = 0; j < width; j++){
				pixelDists[i * width + j] = dist2d(width/2, j, height/2, i);
				pixelThetas[i * width + j] = (float)Math.atan2(i - height/2, j - width/2);
			}
		}

		while(true) {
			// read a frame
			byte buf[] = is.getFrame().data;
			if (buf == null)
				continue;

			// Grab the image, and convert it to gray scale immediately
			BufferedImage im = ImageConvert.convertToImage(fmt.format, fmt.width, fmt.height, buf);

			
			calibrate(im);
				
			//width = fmt.width;
			//height = fmt.height;

			jim.setImage(im);
		   
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
		new LineDetect(is).run();
	}


}
