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

	int width=1028;
	int height=768;

    public LineDetect(ImageSource _is)
    {
        is = _is;

        // Determine which slider values we want
        pg.addIntSlider("calib","Calibrate",0,2000,500);

        jim.setFit(true);

        // Setup window layout
        jf.setLayout(new BorderLayout());
        jf.add(jim, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);
        jf.setSize(width, height);
        jf.setVisible(true);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }
   
	public double dist2d(double x1, double x2, double y1, double y2)
	{
		return Math.sqrt(Math.pow(x2-x1,2)+Math.pow(y2-y1,2));
	}
	    
    public int convertPix(int index)
    {
    	int val;

		//calculate x & y pos
    	int x = index%width;
    	int y = (index - x)/width;

		//x = 750;
		//y = 550;
    	
		//need to shift x&y so that middle pixel is (0,0) 
		x = x - (int)(width/2);
		y = y - (int)(height/2);
	
		double th_out = 0;
		

		th_out = MathUtil.atan2(y,x);
		//double B = pg.gi("calib");	
			

		double B = (1/(double)pg.gi("calib"));
		double r = dist2d(0,x,0,y);

		//System.out.printf("x:%d, y:%d, TH:%f, B:%f\n",x,y,th_out,B);
		//System.out.printf("before r:%f\n",r);
		r = r + B*r*r;
		//System.out.printf("after r:%f\n",r);
		

		double nx = x;
		double ny = y;
		nx = r*Math.cos(th_out);
		
		ny = r*Math.sin(th_out);
				
		//System.out.printf("nx:%f, ny:%f\n\n",nx,ny);


		//nx = nx + (int)(width/2);
		//ny = ny + (int)(height/2);
		
		//System.out.printf("nx:%f, ny:%f\n\n",nx,ny);

		if (nx < 0 || nx >= width || ny < 0 || ny >= height)
			val = -1;
		else
			val = (int)(ny*width+nx);
		
		return index;
    
    }
    
    
    public void calibrate(BufferedImage im, ParameterGUI pg)
    {
    
    	int data[] = ((DataBufferInt) (im.getRaster().getDataBuffer())).getData();
    
    	int[] datanew = new int[data.length];
    	//int[] table = new int[data.length];
    	


    	int index;
    	
    	for (int i=0; i<height; i++)
    	{
    		for (int j=0; j<width; j++)
    		{
    			index = convertPix(i*width+j);
    			datanew[i*width+j] = index;
    			
    		}	
    	}
    	
		int loc;
    	for (int i=0; i<height; i++)
    	{
    		for (int j=0; j<width; j++)
    		{
    			//data[i*width+j] = datanew[i*width+j];
				//data[i*width+j] = 0x00FF00FF;
				loc = datanew[i*width+j];
				if (loc != -1)
				{
					data[loc] = data[i*width+j];

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

        while(true) {
            // read a frame
            byte buf[] = is.getFrame().data;
            if (buf == null)
                continue;

            // Grab the image, and convert it to gray scale immediately
            BufferedImage im = ImageConvert.convertToImage(fmt.format, fmt.width, fmt.height, buf);
			
            calibrate(im, pg);
				
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
