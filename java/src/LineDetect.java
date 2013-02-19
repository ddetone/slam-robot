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

	int width = 1028;
	int height = 768;

    public LineDetect(ImageSource _is)
    {
        is = _is;

        // Determine which slider values we want
        pg.addIntSlider("calib","Calibrate",0,5,2);

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
    	
    	//need to shift to corner
    	int x = index%width;
    	int y = (index - x)/width;
    	
		double th_out = 0;
		
		//first quad
		if(x >=0 && y >= 0)
			th_out = Math.atan(y/x);
		//second quad
		else if(x < 0 && y >= 0)
			th_out = Math.atan(y/x) + Math.PI;
		//third quad
		else if(x < 0 && y < 0)
			th_out = Math.atan(y/x) - Math.PI;
		//fourth quad
		else if(x >= 0 && y < 0) 
			th_out = Math.atan(y/x);
			
		double B = 0.1;	
			
		double r = dist2d(0,x,0,y);
		r = r + B*r*r;
		
		double nx = x;
		double ny = y;
		nx = r*Math.cos(th_out);
		ny = r*Math.sin(th_out);
		
		
		if (nx < 0 || nx >= width || ny < 0 || ny >= height)
			val = -1;
		else
			val = (int)(ny*width+nx);
		
		return val;
    
    }
    
    
    public void calibrate(BufferedImage im, ParameterGUI pg)
    {
    
    	int data[] = ((DataBufferInt) (im.getRaster().getDataBuffer())).getData();
    
    	int[] datanew = new int[data.length];
    	int[] table = new int[data.length];
    	
    	for (int i=0; i<height; i++)
    	{
    		for (int j=0; j<width; j++)
    		{
    			datanew[i*width+j] = data[i*width+j];
    		}	
    	}
    	
    	int index;
    	
    	for (int i=0; i<height; i++)
    	{
    		for (int j=0; j<width; j++)
    		{
    			index = convertPix(i*width+j);
    			if (index != -1)
    			{
    				datanew[index] = data[i*width+j];
    			}
    		}	
    	}
    	
    	for (int i=0; i<height; i++)
    	{
    		for (int j=0; j<width; j++)
    		{
    			data[i*width+j] = datanew[i*width+j];
    		}	
    	}    	
    
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
