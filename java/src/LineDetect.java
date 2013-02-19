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

	static final int DEFAULT_GRAY_THRESHOLD = 235;
    ImageSource is;

    JFrame jf = new JFrame("LED Tracker Demo");
    JImage jim = new JImage();

    ParameterGUI pg = new ParameterGUI();


    public LineDetect(ImageSource _is)
    {
        is = _is;

        // Determine which slider values we want
        pg.addIntSlider("kthresh","Brightness Threshold",0,255,DEFAULT_GRAY_THRESHOLD);

        jim.setFit(true);

        // Setup window layout
        jf.setLayout(new BorderLayout());
        jf.add(jim, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);
        jf.setSize(1024, 768);
        jf.setVisible(true);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
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

            int bounds[] = findLED(im, pg);

            // Display the detection, by drawing on the image
            if (true) {
                // draw the horizontal lines
                for (int y : new int[]{bounds[1], bounds[3]})
                    for (int x = bounds[0]; x <=bounds[2]; x++) {
                        im.setRGB(x,y, 0xff0000ff); //Go Blue!
                    }

                // draw the horizontal lines
                for (int x : new int[]{bounds[0], bounds[2]})
                    for (int y = bounds[1]; y <=bounds[3]; y++) {
                        im.setRGB(x,y, 0xff0000ff); //Go Blue!
                    }

                jim.setImage(im);
            }
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
