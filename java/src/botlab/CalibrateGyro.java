package botlab;

import java.awt.event.*;
import april.jmat.geom.*;
import april.vis.*;

public class CalibrateGyro extends VisEventAdapter
{
	boolean calibrating,calibdone;
	int calibcount;
	
	public void CalibrateGyro()
	{
		calibcount = 0;
		calibdone = false;
		calibrating = false;
	}
	

	public boolean keyTyped(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
    {
		char c = e.getKeyChar();        
		if(c =='c')
		{
			if ((calibcount % 2) == 0)
			{	
				calibdone = false;
				calibrating = true;
			}
			else
			{
				calibdone = true;
				calibrating = false;			
			}
		}
		calibcount++;
		return false;	
    }
 
	public boolean isCalibrated(){return calibdone;}
	public boolean isCalibrating(){return calibrating;}    
}

