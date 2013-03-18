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
import april.image.*;
import april.jmat.geom.*;

import botlab.lcmtypes.*;

import lcm.lcm.*;

public class RobotGUI extends VisEventAdapter implements LCMSubscriber
{

	JFrame jf = new JFrame("RobotGUI");

	VisWorld vw = new VisWorld();
	VisLayer vl = new VisLayer(vw);
	VisCanvas vc = new VisCanvas(vl);

	ParameterGUI pg = new ParameterGUI();

	LCM lcm;

	final boolean verbose = false;

	bot_status_t bot_status = new bot_status_t();
	bot_status_t curr_bot_status = null;
	bot_status_t last_bot_status = null;
	double[] slamBot = null;
	battery_t battery;

	ArrayList<double[]> robotTraj = new ArrayList<double[]>();
	final boolean DEFAULT_SEND_WAYPOINT = false;
	final boolean DEFAULT_DISP_CONV = false;
	boolean sendWayPoint = DEFAULT_SEND_WAYPOINT;
	boolean dispConv = DEFAULT_DISP_CONV;

	RobotGUI()
	{
		try{
			this.lcm = new LCM("udpm://239.255.76.67:7667?ttl=1");
		}catch(IOException e){
			this.lcm = LCM.getSingleton();
		}
		lcm.subscribe("6_POSE",this);
		lcm.subscribe("6_BATTERY",this);
		lcm.subscribe("6_MAP",this);
		lcm.subscribe("6_GOAL",this);
		lcm.subscribe("6_WAYPOINT", this);
		lcm.subscribe("6_SLAM_POSES",this);

		jf.setLayout(new BorderLayout());
		jf.add(vc, BorderLayout.CENTER);
		jf.add(pg, BorderLayout.SOUTH);

		jf.setSize(800,600);
		jf.setVisible(true);
		jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		pg.addCheckBoxes("sendWayPoint", "Send Waypoint", DEFAULT_SEND_WAYPOINT);
		pg.addCheckBoxes("dispConv", "Display Exploded Wall Map", DEFAULT_DISP_CONV);


		pg.addListener(new ParameterListener() {
			public void parameterChanged(ParameterGUI pg, String name)
			{
				if(name == "sendWayPoint")sendWayPoint = pg.gb("sendWayPoint");
				else if(name == "dispConv")dispConv = pg.gb("dispConv");
			}
		});

		//vl.cameraManager.uiLookAt(new double[] {-0.21867, -1.71540, 10.96653 },
		//						  new double[] {-0.21867, -1.71540, 0.00000 },
		//						  new double[] { 0.00000,   1.00000, 0.00000 }, true);

		vl.addEventHandler(this);
		vl.cameraManager.uiLookAt(new double[] {-2.66075, 1.22066, 1.70393 },
					new double[] {1.75367, -0.06226,  0.00000 },
					new double[] {0.33377, -0.09695,  0.93766 }, true);

		VisWorld.Buffer vb = vw.getBuffer("Ground");
		vb.addBack(new VisChain(LinAlg.translate(0,0,-0.025),new VzBox(30,30,0.05,new VzMesh.Style(Color.darkGray))));
		vb.swap();

	}


	public boolean mouseReleased(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
	{
		if(sendWayPoint){
			xyt_t wayPoint = new xyt_t();
			double temp[] = ray.intersectPlaneXY();
			wayPoint.utime = TimeUtil.utime();
			wayPoint.xyt = new double[]{temp[0], temp[1], Math.toRadians(30)};
			
			if((slamBot == null) || (curr_bot_status.xyt == null))
				return true;
			
			//*
			double[] T = LinAlg.xytInvMul31(slamBot, wayPoint.xyt);
			wayPoint.xyt = LinAlg.xytMultiply(curr_bot_status.xyt, T);
			//*/
			lcm.publish("6_WAYPOINT", wayPoint);

			//pg.sb("sendWayPoint",false);
			return true;
		}else return false;
	}


	public void drawMap(map_t map)
	{
		//System.out.println("drawMap");
		double dispConvThresh = 0.99;
		if(dispConv == true)dispConvThresh = 0.6;
		boolean found_point = false;
		VisWorld.Buffer vb = vw.getBuffer("Map");
		for(int i = 0; i < map.size; ++i){
			for(int j = 0; j < map.size; ++j){
				if((int) (map.cost[i][j] & 0xFF) > dispConvThresh*255){
					VzBox mapBox = new VzBox(map.scale,map.scale,((double)((int)(map.cost[i][j] & 0xFF)))/map.max*.06*3, new VzMesh.Style(Color.red));
					VisObject vo_mapBox = new VisChain(LinAlg.translate(i*map.scale-map.size/2*map.scale,j*map.scale-map.size/2*map.scale,0.0),mapBox);

					vb.addBack(vo_mapBox);
				}
				if((int) (map.knowledge[i][j]) == 1){
					VzBox mapBox = new VzBox(map.scale,map.scale,.005, new VzMesh.Style(Color.blue));
					VisObject vo_mapBox = new VisChain(LinAlg.translate(i*map.scale-map.size/2*map.scale,j*map.scale-map.size/2*map.scale,0.0),mapBox);
					vb.addBack(vo_mapBox);
				}
			}
		}

		vb.addBack(new VisChain(LinAlg.translate(0,0,-0.025),new VzBox(map.size * map.scale, map.size * map.scale,0.05,new VzMesh.Style(Color.darkGray))));
		vw.getBuffer("Ground").swap();
		vb.swap();
	}


	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_POSE"))
			{

				curr_bot_status = new bot_status_t(dins);
				double[]T;
				if(last_bot_status != null) T = LinAlg.xytInvMul31(last_bot_status.xyt, curr_bot_status.xyt);
				else T = new double[3];
				bot_status.xyt = LinAlg.xytMultiply(bot_status.xyt, T);
				bot_status.utime = curr_bot_status.utime;
				bot_status.xyt_dot = curr_bot_status.xyt_dot;
				bot_status.yaw = curr_bot_status.yaw;
				bot_status.cov = curr_bot_status.cov;
				bot_status.voltage = curr_bot_status.voltage;

				drawRobot();
				//drawCovariance();

				VisWorld.Buffer vb = vw.getBuffer("Battery");
				if((bot_status.voltage < 8.5) && (bot_status.voltage != 0)){
					vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.CENTER, 
									new VzText(VzText.ANCHOR.CENTER, 
									"<<sansserif-bold-16,white>>LOW BATTERY VOLTAGE:" + 
									String.format("%.3g%n", bot_status.voltage))));
				}
				vb.swap();

				last_bot_status = curr_bot_status;
			}
			else if(channel.equals("6_BATTERY"))
			{

			}
			else if(channel.equals("6_MAP"))
			{
				map_t map = new map_t(dins);
				drawMap(map);
			}
			else if(channel.equals("6_WAYPOINT"))
			{
				xyt_t point = new xyt_t(dins);
				VisWorld.Buffer vb = vw.getBuffer("Waypoint");
				VzCircle pointBox = new VzCircle(.04, new VzMesh.Style(Color.yellow));
				VisObject vo_pointBox = new VisChain(LinAlg.translate(point.xyt[0], point.xyt[1], 0.009),pointBox);
				vb.addBack(vo_pointBox);
				vb.swap();
			}
			else if(channel.equals("6_GOAL"))
			{

				xyt_t point = new xyt_t(dins);
				if(verbose)System.out.println("found goal at: "+point.xyt[0]+","+point.xyt[1]);
				VisWorld.Buffer vb = vw.getBuffer("Goal");
				VzCircle pointBox = new VzCircle(.03, new VzMesh.Style(Color.white));
				VisObject vo_pointBox = new VisChain(LinAlg.translate(point.xyt[0], point.xyt[1], 0.01),pointBox);
				vb.addBack(vo_pointBox);
				vb.swap();
			}
			else if(channel.equals("6_SLAM_POSES"))
			{
				slam_vector_t slamVec = new slam_vector_t(dins);
				ArrayList<double[]>vec = new ArrayList<double[]>();

				for(int i = 0; i < slamVec.numPoses; i++)
					vec.add(new double[]{ slamVec.xyt[i].xyt[0], slamVec.xyt[i].xyt[1], 0.005 });

				bot_status.xyt = slamVec.xyt[slamVec.numPoses - 1].xyt;
				slamBot = LinAlg.copy(slamVec.xyt[slamVec.numPoses - 1].xyt);

				VisWorld.Buffer vb = vw.getBuffer("Robot_Path_SLAM");
				vb.addBack(new VzPoints(new VisVertexData(vec), new VzPoints.Style(Color.magenta,2)));
				vb.swap();


				//Draws the covariance ellipse for the slam pose
				VisWorld.Buffer vb_pc = vw.getBuffer("CovarianceEllipseSlam");
				double[][] cov22 = new double[][]{{slamVec.poseCov.cov[0][0], slamVec.poseCov.cov[0][1]},
												  {slamVec.poseCov.cov[1][0], slamVec.poseCov.cov[1][1]}};


				vb_pc.addBack(new VisChain(LinAlg.translate(0,0,0.01),
						new VzEllipse(new double[]{bot_status.xyt[0],bot_status.xyt[1]}, cov22, new VzMesh.Style(Color.black))));
				vb_pc.swap();


				//Draws the triangles as given by the slam vector LCM message
				VisWorld.Buffer vb_t = vw.getBuffer("Triangles");
				VisWorld.Buffer vb_tc = vw.getBuffer("CovarianceTriangles");
				for(int i = 0; i < slamVec.numTriangles ; i++)
				{
					VzTriangle trB = new VzTriangle(0.085,0.085,0.085, new VzMesh.Style(Color.black));
					VzTriangle trG = new VzTriangle(0.085,0.085,0.085, new VzMesh.Style(Color.green));
					VisObject tr = new VisChain(trB, LinAlg.translate(0,0,0.001), trG);
					VisObject vo_tr = new VisChain(LinAlg.translate(slamVec.triangles[i][0], slamVec.triangles[i][1], 0.15),
												   LinAlg.rotateZ(slamVec.triangles[i][2]),
												   LinAlg.rotateY(-Math.PI/2),
												   LinAlg.rotateZ(Math.PI/2),
												   tr);
					vb_t.addBack(vo_tr);

					//Draws the covariance of the triangles
					double[][] cov22_t = new double[][]{{slamVec.trianglesCov[i].cov[0][0], slamVec.trianglesCov[i].cov[0][1]},
													 {slamVec.trianglesCov[i].cov[1][0], slamVec.trianglesCov[i].cov[1][1]}};

					vb_tc.addBack(new VisChain(LinAlg.translate(0,0,0.1),
							new VzEllipse(new double[]{slamVec.triangles[i][0], slamVec.triangles[i][1]}, cov22_t, new VzMesh.Style(Color.gray))));
;


				}
				vb_tc.swap();
				vb_t.swap();



			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public void drawRobot()
	{
		double[] xyt = new double[3];
		xyt[0] = bot_status.xyt[0];
		xyt[1] = bot_status.xyt[1];
		xyt[2] = bot_status.xyt[2];

		double wheelRadius = 0.04;
		VzBox base = new VzBox(0.155,0.166,0.07, new VzMesh.Style(Color.red));
		VisObject vo_base = new VisChain(LinAlg.translate(0,0.06,0.10),base);

		VzBox cameraBase = new VzBox(0.05,0.01,0.04, new VzMesh.Style(Color.white));
		VisObject vo_cameraBase = new VisChain(LinAlg.translate(0,0,0.145),cameraBase);

		VzCylinder wheels = new VzCylinder(wheelRadius,0.01, new VzMesh.Style(Color.white));
		VisObject vo_wheels = new VisChain(LinAlg.rotateY(Math.PI/2),LinAlg.translate(-wheelRadius,0,0.09),wheels,LinAlg.translate(0,0,-0.18),wheels);

		double castorRad = 0.03;
		VzCylinder castor = new VzCylinder(castorRad,0.02, new VzMesh.Style(Color.black));
		VisObject vo_castor = new VisChain(LinAlg.rotateY(Math.PI/2), LinAlg.translate(-castorRad,0.115,0),castor);

		VisChain pandaBot = new VisChain();

		pandaBot.add(vo_base,vo_cameraBase,vo_wheels,vo_castor);

		VisWorld.Buffer vb = vw.getBuffer("Robot");

		//vb.addBack(new VzAxes());
		//vb.addBack(new VisChain(LinAlg.translate(xyt[0],xyt[1],0), LinAlg.rotateZ(xyt[2]-Math.PI/2),new VzTriangle(0.25,0.4,0.4,new VzMesh.Style(Color.GREEN))));
		vb.addBack(new VisChain(LinAlg.translate(xyt[0],xyt[1],0), LinAlg.rotateZ(xyt[2]-Math.PI/2),pandaBot));

		vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.BOTTOM_LEFT,new VzText(VzText.ANCHOR.BOTTOM_LEFT, "Angle = " + Math.toDegrees(xyt[2]))));
		vb.swap();

		robotTraj.add(new double[]{curr_bot_status.xyt[0],curr_bot_status.xyt[1],0.005});
		vb = vw.getBuffer("Robot_Path");
		//vb.addBack(new VisChain(LinAlg.translate(xyt[0], xyt[1], 0), new VzPoints()));
		vb.addBack(new VzPoints(new VisVertexData(robotTraj), new VzPoints.Style(Color.gray,2)));
		vb.swap();

	}

	public void drawCovariance()
	{
		//double[] eigTheta = computerEigenValues();
		VisWorld.Buffer vb = vw.getBuffer("CovarianceEllipse");
		double[][] cov22 = new double[][]{{curr_bot_status.cov[0][0], curr_bot_status.cov[0][1]},
										  {curr_bot_status.cov[1][0], curr_bot_status.cov[1][1]}};

		double covAngle = curr_bot_status.cov[2][2];

		vb.addBack(new VisChain(LinAlg.translate(0,0,0.01),new VzEllipse(new double[]{curr_bot_status.xyt[0],curr_bot_status.xyt[1]}, cov22, new VzMesh.Style(Color.black))));

		//vb.addBack(new VzAxes());
		//draw covariance in pose lines
		//*
		vb.addBack(new VisChain(LinAlg.translate(bot_status.xyt[0],bot_status.xyt[1],0.002),LinAlg.rotateZ(bot_status.xyt[2]),LinAlg.rotateY(Math.PI/2),LinAlg.translate(0,0.3*Math.asin(covAngle/2),0.3),LinAlg.rotateX(-covAngle),new VzBox(0.003,0.003,0.6,new VzMesh.Style(Color.green))));
		vb.addBack(new VisChain(LinAlg.translate(bot_status.xyt[0],bot_status.xyt[1],0.002),LinAlg.rotateZ(bot_status.xyt[2]),LinAlg.rotateY(Math.PI/2),LinAlg.translate(0,0.3*Math.asin(-covAngle/2),0.3),LinAlg.rotateX(covAngle),new VzBox(0.003,0.003,0.6,new VzMesh.Style(Color.green))));
		//*/

		//vb.addBack(new VisChain(LinAlg.translate(0.15,0,0.02),LinAlg.rotateY(Math.PI/2), LinAlg.translate(0.3*Math.asin(1),0,0), LinAlg.rotateX(1),new VzBox(0.005,0.005,0.3,new VzMesh.Style(Color.green))));
		//vb.addBack(new VzBox(0.005,0.005,0.3,new VzMesh.Style(Color.green)));

		vb.swap();
	}
	//returns two eigen values (first one larger) and theta of max eigen vector
	/*double[] computeEigenValues()
	{
		double[][] cov = bot_status.cov;
		double a = cov[0][0];
		double b = cov[0][1];
		double d = cov[1][1];
		double e1 = (a+d)/2 + (Math.sqrt((a+d)*(a+d) - 4*(a*d-b*b)))/2;
		double e2 = (a+d)/2 - (Math.sqrt((a+d)*(a+d) - 4*(a*d-b*b)))/2;
		double theta = Math.atan2(2*b/(a-d));
		if(e1>e2)
			return(new double[]{e1,e2,theta});
		else
			return(new double[]{e2,e1,theta});
	}*/

	public static void main(String[] args) throws Exception
	{
		RobotGUI pl = new RobotGUI();

		while(true)
		{
			Thread.sleep(1000);
		}


	}


}
