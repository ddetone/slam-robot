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

public class MapBuilder implements LCMSubscriber
{

	static final double DEFAULT_DECAY_DIST = 0.1;
	static final double DEFAULT_DECAY_RATE = 1.0;
	static final double DEFAULT_INV_KNOWLEDGE_DIST = 2.0;
	static final double DEFAULT_INV_COST_DECAY = 20.0;

	double decay_dist = DEFAULT_DECAY_DIST;
	double decay_rate = DEFAULT_DECAY_RATE;
	double inverse_knowledge_dist = DEFAULT_INV_KNOWLEDGE_DIST;
	double inverse_cost_decay = DEFAULT_INV_COST_DECAY;


	LCM lcm;
	botlab.PoseTracker tracker;
	bot_status_t bot_status;
	map_t map;
	double dist_traveled;
	boolean from_log;
	LinkedList<map_features_t> all_features;



    MapBuilder(boolean data_from_log)
    {
        this.lcm =  LCM.getSingleton();
        map = new map_t();
        bot_status = null;
        all_features = new LinkedList<map_features_t>();

		tracker = botlab.PoseTracker.getSingleton();

		bot_status = new bot_status_t();
        lcm.subscribe("6_POSE",this);
        lcm.subscribe("6_PARAM",this);
        lcm.subscribe("6_FEATURES",this);
        map.scale = 0.06;
        map.max = 255;
        map.size = (int) (10.0/map.scale);
        map.cost = new byte[(int) map.size][(int) map.size];
        map.knowledge = new byte[(int) map.size][(int) map.size];
        dist_traveled = 0.0;
        from_log = data_from_log;
    }

    public void clear() {
    	map.cost = new byte[(int) map.size][(int) map.size];
    	map.max = 255;
    }

	public static bot_status_t copyBotStatus(bot_status_t _bot_status){
		if(_bot_status == null)return null;
		bot_status_t temp = new bot_status_t();
		temp.xyt = new double[]{_bot_status.xyt[0], _bot_status.xyt[1], _bot_status.xyt[2]};
		temp.utime = _bot_status.utime;
		temp.xyt_dot = new double[]{_bot_status.xyt_dot[0], _bot_status.xyt_dot[1], _bot_status.xyt_dot[2]};
		temp.cov = new double[][]{new double[]{_bot_status.cov[0][0], _bot_status.cov[0][1], _bot_status.cov[0][2]},
					new double[]{_bot_status.cov[1][0], _bot_status.cov[1][1], _bot_status.cov[1][2]},
					new double[]{_bot_status.cov[2][0], _bot_status.cov[2][1], _bot_status.cov[2][2]}};
		return temp;
	}

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_PARAM"))
			{
				parameter_t param = new parameter_t(dins);
				if(param.name == "decay_dist" ){
					decay_dist = param.value;
				}
				if(param.name == "decay_rate" ){
					decay_rate = param.value;
				}
				if(param.name == "inverse_knowledge_dist" ){
					inverse_knowledge_dist = param.value;
				}
				if(param.name == "inverse_cost_decay" ){
					inverse_cost_decay = param.value;
				}
			}
			/*else if(channel.equals("6_POSE"))
			{
				if(from_log) {
					bot_status_t new_bot_status = new bot_status_t(dins);
					new_bot_status.xyt[0] += (map.size/2)*map.scale;
					new_bot_status.xyt[1] += (map.size/2)*map.scale;

					bot_status = new_bot_status;
				}
			} */

			else if(channel.equals("6_FEATURES"))
			{
				//System.out.println("got_feature");
				all_features.add(new map_features_t(dins));
				
			}
			else if(channel.equals("6_SLAM_POSES"))
			{
				this.clear();
				slam_vector_t slam_vec = new slam_vector_t(dins);

				for(map_features_t features : all_features)
				{
					bot_status.xyt = null;
					for(int i = 0; i < slam_vec.numPoses; ++i){
						if(slam_vec.xyt[i].utime > features.utime){
							bot_status.xyt = slam_vec.xyt[(int) Math.max(i - 1,0)].xyt;
						}
					}
					
					if(bot_status.xyt == null)
						return;
					//System.out.println("got_pos");
					//LinAlg.print(bot_status.xyt);
					//this.clear();
					
					//bot_status = features.bot;
					LinAlg.print(bot_status.xyt);
					bot_status.xyt[0] += (map.size/2)*map.scale;
					bot_status.xyt[1] += (map.size/2)*map.scale;

					for(int f = 0; f < features.nlineSegs; ++f){
						//System.out.println("adding_feature");
						//System.out.println(f);
						double p1[] = new double[2];
						double p2[] = new double[2];
						double l1[] = {features.lineSegs[f][0],features.lineSegs[f][1],0};
						double l2[] = {features.lineSegs[f][2],features.lineSegs[f][3],0};

						p1 = LinAlg.xytMultiply(bot_status.xyt, l1);
						p2 = LinAlg.xytMultiply(bot_status.xyt, l2);


						//remove things in line of sight (replaces ray casting)
						//uses barycentric coords to tell if it's in triangle
						int xmin = (int) (Math.min(p1[0],Math.min(p2[0],bot_status.xyt[0])) / map.scale);
						int xmax = (int) (Math.max(p1[0],Math.max(p2[0],bot_status.xyt[0])) / map.scale);
						int ymin = (int) (Math.min(p1[1],Math.min(p2[1],bot_status.xyt[1])) / map.scale);
						int ymax = (int) (Math.max(p1[1],Math.max(p2[1],bot_status.xyt[1])) / map.scale);
						double det = ((p2[1]-bot_status.xyt[1])*(p1[0]-bot_status.xyt[0])+ (bot_status.xyt[0]-p2[0])*(p1[1]-bot_status.xyt[1]));
						for(int i = xmin; i < xmax; ++i){
							for(int j = ymin; j < ymax; ++j){
								double lambda1 = ((p2[1]-bot_status.xyt[1])*(i*map.scale - bot_status.xyt[0])  + (bot_status.xyt[0]-p2[0])*(j*map.scale - bot_status.xyt[1]))/det;
								if(lambda1 < 0.0 || lambda1 > 1.0)
									continue;
								double lambda2 = ((bot_status.xyt[1]-p1[1])*(i*map.scale - bot_status.xyt[0])  + (p1[0]-bot_status.xyt[0])*(j*map.scale - bot_status.xyt[1]))/det;
								if(lambda2 < 0.0 || lambda2 > 1.0)
									continue;
								double lambda3 = 1 - lambda1 - lambda2;
								if(lambda3 < 0.0 || lambda3 > 1.0)
									continue;
								map.cost[i][j] = (byte) 0;
								map.knowledge[i][j] = (byte) 1;
							}
						}


						//add points to map
						double dist = LinAlg.distance(p1,p2,2);
						int nsteps = (int) (dist/map.scale)+1;

						for(int i = 0; i <= nsteps; ++i){ //for each pixel between p1 and p2
							
							double wall_point[] = new double[2];
							wall_point[0] = p1[0]*(double)i/nsteps + p2[0]*(1.0-(double)i/nsteps);
							wall_point[1] = p1[1]*(double)i/nsteps + p2[1]*(1.0-(double)i/nsteps);

							//raycast to points
							/*
							double rdist = LinAlg.distance(bot_status.xyt,wall_point,2);
							int rsteps = (int) (rdist/map.scale)+1;
							
							for(int r = 0; r < rsteps; ++r){
								int rx = (int) ((wall_point[0]*(double)i/nsteps + bot_status.xyt[0]*(1.0-(double)i/nsteps))/map.scale);
								int ry = (int) ((wall_point[1]*(double)i/nsteps + bot_status.xyt[1]*(1.0-(double)i/nsteps))/map.scale);
								if(rx > 0 && ry > 0 && rx < map.size && ry < map.size) {
									map.cost[rx][ry] = 0;
									//map.knowledge[rx][ry] = 1;
								}
							}
							*/
							
							int x = (int) (wall_point[0] / map.scale);
							int y = (int) (wall_point[1] / map.scale);

							//System.out.println(x + " " + y);
							if(x > 0 && y > 0 && x < map.size && y < map.size){
								map.cost[x][y] = (byte) 255; //highest cost
								map.knowledge[x][y] = (byte) 2;
							}
							/*
							for(int j = x - 3; j < x + 3; ++j){
								for(int k = y - 3; k < y + 3; ++k){
									if(j == x && k == y || k < 0 || k >= map.size || j < 0 || j >= map.size)
										continue;
									double w[] = {x,y};
									double d[] = {j,k};
									map.cost[j][k] = (byte) Math.max(255.0 - (inverse_cost_decay * LinAlg.distance(w,d)), map.cost[j][k]);
								}
							}
							*/

							//System.out.println((wall_point[0]/map.scale)+ ", " +(int)(wall_point[1]/map.scale));
							/*for(int k = 0; k < map.size; ++k){ //calculate costs
								for(int j = 0; j < map.size; ++j){

									//use if marginalizing
									if(k == (int) (wall_point[0] / map.scale) && j == (int) (wall_point[1]/map.scale))	
										map.cost[k][j] = 255; //highest cost
									else
									{
										int kj[] = {k, j};
										int wp[] = {(int) (wall_point[0]/map.scale), (int)(wall_point[1]/map.scale)};
										//map.cost[k][j] = Math.max(map.cost[k][j], Math.min(255, (int) (255.0/(LinAlg.distance(kj,wp)*inverse_cost_decay)))); //inverse distance decay
									}
									map.max = 255;
									

									//use if covariance-ing
									/*
									double x_minus_mu[][] = {{j/map.scale - wall_point[0]},{k/map.scale - wall_point[1]}};
									double cov[][] = {{bot_status.cov[0][0],bot_status.cov[0][1]},{bot_status.cov[1][0],bot_status.cov[1][1]}};

									//Calculate cost
									//cost = 1 / decay * (x-u)*E^-1*(x-u)^T
									double[][] chi_squared = LinAlg.multiplyMany(x_minus_mu, LinAlg.inverse(cov),LinAlg.transpose(x_minus_mu));
									int cost = (int) (1.0/(inverse_cost_decay *chi_squared[0][0]));
									map.cost[j][k] = Math.max(map.cost[j][k], cost);
									
									map.max = Math.max(map.max, cost);
									*/
									/*
								}
							}
							*/
						}
					}
				}

				

				lcm.publish("6_MAP",map);
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public void marginalize(bot_status_t new_bot_status)
	{
		dist_traveled += LinAlg.distance(new_bot_status.xyt, bot_status.xyt, 2);
		int decay = 0;
		while(dist_traveled < decay_dist){
			decay += decay_rate;
			dist_traveled -= decay_dist;
		}

		for(int i = 0; i < map.size; ++i){
			for(int j = 0; j < map.size; ++j){
				map.cost[i][j] = (byte) Math.max((int) map.cost[i][j] - decay, 0);
				double xy[] = {i*map.scale, j*map.scale};
				//map.knowledge[i][j] = (int) Math.max(map.knowledge[i][j], 255.0/(LinAlg.distance(new_bot_status.xyt, xy, 2)*inverse_knowledge_dist));
			}
		}
	}

	public static void main(String[] args)
	{
		MapBuilder mb = null;
		if(args[1] == "-l") {
			mb = new MapBuilder(true);
		} else {
			mb = new MapBuilder(false);
		}

        while(true)
        {
        	try{
            	Thread.sleep(1000);
            }catch(InterruptedException e){

            }
        }


	}


}
