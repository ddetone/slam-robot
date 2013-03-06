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
	static final double DEFAULT_INV_COST_DECAY = 1.0;

	double decay_dist = DEFAULT_DECAY_DIST;
	double decay_rate = DEFAULT_DECAY_RATE;
	double inverse_knowledge_dist = DEFAULT_INV_KNOWLEDGE_DIST;
	double inverse_cost_decay = DEFAULT_INV_COST_DECAY;


	LCM lcm;
	botlab.PoseTracker tracker;
	bot_status_t bot_status;
	map_t map;
	double dist_traveled;



    MapBuilder()
    {
        this.lcm =  LCM.getSingleton();
        map = new map_t();
        bot_status = null;

	tracker = botlab.PoseTracker.getSingleton();

        //lcm.subscribe("6_POSE",this);
        lcm.subscribe("6_PARAM",this);
        lcm.subscribe("6_FEATURES",this);
        map.scale = 0.06;
        map.size = (int) (10.0/map.scale);
        map.cost = new int[(int) map.size][(int) map.size];
        map.knowledge = new int[(int) map.size][(int) map.size];
        dist_traveled = 0.0;
    }

    public void clear() {
    	map.cost = new int[(int) map.size][(int) map.size];
    	map.max = 0;
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
			/*elseif(channel.equals("6_POSE"))
			{
				bot_status_t new_bot_status = new bot_status_t(dins);
				new_bot_status.xyt[0] += (map.size/2)*map.scale;
				new_bot_status.xyt[1] += (map.size/2)*map.scale;

				//marginalize(new_bot_status);

				bot_status = new_bot_status;
			}*/
			else if(channel.equals("6_FEATURES"))
			{
				map_features_t features = new map_features_t(dins);
				bot_status = tracker.get(features.utime);
				
				if(bot_status == null)
					return;
				this.clear();
				
				//bot_status = features.bot;
				//bot_status.xyt[0] += (map.size/2)*map.scale;
				//bot_status.xyt[1] += (map.size/2)*map.scale;


				for(int f = 0; f < features.nlineSegs; ++f){
					//System.out.println(f);
					double p1[] = new double[2];
					double p2[] = new double[2];
					double l1[] = {features.lineSegs[f][0],features.lineSegs[f][1],0};
					double l2[] = {features.lineSegs[f][2],features.lineSegs[f][3],0};

					p1 = LinAlg.xytMultiply(bot_status.xyt, l1);
					p2 = LinAlg.xytMultiply(bot_status.xyt, l2);
					/*
					p1[0] = bot_status.xyt[0] + features.lineSegs[f][0];
					p1[1] = bot_status.xyt[1] + features.lineSegs[f][1];

					p2[0] = bot_status.xyt[0] + features.lineSegs[f][2];
					p2[1] = bot_status.xyt[1] + features.lineSegs[f][3];
					*/

					//add points to map
					double dist = LinAlg.distance(p1,p2,2);
					int nsteps = (int) (dist/map.scale)+1;

					for(int i = 0; i <= nsteps; ++i){ //for each pixel between p1 and p2
						double wall_point[] = new double[2];
						wall_point[0] = p1[0]*(double)i/nsteps + p2[0]*(1.0-(double)i/nsteps);
						wall_point[1] = p1[1]*(double)i/nsteps + p2[1]*(1.0-(double)i/nsteps);

						//raycast to points
						double rdist = LinAlg.distance(bot_status.xyt,wall_point,2);
						int rsteps = (int) (rdist/map.scale)+1;
						int ray_pixel[] = new int[2];
						for(int r = 0; r < rsteps; ++r){
							ray_pixel[0] = (int) ((wall_point[0]*(double)i/nsteps + bot_status.xyt[0]*(1.0-(double)i/nsteps))*map.scale);
							ray_pixel[1] = (int) ((wall_point[1]*(double)i/nsteps + bot_status.xyt[1]*(1.0-(double)i/nsteps))*map.scale);
							map.cost[ray_pixel[0]][ray_pixel[1]] = (int) Math.min(map.cost[ray_pixel[0]][ray_pixel[1]],0);
							//map.knowledge[ray_pixel[0]][ray_pixel[1]] = 1;
						}
						

						//System.out.println((wall_point[0]/map.scale)+ ", " +(int)(wall_point[1]/map.scale));
						for(int k = 0; k < map.size; ++k){ //calculate costs
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
							}
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
				map.cost[i][j] = Math.max((int) map.cost[i][j] - decay, 0);
				double xy[] = {i*map.scale, j*map.scale};
				map.knowledge[i][j] = (int) Math.max(map.knowledge[i][j], 255.0/(LinAlg.distance(new_bot_status.xyt, xy, 2)*inverse_knowledge_dist));
			}
		}
	}

	public static void main(String[] args)
	{
		MapBuilder mb = new MapBuilder();

        while(true)
        {
        	try{
            	Thread.sleep(1000);
            }catch(InterruptedException e){

            }
        }


	}


}
