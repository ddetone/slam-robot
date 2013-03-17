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

public class RobotController implements LCMSubscriber
{

	LCM lcm;
	bot_status_t bot_status;
	botlab.PoseTracker tracker;
	map_t map;
	String state;
	xyt_t robotPose;

	int pastNumTriangles;
	LinkedList<Integer> trianglesToKill;

	RobotController()
	{
		try{
			this.lcm = new LCM("udpm://239.255.76.67:7667?ttl=1");
		}catch(IOException e){
			this.lcm = LCM.getSingleton();
		}
		tracker = botlab.PoseTracker.getSingleton();
		map = null;
		robotPose = null;
		state = "explore";
		pastNumTriangles = 0;
		trianglesToKill = new LinkedList<Integer>();
		lcm.subscribe("6_MAP",this);
		lcm.subscribe("6_SLAM_POSES",this);
	}
	
	public void planFrontier(){
		//frontier planner
		if(state == "explore"){
			boolean knowledge_bounds[][] = new boolean[(int) map.size][(int) map.size]; //is [y][x]
			for(int i = 1; i < map.size-1; ++i){
				for(int j = 1; j < map.size-1; ++j){
					knowledge_bounds[i][j] = false; //default
					if(map.knowledge[i][j] != 1){
						continue;
					}

					int neighbor_count = 0;
					if(map.knowledge[i-1][j-1] == 0) neighbor_count++;
					if(map.knowledge[i-1][j  ] == 0) neighbor_count++;
					if(map.knowledge[i-1][j+1] == 0) neighbor_count++;
					if(map.knowledge[i  ][j-1] == 0) neighbor_count++;

					if(map.knowledge[i  ][j+1] == 0) neighbor_count++;
					if(map.knowledge[i+1][j-1] == 0) neighbor_count++;
					if(map.knowledge[i+1][j  ] == 0) neighbor_count++;
					if(map.knowledge[i+1][j+1] == 0) neighbor_count++;

					if(neighbor_count > 0){ //use to set knowledge threshold
						knowledge_bounds[i][j] = true;
					}
				}
			}

			UnionFind edge_sets = new UnionFind(map.size*map.size);
			edge_sets.FindSets(knowledge_bounds, map.size, map.size);
			HashMap<Integer, Integer> counter = new HashMap<Integer, Integer>();
			HashMap<Integer, ArrayList<Double>> mean_x = new HashMap<Integer, ArrayList<Double>>();
			HashMap<Integer, ArrayList<Double>> mean_y = new HashMap<Integer, ArrayList<Double>>();
			for(int i = 1; i < map.size-1; ++i){
				for(int j = 1; j < map.size-1; ++j){
					if(knowledge_bounds[i][j]) {
						int representative = edge_sets.getRepresentative(i*map.size+j);
						if(counter.containsKey(representative)){
							Integer old_count = counter.get(representative);
							counter.put(representative, new Integer(old_count + 1));
							mean_x.get(representative).add(new Double(i));
							mean_y.get(representative).add(new Double(j));
						} else {
							counter.put(representative, new Integer(1));
							ArrayList<Double> xs = new ArrayList<Double>();
							xs.add(new Double(i));
							mean_x.put(representative, xs);
							ArrayList<Double> ys = new ArrayList<Double>();
							ys.add(new Double(j));
							mean_y.put(representative, ys);
						}
					}
				}
			}
			int max_size = 0;
			double goal_x = 0;
			double goal_y = 0;
			for(Integer i : counter.keySet()){
				int size_not_in_wall = 0;
				for(int j = 0; j < counter.get(i); ++j){
					if((int)(map.cost[mean_x.get(i).get(j).intValue()][mean_y.get(i).get(j).intValue()] & 255) < 255*0.6)
						size_not_in_wall++;
				}
				double lowest_distance = Double.MAX_VALUE;
				if(size_not_in_wall > max_size){
					max_size = size_not_in_wall;
					for(int j = 0; j < counter.get(i); ++j){
						if((int)(map.cost[mean_x.get(i).get(j).intValue()][mean_y.get(i).get(j).intValue()] & 255) < 255*0.6){
							double[] robotXY = new double[]{robotPose.xyt[0]/map.scale + map.size/2,robotPose.xyt[1]/map.scale + map.size/2};
							double dist = LinAlg.squaredDistance(robotXY, new double[]{mean_x.get(i).get(j), mean_y.get(i).get(j)});
							if(dist < lowest_distance){
								lowest_distance = dist;
								goal_x = mean_x.get(i).get(j)*map.scale - (map.size/2)*map.scale;
								goal_y = mean_y.get(i).get(j)*map.scale - (map.size/2)*map.scale;
							}
						}
					}
				}
			}

			if(max_size < 0.10/map.scale){ //map explored return to start
				goal_x = 0;
				goal_y = 0;
			}

			xyt_t goal = new xyt_t();
			goal.utime = TimeUtil.utime();
			double xyt[] = {goal_x, goal_y, 0.0};
			goal.xyt = xyt;
			lcm.publish("6_GOAL", goal);
		}	
	}
	
	
	
	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_MAP")){
				map = new map_t(dins);
				if(robotPose != null) planFrontier();
				/*for(int i = pastNumTriangles; i < map.numTriangles; ++i) {
					trianglesToKill.add(new Integer(i));
				}
				if(trianglesToKill.size() > 0){
					state = "Shoot";
					//do shooting stuff
				}*/
			} else if(channel.equals("6_SLAM_POSES")){
				slam_vector_t slam_vec = new slam_vector_t(dins);
				robotPose = slam_vec.xyt[slam_vec.numPoses - 1];
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public static void main(String[] args) throws Exception
	{
		RobotController rc = new RobotController();

		while(true)
		{
			Thread.sleep(1000);
		}
	}
}
