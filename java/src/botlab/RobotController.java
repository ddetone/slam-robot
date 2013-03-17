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

	int pastNumTriangles;
	LinkedList<Integer> trianglesToKill;

	RobotController()
	{
		this.lcm =  LCM.getSingleton();
		lcm.subscribe("6_MAP",this);
		tracker = botlab.PoseTracker.getSingleton();
		map = null;
		state = "explore";
		pastNumTriangles = 0;
		trianglesToKill = new LinkedList<Integer>();
	}

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_MAP")){
				map = new map_t(dins);
				/*for(int i = pastNumTriangles; i < map.numTriangles; ++i) {
					trianglesToKill.add(new Integer(i));
				}
				if(trianglesToKill.size() > 0){
					state = "Shoot";
					//do shooting stuff
				}*/
			}

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
				HashMap<Integer, Double> mean_x = new HashMap<Integer, Double>();
				HashMap<Integer, Double> mean_y = new HashMap<Integer, Double>();
				for(int i = 1; i < map.size-1; ++i){
					for(int j = 1; j < map.size-1; ++j){
						if(knowledge_bounds[i][j]) {
							int representative = edge_sets.getRepresentative(i*map.size+j);
							if(counter.containsKey(representative)){
								Integer old_count = counter.get(representative);
								counter.put(representative, new Integer(old_count + 1));

								Double old_x = mean_x.get(representative);
								mean_x.put(representative, new Double(old_x + i));

								Double old_y = mean_y.get(representative);
								mean_y.put(representative, new Double(old_y + j));
							} else {
								counter.put(representative, new Integer(1));
								mean_x.put(representative, new Double(i));
								mean_y.put(representative, new Double(j));
							}
						}
					}
				}
				int max_size = 0;
				double goal_x = 0;
				double goal_y = 0;
				for(Integer i : counter.keySet()){
					if(counter.get(i) > max_size){
						max_size = counter.get(i);
						goal_x = mean_x.get(i)*map.scale/counter.get(i) - (map.size/2)*map.scale;
						goal_y = mean_y.get(i)*map.scale/counter.get(i) - (map.size/2)*map.scale;
					}
				}

				if(max_size < 8){ //map explored return to start
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
