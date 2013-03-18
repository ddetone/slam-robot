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

public class PathPlanner implements LCMSubscriber
{

	LCM lcm;
	map_t map = null;
	int travel_cost_map[][];
	bot_status_t openLoopBot = null;
	xyt_t status = null;
	double[] slamBot = null;
	xyt_t goal = null;
	xyt_t lastPlannedWaypoint = null;

	final boolean verbose = true;

	PathPlanner()
	{
		try{
			this.lcm = new LCM("udpm://239.255.76.67:7667?ttl=1");
		}catch(IOException e){
			this.lcm = LCM.getSingleton();
		}
		status = new xyt_t();
		lcm.subscribe("6_POSE",this);
		lcm.subscribe("6_MAP",this);
		lcm.subscribe("6_GOAL",this);
		lcm.subscribe("6_SLAM_POSES",this);
	}

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_MAP"))
			{
				map = new map_t(dins);
				//TODO: switch to only run when near waypoint or waypoint's cost is too high
				if(verbose)System.out.println("starting map");

				if(map != null && status != null && goal != null && slamBot != null && openLoopBot != null){
					if(verbose)System.out.println("found everything");
				
					if(lastPlannedWaypoint == null || LinAlg.distance(status.xyt, lastPlannedWaypoint.xyt, 2) < 0.2 || 
								(map.cost[(int) (lastPlannedWaypoint.xyt[0]/map.scale)+map.size/2][ (int) (lastPlannedWaypoint.xyt[1]/map.scale)+map.size/2] & 0xFF) > 0.6 * map.max)
					{
						if(verbose)System.out.println("attempting A Star");
						if(aStar(true)){
							if(verbose)System.out.println("A Start finished finding next waypoint");
							xyt_t waypoint = nextWaypoint(true);
							if(verbose)System.out.println("publishing");
							lcm.publish("6_WAYPOINT",waypoint);
							/*astar_view_t view = new astar_view_t();
							view.size = map.size;
							view.scale = map.scale;
							view.cost = travel_cost_map;
							lcm.publish("6_ASTAR",view);*/
						} else {
							if(verbose)System.out.println("No possible path to goal, trying to get close");
							aStar(true);
							xyt_t waypoint = nextWaypoint(false);
							lcm.publish("6_WAYPOINT",waypoint);/*
							astar_view_t view = new astar_view_t();
							view.size = map.size;
							view.scale = map.scale;
							view.cost = travel_cost_map;
							lcm.publish("6_ASTAR",view);*/
						}
					}
				}
			}
			else if(channel.equals("6_POSE"))
			{
				openLoopBot = new bot_status_t(dins);
			}
			else if(channel.equals("6_SLAM_POSES")){
				if(map != null){
					slam_vector_t temp = new slam_vector_t(dins);
					slamBot = LinAlg.copy(temp.xyt[temp.numPoses - 1].xyt);
					status.xyt[0] = temp.xyt[temp.numPoses-1].xyt[0] + (map.size/2)*map.scale;
					status.xyt[1] = temp.xyt[temp.numPoses-1].xyt[1] + (map.size/2)*map.scale;
				}
			}
			else if(channel.equals("6_GOAL"))
			{
				if(verbose)System.out.println("goal");
				if(map != null){
					goal = new xyt_t(dins);
					goal.xyt[0] += (map.size/2)*map.scale;
					goal.xyt[1] += (map.size/2)*map.scale;
				}
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public boolean aStar(boolean plan_through_walls)
	{
		ArrayList<MapNode> closed_set = new ArrayList<MapNode>();
		PriorityQueue<MapNode> open_set = new PriorityQueue<MapNode>();
		travel_cost_map = new int[map.size][map.size];

		for(int i = 0; i < map.size; ++i){
			for(int j = 0; j < map.size; ++j){
				travel_cost_map[i][j] = Integer.MAX_VALUE;
			}
		}
		
		open_set.add(new MapNode((int)(goal.xyt[0]/map.scale),(int)(goal.xyt[1]/map.scale),this));
		travel_cost_map[(int)(goal.xyt[0]/map.scale)][(int)(goal.xyt[1]/map.scale)] = 0;

		while(open_set.size() > 0)
		{
			MapNode current = open_set.poll();
			closed_set.add(current);
			//System.out.println(LinAlg.distance(new double[]{current.x,current.y}, new double[]{status.xyt[0]/map.scale, status.xyt[1]/map.scale}));

			if(current.x == (int) (status.xyt[0]/map.scale) && current.y == (int) (status.xyt[1]/map.scale)){
				return true;
			}
			
			for(MapNode neighbor : current.neighbors())
			{
				if(neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= map.size || neighbor.y >= map.size){
					if(verbose)System.out.println("planning to edge of map:"+neighbor.x+","+neighbor.y);
					continue;
				}

				int tentative_g_score = travel_cost_map[current.x][current.y] + 9;// + (map.cost[neighbor.x][neighbor.y] & 0xFF);
				if(plan_through_walls && (map.cost[neighbor.x][neighbor.y] & 0xFF) > 0.6 * map.max)
					tentative_g_score = travel_cost_map[current.x][current.y] + 9 + (map.cost[neighbor.x][neighbor.y] & 0xFF);

				boolean in_closed_set = false;
				for(MapNode compare : closed_set) {
					if(compare.x == neighbor.x && compare.y == neighbor.y){
						in_closed_set = true;
						break;
					}
				}
				if(in_closed_set) 
					if(tentative_g_score >= travel_cost_map[neighbor.x][neighbor.y])
						continue;

				boolean in_open_set = false;
				for(MapNode compare : open_set) {
					if(compare.x == neighbor.x && compare.y == neighbor.y){
						in_open_set = true;
						break;
					}
				}
				if(!in_open_set || tentative_g_score < travel_cost_map[neighbor.x][neighbor.y]){
					travel_cost_map[neighbor.x][neighbor.y] = tentative_g_score;
					if(!in_open_set)
						open_set.add(neighbor);
				}
			}
		}
		return false;
	}

	public xyt_t nextWaypoint(boolean planThroughWalls)
	{
		MapNode start = new MapNode((int) (status.xyt[0]/map.scale),(int)(status.xyt[1]/map.scale),this);
		MapNode current = start;
		MapNode minNeighbor = null;
		MapNode secMinNeighbor = null;

		//if changed, change in map builder
		//double knowledge_dist = 0.4/map.scale;
		double knowledge_dist = 0.3/map.scale;

		//plan long path
		for(int i = 0; i < knowledge_dist; ++i) {
			if(verbose)System.out.println(LinAlg.distance(new double[]{current.x,current.y}, new double[]{status.xyt[0]/map.scale, status.xyt[1]/map.scale}) + " xy:" +current.x + ","+ current.y+" cost:"+travel_cost_map[current.x][current.y]);
			//find lowest cost neighbor to crrent
			minNeighbor = null;
			secMinNeighbor = null;
			double min_dist_to_start = 0;
			for(MapNode neighbor : current.neighbors()){
				double distance_to_start = LinAlg.squaredDistance(new double[]{neighbor.x,neighbor.y}, new double[]{goal.xyt[0]/map.scale,goal.xyt[1]/map.scale});
				if(minNeighbor == null || travel_cost_map[neighbor.x][neighbor.y] < travel_cost_map[minNeighbor.x][minNeighbor.y] || travel_cost_map[neighbor.x][neighbor.y] == travel_cost_map[minNeighbor.x][minNeighbor.y] && distance_to_start < min_dist_to_start){
					minNeighbor = neighbor;
					min_dist_to_start = distance_to_start;
				}
			}

			if(minNeighbor == null)
				break;
			//raycast path
			double[] nextPoint = new double[]{minNeighbor.x, minNeighbor.y};
			double[] startPoint = new double[]{start.x,start.y};
			double dist = LinAlg.distance(nextPoint, startPoint);
			boolean intoWall = false;
			for(int j = 0; j < 2*dist+1; ++j){
				int[] rpos = new int[2];
				rpos[0] = (int)(nextPoint[0] * j/(2*dist) + startPoint[0]* (1- (j/(2*dist))));
				rpos[1] = (int)(nextPoint[1] * j/(2*dist) + startPoint[1]* (1- (j/(2*dist))));
				if(((map.cost[rpos[0]][rpos[1]] & 0xFF) > 0.6 * map.max) && !(((map.cost[start.x][start.y] & 0xFF) > 0.6 * map.max))){
					intoWall = true;
					break;
				}
				if(travel_cost_map[rpos[0]][rpos[1]] == Integer.MAX_VALUE){
					intoWall = true; //sanity check
					break;
				}
			}
			//if path goes into a wall or if lowest cost neighbor has a higher cost than this node
			if((intoWall || travel_cost_map[minNeighbor.x][minNeighbor.y] > travel_cost_map[current.x][current.y] && (current != start))){
				break;
			} else {
				current = minNeighbor;
			}
			//find the lowest neightbor to the lowest neighbor (for angles)
			for(MapNode neighbor : minNeighbor.neighbors()){
				if(secMinNeighbor == null || travel_cost_map[neighbor.x][neighbor.y] < travel_cost_map[secMinNeighbor.x][secMinNeighbor.y]){
					secMinNeighbor = neighbor;
				}
			}
		}

		//get the slam coord x, y of the goal
		xyt_t ret = new xyt_t();
		ret.xyt[0] = current.x * map.scale - (map.size/2)*map.scale;
		ret.xyt[1] = current.y * map.scale - (map.size/2)*map.scale;
	
		//if !goal find the angle of the second min neighbor
		if(travel_cost_map[current.x][current.y] < travel_cost_map[start.x][start.y] && secMinNeighbor != null){ //normal
			
			ret.xyt[2] = Math.atan2((goal.xyt[1] - (map.size/2)*map.scale) - ret.xyt[1], (goal.xyt[0] - (map.size/2)*map.scale) - ret.xyt[0]);
			/*
			if(secMinNeighbor.x < current.x - .01)
				ret.xyt[2] = Math.PI;
			else if(secMinNeighbor.x > current.x + .01)
				ret.xyt[2] = 0.0;
			else if(secMinNeighbor.y < current.y - .01)
				ret.xyt[2] = 3.0* Math.PI / 2.0;
			else 
				ret.xyt[2] = Math.PI/2.0;
			*/
		} else { //we're at the goal
			ret.xyt[2] = goal.xyt[2];
		}
		ret.xyt = LinAlg.xytMultiply(openLoopBot.xyt, LinAlg.xytInvMul31(slamBot, ret.xyt));
		return ret;
	}

	public static void main(String[] args) throws Exception
	{
		PathPlanner pp = new PathPlanner();

		while(true)
		{
			Thread.sleep(1000);
		}

	}


}
