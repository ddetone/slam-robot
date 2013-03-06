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
	map_t travel_cost_map = null;
	bot_status_t status = null;
	xyt_t goal = null;

    PathPlanner()
    {
        this.lcm =  LCM.getSingleton();
        lcm.subscribe("6_POSE",this);
    }

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_MAP"))
			{
				map = new map_t(dins);
			}
			if(channel.equals("6_POSE"))
			{
				status = new bot_status_t(dins);
				status.xyt[0] += (map.size/2)/map.scale;
				status.xyt[0] += (map.size/2)/map.scale;
			}
			if(channel.equals("6_GOAL"))
			{
				status = new bot_status_t(dins);
			}

			if(map != null && status != null && goal != null){
				xyt_t waypoint = nextWaypoint(aStar());
				lcm.publish("6_WAYPOINT",waypoint);
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public map_t aStar()
	{
		ArrayList<MapNode> closed_set = new ArrayList<MapNode>();
		PriorityQueue<MapNode> open_set = new PriorityQueue<MapNode>();
		travel_cost_map = new map_t();
		travel_cost_map.size = map.size;
		travel_cost_map.scale = map.scale;
		travel_cost_map.cost = new int[map.size][map.size];
		for(int i = 0; i < map.size; ++i){
			for(int j = 0; j < map.size; ++j){
				travel_cost_map.cost[i][j] = Integer.MAX_VALUE;
			}
		}
		
		open_set.add(new MapNode((int)(goal.xyt[0]/map.scale),(int)(goal.xyt[1]/map.scale),this));

		while(open_set.size() > 0)
		{
			MapNode current = open_set.poll();
			closed_set.add(current);
			if(current.x == status.xyt[0]/travel_cost_map.scale && current.y == status.xyt[1]/travel_cost_map.scale){
				return travel_cost_map;
			}
			
			
			for(MapNode neighbor : current.neighbors())
			{
				if(map.cost[neighbor.x][neighbor.y] > 0.6 * map.max)
					continue;
				int tentative_g_score = current.cost() + map.max/(map.size*map.size) + map.cost[neighbor.x][neighbor.y];

				boolean in_closed_set = false;
				for(MapNode compare : closed_set) {
					if(compare.x == neighbor.x && compare.y == neighbor.y){
						in_closed_set = true;
						break;
					}
				}
				if(in_closed_set) 
					if(tentative_g_score >= neighbor.cost())
						continue;

				boolean in_open_set = false;
				for(MapNode compare : open_set) {
					if(compare.x == neighbor.x && compare.y == neighbor.y){
						in_open_set = true;
						break;
					}
				}
				if(!in_open_set || tentative_g_score < neighbor.cost()){
					travel_cost_map.cost[neighbor.x][neighbor.y] = tentative_g_score;
					if(!in_open_set)
						open_set.add(neighbor);
				}
			}
		}
		return null;
	}

	public xyt_t nextWaypoint(map_t travel_cost_map)
	{
		MapNode start = new MapNode((int) (status.xyt[0]/travel_cost_map.scale),(int)(status.xyt[1]/travel_cost_map.scale),this);
		MapNode minNeighbor = null;
		for(MapNode neighbor : start.neighbors()){
			if(minNeighbor == null || neighbor.cost() < minNeighbor.cost()){
				minNeighbor = neighbor;
			}
		}
		MapNode secMinNeighbor = null;
		for(MapNode neighbor : minNeighbor.neighbors()){
			if(secMinNeighbor == null || neighbor.cost() < secMinNeighbor.cost()){
				secMinNeighbor = neighbor;
			}
		}
		xyt_t ret = new xyt_t();
		ret.xyt[0] = minNeighbor.x * travel_cost_map.scale;
		ret.xyt[1] = minNeighbor.y * travel_cost_map.scale;
		if(secMinNeighbor.x < minNeighbor.x - .01)
			ret.xyt[2] = Math.PI;
		else if(secMinNeighbor.x > minNeighbor.x + .01)
			ret.xyt[2] = 0.0;
		else if(secMinNeighbor.y < minNeighbor.y - .01)
			ret.xyt[2] = 3.0* Math.PI / 2.0;
		else 
			ret.xyt[2] = Math.PI/2.0;
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
