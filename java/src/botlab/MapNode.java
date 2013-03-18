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

public class MapNode implements Comparable
{

	int x;
	int y;
	PathPlanner pp = null;

    MapNode(int nx, int ny, PathPlanner npp)
    {
        x = nx;
        y = ny;
        pp = npp;
    }

	public int compareTo(Object o)
	{
		MapNode n = (MapNode) o;
		return (cost() + heuristic()) - (n.cost() + n.heuristic());
	}

	public int heuristic()
	{
		double xy[] = {x,y};
		double goalxy[] = {pp.status.xyt[0]/pp.map.scale,pp.status.xyt[1]/pp.map.scale};
		return (int) LinAlg.distance(xy, goalxy,2);
	}

	public int cost()
	{
		return pp.travel_cost_map[x][y];
	}

	public ArrayList<MapNode> neighbors()
	{
		ArrayList<MapNode> ret = new ArrayList<MapNode>();	
		
		if(x > 0)
			ret.add(new MapNode(x - 1, y, pp));
	
		if(y > 0)
			ret.add(new MapNode(x, y - 1, pp));
	
		if(x < pp.map.size-1)
			ret.add(new MapNode(x + 1, y, pp));
	
		if(y < pp.map.size-1)
			ret.add(new MapNode(x, y+1, pp));

		return ret;
	}

}
