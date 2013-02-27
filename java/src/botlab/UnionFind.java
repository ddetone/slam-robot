package botlab;

import java.io.*;
import java.util.*;

public class UnionFind
{

	public int N;			//number of pixels
	public int[] parent;
	public int[] rank;

	public UnionFind(int N)	//constructor
	{
		parent = new int[N];	//initialize parent array
		rank = new int[N];		//initialize rank array
		for (int i=0; i<N; i++) 
		{	
			parent[i] = i;		//parent index is initially pixel number
			rank[i] = 0;
		}
	}
	
	public int getRepresentative(int x)
	{
		if (parent[x] != x)
			parent[x] = getRepresentative(parent[x]);
		return parent[x];
	}
	
	public void connectNodes(int x, int y)
	{
		int xRoot = getRepresentative(x);
		int yRoot = getRepresentative(y);
		if (xRoot == yRoot)
			return;
			
		if (rank[xRoot] < rank[yRoot])
			parent[xRoot] = yRoot;
		else if (rank[xRoot] > rank[yRoot])
			parent[yRoot] = xRoot;	
		else
		{
			parent[yRoot] = xRoot;	
			++rank[xRoot];	
		}
	}

	public int[] FindSets(boolean[][] map, int height, int width)
	{
		
		
		for (int y=0; y<height; y++)
		{
			for (int x=0; x<width; x++)
			{			
				if (map[y][x])
				{
					if (x<(width-1) && map[y][x+1]) //check right
						connectNodes(width*y+x,width*y+(x+1));
					if (y<(height-1) && map[y+1][x]) //check below
						connectNodes(width*y+x,width*(y+1)+x);
				}
			}
		}
		return parent;
	}
	
}
















