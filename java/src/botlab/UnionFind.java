package botlab;

import java.io.*;
import java.util.*;

public class UnionFind
{

	public int height;		//height of image
	public int width;		//width of image
	public int N;			//number of pixels
	public boolean[][] map;
	public int[] parent;
	public int[] rank;

	public UnionFind(int h, int w, boolean[][] m)	//constructor
	{
		height = h;
		width = w;
		N = h*w;
		map = m;
		
		parent = new int[N];	//initialize parent array
		rank = new int[N];		//initialize rank array
		for (int i=0; i<N; i++) 
		{	
			parent[i] = i;		//parent index is initially pixel number
			rank[i] = 0;
		}
	}
	
	public int Find(int x)
	{
		if (parent[x] != x)
			parent[x] = Find(parent[x]);
		return parent[x];
	}
	
	public void Union(int x, int y)
	{
		int xRoot = Find(x);
		int yRoot = Find(y);
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

	public int[] FindSets()
	{
		
		
		for (int y=0; y<height; y++)
		{
			for (int x=0; x<width; x++)
			{			
				if (map[y][x])
				{
					if (x<(width-1) && map[y][x+1]) //check right
						Union(width*y+x,width*y+(x+1));
					if (y<(height-1) && map[y+1][x]) //check below
						Union(width*y+x,width*(y+1)+x);
				}
			}
		}
		return parent;
	}
	
}
















