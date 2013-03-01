package botlab;

import april.graph.*;
import lcm.lcm.*;
import april.jmat.*;
import april.util.*;
import java.io.*;
import botlab.PoseTracker;

import botlab.lcmtypes.*;

public class MapSLAM implements LCMSubscriber
{


	private class GraphPoses{
		ArrayList<Integer> graphNodeIndices;
		ArrayList<Long> utimes;
		
		GraphNodes(){
			graphNodeIndices = new ArrayList<Integer>();
			utimes = new ArrayList<Long>();
		}

		public void addNode(int _nodeIndex, long _utime)
		{
			graphNodeIndices.add(new Integer(_nodeIndex));
			utimes.add(new Long(_utime));
		}

		// Check if array list has an optimized search routine.
		// Don't have internet to check
		public int findNode(long _utime){
			int size = utimes.size()
			for(int i = 0; i < size; i++){
				if(utimes.get(i) == _utime)return new int(graphNodeIndices.get(i));
			}
			return null;
		}

		public int getNumNodes(){
			return utimes.size();
		}

		public int getNodeGraphIndex(int local_index){
			return new int(graphNodeIndices.get(local_index));
		}
	}
	
	LCM lcm;
	Graph g;
	botlab.PoseTracker pt;
	GraphNodes poseNodes;
	GraphNodes featureNodes;
	bot_status_t lastBot = new bot_status_t();

	MapMaker(boolean showGUI){
		g = new Graph();
		lcm = LCM.getSingleton();
		pt = botlab.PoseTracker.getSingleton();
		poseNodes = new GraphPoses();
		featureNodes = new GraphPoses();

		lcm.subscribe("6_FEATURES",this);
		lcm.subscribe("6_POSE",this);
	}


	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_POSE"){
				bot_status_t bot = new bot_status_t(dins);
				
				
				poseNodes.addNode(g.nodes.length, bot.utime);
				int numPoses = poseNodes.getNumNodes();
				
				if(numPoses == 1){
					// create new node and add it to the graph
					GXYTNode gna = new GXYTNode();
					gna.state = new double[]{bot.xyt[0], bot.xyt[1], bot.xyt[2]};
					gna.init = LinAlg.copy(gna.state);
					g.nodes.add(gna);
					
					// pin whole map down to starting point if on first pose
					//GXYTPosEdge ge = new GXYTPosEdge();
					
					return;
				}
				
				// create edge between last two poses characterized by covariance P
				GXYTEdge ge = new GXYTEdge();
				ge.nodes = new int[]{poseNodes.getNodeGraphIndex(numPoses - 2), poseNodes.getNodeGraphIndex(numPoses - 1)};
				ge.z = LinAlg.xytInvMul31(lastBot.xyt, bot.xyt);
				double xDist = Math.abs(ge.z[0]);
				ge.P = LinAlg.diag(new double[]{xDist * 0.1,
								xDist * 0.01,
								xDist * 0.05});
				g.edges.add(ge);

				GXYTNode gn = new GXYTNode();
				gn.state = LinAlg.xytMultiply(g.nodes.get(numPoses - 1).state, ge.z);
				gn.init = LinAlg.copy(gn.state);
				g.nodes.add(gn);
				
				
				lastBot = bot;
			}
			else if(channel.equals("6_FEATURES"))
			{
				map_features_t features = new map_features_t(dins);
				bot_status_t bot = pt.get(features.utime);

				if(bot == null) return;

				for(int i = 0; i < features.ntriangles; i++){
					// find the pose that corresponds with the utime
					// of the features message in our list of poses

					int poseNode = poseNodes.findNode(bot.utime);
				
					double dist = minMahalanobisDist(features.triangles[i]);
					if((dist > newFeatureDist) || (dist < oldFeatureDist)){
						// we add the new feature node to the graph and make an
						// edge between it and the pose from which it was observed.
						GXYTNode gna = new GXYTNode();
						// the feature is added to the graph as (-z,x,theta+180) to account
						// for coordinate frame transformation and orientation
						while (bot.xyt[2] < -Math.PI * 2)bot.xyt[2] += Math.PI * 2;
						while (bot.xyt[2] > Math.PI * 2)bot.xyt[2] -= Math.PI * 2;
						gna.state = new double[]{-features.triangles[i][1],
									features.triangles[i][0],
									MathUtil.mod2Pi(bot.xyt[2] + Math.PI)};
						gna.init = LinAlg.copy(gna.state);
						g.nodes.add(gna);
					}

					// if first observation
					if(dist > newFeatureDist){
						GXYTEdge ge = new GXYTEdge();
						ge.nodes = new int[]{poseNode, g.nodes.length - 1};
						
					}
					else if (dist < oldFeatureDist){
					}
				}
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}
	
}

















