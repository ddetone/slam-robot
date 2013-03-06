package botlab;

import java.util.*;
import java.io.*;

import april.graph.*;
import lcm.lcm.*;
import april.jmat.*;
import april.util.*;
import java.io.*;
import botlab.PoseTracker;

import botlab.lcmtypes.*;

public class MapSLAM implements LCMSubscriber
{

	
	LCM lcm;
	Graph g;
	botlab.PoseTracker pt;
	GraphNodes poseNodes;
	GraphNodes featureNodes;
	bot_status_t lastBot = new bot_status_t();

	double newFeatureDist = 0.3;
	double oldFeatureDist = 0.1;
	int num_solver_iterations = 200;
	int decimate = 1;	// number of poses to throw out. In other words, every <decimate> poses, 1 pose will be added to the graph
	int numPoseMessages = 0;

	private class GraphNodes{
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
			int size = utimes.size();
			for(int i = 0; i < size; i++){
				if(utimes.get(i) == _utime)return graphNodeIndices.get(i);
			}
			return -1;
		}

		public int getNumNodes(){
			return utimes.size();
		}

		public int getNodeGraphIndex(int local_index){
			try{
				return graphNodeIndices.get(local_index);
			}catch(IndexOutOfBoundsException e){
				e.printStackTrace();
				return -1;
			}
		}

		public int getLastNodeIndex(){
			if(graphNodeIndices.size() > 0){
				return getNodeGraphIndex(graphNodeIndices.size() - 1);
			}
			return -1;
		}
		
	}

	MapSLAM(){
		g = new Graph();
		lcm = LCM.getSingleton();
		pt = botlab.PoseTracker.getSingleton();
		poseNodes = new GraphNodes();
		featureNodes = new GraphNodes();

		lcm.subscribe("6_FEATURES",this);
		lcm.subscribe("6_POSE",this);
	}

	//public static double mahalanobisDistance(double u[], double x[], double[][]W){
	public static double mahalanobisDistance(double u[], double x[]){
		double dx[] = LinAlg.subtract(x,u);
		//return Math.sqrt(LinAlg.dotProduct(dx, LinAlg.multiply(W, dx));
		return Math.sqrt(LinAlg.dotProduct(dx, dx));
	}
	

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try
		{
			if(channel.equals("6_POSE")){
				if(numPoseMessages % decimate == 0){
					bot_status_t bot = new bot_status_t(dins);

					int lastPoseIndex = poseNodes.getLastNodeIndex(); //equivalent to g.nodes.size()
					poseNodes.addNode(g.nodes.size(), bot.utime);
					int currPoseIndex = poseNodes.getLastNodeIndex(); //equivalent to g.nodes.size()
					
					// If this is the first pose we are receiving add the node to the graph
					// and add a POS edge to the graph to constrain the SLAM solution to the real world
					// POS edges are like gps edges. They constrain the map to the real world. We only
					// need one of these for the very first pose the robot sees. This is the definition
					// of the origin
					if(poseNodes.getNumNodes() == 1){
						// create new node and add it to the graph
						GXYTNode gna = new GXYTNode();
						gna.state = new double[]{bot.xyt[0], bot.xyt[1], bot.xyt[2]};
						gna.init = LinAlg.copy(gna.state);
						g.nodes.add(gna);
						
					// pin whole map down to starting point if on first pose.
					// Not sure if the below is correct and it shouldn't
					// affect the graph with relation to itself so I am
					// leaving it out until after testing the rest of
					// the code
					// ****adding the following back in to see if it works****
					//*
						GXYTPosEdge ge = new GXYTPosEdge();
 						//For this line lastPoseIndex should always be 0.
						//****might not be zero if we JUST HAPPEN to get a features
						// message before a single pose message****
						ge.nodes = new int[]{lastPoseIndex};
						// should the covariance be 0? or close to 0?
						ge.z = new double[]{bot.xyt[0], bot.xyt[1], bot.xyt[2]};
						// This line is not technically correct. If we assume that
						// the first pose message we get will show the robot at 0,0,0.
						// However, since we will never be adding another POS edge,
						// we can be practically 100% sure about this one. And as always,
						// I just made all of that up. But it does sound plausible.
						ge.P = LinAlg.diag(new double[]{0.0001,0.0001,0.0001});
						g.edges.add(ge);
					//*/
						
						return;
					}
					
					// create edge between last two poses characterized by covariance P
					GXYTEdge ge = new GXYTEdge();
					ge.nodes = new int[]{lastPoseIndex, currPoseIndex};
					ge.z = LinAlg.xytInvMul31(lastBot.xyt, bot.xyt);
					double dist = Math.sqrt(ge.z[0] * ge.z[0] + ge.z[1] * ge.z[1]);
					double distErr = 0.1;
					double rot = Math.abs(MathUtil.mod2pi(ge.z[2]));
					double rotErr = 0.1;
					ge.P = LinAlg.diag(new double[]{LinAlg.sq(dist*distErr+0.001),
									LinAlg.sq(dist*distErr+0.001),
									LinAlg.sq(rot*rotErr)+Math.toRadians(1)});
					g.edges.add(ge);

					GXYTNode gn = new GXYTNode();
					// make the state of this node equal to the state of the last node after
					// slam + the current POSE observation. Notice the difference between this
					// line and gn.state = new double[]{bot.xyt[0], bot.xyt[1], bot.xyt[2]};
					gn.state = LinAlg.xytMultiply(g.nodes.get(lastPoseIndex).state, ge.z);
					gn.init = LinAlg.copy(gn.state);
					g.nodes.add(gn);
					
					
					lastBot = bot;
					assert(g.nodes.size() == featureNodes.getNumNodes() + poseNodes.getNumNodes());
					System.out.println("added pose " + poseNodes.getNumNodes() + " to slam graph");
				}
				numPoseMessages++;
			}
			else if(channel.equals("6_FEATURES"))
			{
				map_features_t features = new map_features_t(dins);
				// get the pose that corresponds with this feature message. Synced off of utime
				bot_status_t bot = pt.get(features.utime);

				// If there is no pose to match with the current observation, then
				// the features in the message are useless to us. This function should
				// really only return here if we are not receiving pose messages.
				if(bot == null){
					System.out.println("Discarding feature message. No corresponding pose found.");
					return;
				}
				
				
				// find the node index in the slam graph that corresponds with
				// the the pose that pose tracker returns
				int poseNode = poseNodes.findNode(bot.utime);
				if(poseNode == -1)return;
				
				for(int i = 0; i < features.ntriangles; i++){
					// The feature state is represented as (-z, x, theta_bot) to account
					// for coordinate frame transformation and orientation.
					// Orientation is defined as if the feature were looking where
					// the robot was facing when it was observed. As long this 
					// convention remains consistent, it should be fine.
					double[] featureState = new double[]{-features.triangles[i][1],
										features.triangles[i][0],
										bot.xyt[2]};
					
					double dist, minDist = Double.MAX_VALUE;
					int closestFeatureNode = -1;
					for(int j = 0; j < featureNodes.getNumNodes(); j++){
						int closeFeatureIndex = featureNodes.getNodeGraphIndex(j);
						dist = mahalanobisDistance(featureState, g.nodes.get(closeFeatureIndex).state);
						if(dist < minDist){
							dist = minDist;
							closestFeatureNode = closeFeatureIndex;
						}
					}
					if(minDist > newFeatureDist){
						// we add the new feature node to the graph and make an
						// edge between it and the pose from which it was observed.
						GXYTNode gna = new GXYTNode();
						gna.state = featureState;
						gna.init = LinAlg.copy(gna.state);
						featureNodes.addNode(g.nodes.size(), features.utime);
						g.nodes.add(gna);
						// add an edge between the feature and the pose from which
						// the feature was observed
						GXYTEdge ge = new GXYTEdge();
						ge.nodes = new int[]{poseNode, g.nodes.size() - 1};
						ge.z = LinAlg.xytInvMul31(bot.xyt, featureState);
						// Hopefully the model below works. Theta very uncertain because
						// we might not view the feature from straight on every time we see it.
						ge.P = LinAlg.diag(new double[]{ge.z[0] * ge.z[0], ge.z[1] * ge.z[1], 100});
						g.edges.add(ge);
					}
					else if (minDist < oldFeatureDist){
						GXYTEdge ge = new GXYTEdge();
						// When reobserving a feature, you add an edge from the old feature
						// node to the new pose node from which you reobserved the feature
						assert(closestFeatureNode != -1);
						ge.nodes = new int[]{poseNode, closestFeatureNode};
						// However you still model the observation as it was taken. From
						// the new pose to the feature you saw from that pose. NOT from the
						// new pose to the state of the old feature. Careful of the subtle
						// difference. The wrong way to do it would look like this:
						// ge.z = LinAlg.xytInvMul31(bot.xyt, g.nodes.get(closestFeatureNode).xyt);
						ge.z = LinAlg.xytInvMul31(bot.xyt, featureState);
						// Model the reobservation uncertainty proportional to the square of the
						// distance from which the feature was observed.
						ge.P = LinAlg.diag(new double[]{ge.z[0] * ge.z[0], ge.z[1] * ge.z[1], 100});
						g.edges.add(ge);
						
						CholeskySolver solver = new CholeskySolver(g);
						solver.verbose = false;
						for(int j = 0; j < num_solver_iterations; j++)
							solver.iterate();
						
						//packageAndPublish();
					}

				}
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public static void main(String args[]) throws Exception{
		new MapSLAM();
		while(true){
			Thread.sleep(1000);
		}
	}
	
}

















