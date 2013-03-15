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

	double newFeatureDist = .7;
	double oldFeatureDist = 0.5;
	int num_solver_iterations = 200;
	int decimate = 1;	// number of poses to throw out. In other words, every <decimate> poses, 1 pose will be added to the graph
	int numPoseMessages = 0;
	int numFeatureMessages = 0;

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
			if((long)utimes.get(size - 1) <= _utime)return graphNodeIndices.get(size - 1);
			for(int i = size - 1; i > 0; i--){
				if((long)utimes.get(i - 1) < _utime){
					return graphNodeIndices.get(i);
				}
			}
			if(utimes.get(0) - _utime < 3 * 1e6){
				return graphNodeIndices.get(0);
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

	public static double mahalanobisDistance(double u[], double x[], double[][]W){
		double dx[] = LinAlg.subtract(x,u);
		Matrix w = new Matrix(W);
		return Math.sqrt(LinAlg.dotProduct(dx, w.times(dx)));
	}


	/** Assume that ge is a prediction of the relative position of two
	 * poses. What is the probability that the two poses are no more
	 * than 'range' apart? **/
	double mahalanobisDistance(GXYTEdge ge, double range)
	{
		// which nodes *might* be in the neighborhood?
		// Compute the mahalanobis distance that the nodes are
		// within 5 meters of each other.

		// how far away is the other node according to the dijkstra edge?
		double dist = Math.sqrt(ge.z[0]*ge.z[0] + ge.z[1]*ge.z[1]);

		// when we subtract our sensor range, what fraction of the distance do we have left?
		double dist_ratio;

		if (dist < range)
			dist_ratio = 0;
		else
			dist_ratio = (dist - range) / dist;

		// compute the positional error, accounting for our sensor range.
		double err[] = new double[] { ge.z[0] * dist_ratio, ge.z[1] * dist_ratio };

		// compute mahalanobis distance of err.
		double W[][] = ge.getW();

		// non invertible. Usually means that we're querying the
		// distance of node 'a' with respect to the same node 'a'.
		if (W == null) {
			if (dist < range)
				return 0;
			else
				return Double.MAX_VALUE;
		}

		return Math.sqrt(err[0]*err[0]*W[0][0] + 2 * err[0]*err[1]*W[0][1] + err[1]*err[1]*W[1][1]);
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
					System.out.println("added pose " + poseNodes.getNumNodes() + " to slam graph as node #" + g.nodes.size());
					
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
						// The above should no longer be true. Changed from lastPoseIndex
						// to currPoseIndex
						ge.nodes = new int[]{currPoseIndex};
						// should the covariance be 0? or close to 0?
						ge.z = new double[]{bot.xyt[0], bot.xyt[1], bot.xyt[2]};
						// This line is not technically correct. If we assume that
						// the first pose message we get will show the robot at 0,0,0.
						// However, since we will never be adding another POS edge,
						// we can be practically 100% sure about this one. And as always,
						// I just made all of that up. But it does sound plausible.
						ge.P = LinAlg.diag(new double[]{0.00001,0.00001,0.00001});
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
					double rot = ge.z[2];
					double rotErr = 0.1;
					// model the covariance in the direction that the robot is travelling
					ge.P = LinAlg.diag(new double[]{LinAlg.sq(dist*distErr+0.01),
									LinAlg.sq(dist*distErr+0.01),
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
				if(poseNode == -1){
					System.out.println("Discarding feature message. No corresponding slam node found.");
					return;
				}
				double[] botSlamPose = LinAlg.copy(g.nodes.get(poseNode).state);

				boolean needToSolve = false;
				for(int i = 0; i < features.ntriangles; i++){
					// The feature state is represented as (-z, x, theta_bot) to account
					// for coordinate frame transformation and orientation between image
					// processing and map making. Orientation is defined as if the feature
					// were looking where the robot was facing when it was observed. As
					// long as this convention remains consistent, it should be fine.
					//double[] featureState = new double[]{-features.triangles[i][1],
					//					features.triangles[i][0],
					//					bot.xyt[2]};
					// This has been changed in the image processing code. No need
					// to account for coordinate frame transformation here.
					// make the state of this node equal to the state of the last pose after
					// slam + the current feature observation. Notice the difference between this
					// line and 
					//double[] featureState = new double[]{features.triangles[i][0],
					//					features.triangles[i][1],
					//					bot.xyt[2]};
					double[] featureState = LinAlg.xytMultiply(botSlamPose,
									new double[]{	features.triangles[i][0],
											features.triangles[i][1],
											0});
					
					double dist, minDist = Double.MAX_VALUE;
					int closestFeatureNode = -1;
					for(int j = 0; j < featureNodes.getNumNodes(); j++){
						int closeFeatureIndex = featureNodes.getNodeGraphIndex(j);
						// so the following line is definitely wrong. I think my
						// fundamental understanding of mahalanobis distance was wrong.
						// For every feature you've seen so far, find the edge between
						// the feature you just observed and a prior feature with the
						// lowest uncertainty. The edge will be defined as a composition
						// of edges that connect the two features. To find the
						// composition with the lowest uncertainty, use a shortest path
						// algorithm over the edges of the graph, g. Maybe using A* and a
						// composition of edges as the cost heuristic. e3 = e1.compose(e2)
						// creates a single edge, e3, that embodies the uncertainties of
						// both edges e1, and e2. 
						// After you find that edge, calculating the mahalanobis distance
						// of that edge answers the following question: "What is the
						// probability that the nodes that the edge connects are within x
						// meters of each other?" Magic asked that question using 5 meters
						// as their 'x'. You then want to find the max mahalanobis distance
						// since you want to find the edge with the highest probability
						// that its nodes are within x meters. Not sure if that explanation
						// is correct.
						// In the mean time, the following effectively calculates the
						// euclidean distance. In fact, it just does calculate the euclidean
						// distance practically ignoring theta for now.
						
						dist = mahalanobisDistance(featureState, g.nodes.get(closeFeatureIndex).state, LinAlg.diag(new double[]{1,1,0.01}));
						if(dist < minDist){
							minDist = dist;
							closestFeatureNode = closeFeatureIndex;
						}
					}
					System.out.println("The distance between the current feature and the closest" + 
								" already observed feature, slam node #" + closestFeatureNode + " is " + minDist);
					if(minDist > newFeatureDist){
						// we add the new feature node to the graph and make an
						// edge between it and the pose from which it was observed.
						GXYTNode gna = new GXYTNode();
						gna.state = LinAlg.copy(featureState);
						gna.init = LinAlg.copy(gna.state);
						featureNodes.addNode(g.nodes.size(), features.utime);
						System.out.println("added feature " + featureNodes.getNumNodes() + " to slam graph as node #" + g.nodes.size());
						g.nodes.add(gna);
						// Add an edge between the feature and the pose from which
						// the feature was observed
						GXYTEdge ge = new GXYTEdge();
						ge.nodes = new int[]{poseNode, g.nodes.size() - 1};
						// ******* CONFIRM THIS IS THE CORRECT Z ******
						ge.z = new double[]{features.triangles[i][0],
								features.triangles[i][1],
								0};
						// Hopefully the model below works. Theta very uncertain because
						// we might not view the feature from straight on every time we see it.
						dist = Math.sqrt(ge.z[0] * ge.z[0] + ge.z[1] * ge.z[1]);
						double distErr = 0.2;
						ge.P = LinAlg.diag(new double[]{LinAlg.sq(dist*distErr+0.01),
										LinAlg.sq(dist*distErr+0.01),
										LinAlg.sq(Math.toRadians(90))});
						g.edges.add(ge);
					}
					else if (minDist < oldFeatureDist){
						GXYTEdge ge = new GXYTEdge();
						// When reobserving a feature, you add an edge from the old feature
						// node to the new pose node from which you reobserved the feature
						assert(closestFeatureNode != -1);
						ge.nodes = new int[]{poseNode, closestFeatureNode};
						System.out.println("added edge between pose " + poseNode + " and feature " + closestFeatureNode + " to slam graph");
						// However you still model the observation as it was taken. From
						// the new pose to the feature you saw from that pose. NOT from the
						// new pose to the state of the old feature. Careful of the subtle
						// difference. The wrong way to do it would look like this:
						// ge.z = LinAlg.xytInvMul31(bot.xyt, g.nodes.get(closestFeatureNode).state);
						// Again just made that up. Hope it's right.
						// ******* CONFIRM THIS IS THE CORRECT Z ******
						// confirmed
						ge.z = new double[]{features.triangles[i][0],
								features.triangles[i][1],
								0};
						// Model the reobservation uncertainty proportional to the square of the
						// distance from which the feature was observed.
						dist = Math.sqrt(ge.z[0] * ge.z[0] + ge.z[1] * ge.z[1]);
						double distErr = 0.2;
						ge.P = LinAlg.diag(new double[]{LinAlg.sq(dist*distErr+0.01),
										LinAlg.sq(dist*distErr+0.01),
										LinAlg.sq(Math.toRadians(90))});
						g.edges.add(ge);
						

						needToSolve = true;
					}
				}
				if(needToSolve){
					solve();
				}
				packageAndPublish();
			}
		}		catch (IOException e)
		{
			e.printStackTrace();
		}
	}

	public void solve(){
		CholeskySolver solver = new CholeskySolver(g);
		solver.verbose = false;
		int numIterations = 0;

		double chi2Before = g.getErrorStats().chi2normalized;

		for(int j = 0; j < num_solver_iterations; j++){
			solver.iterate();
			numIterations++;
			double chi2After = g.getErrorStats().chi2normalized;
			System.out.println("Iteration " + numIterations +
					" reduced chi2 by " + (chi2Before - chi2After));
			// if the further iteration is futile
			if (chi2After >= 0.9 * chi2Before || chi2After < 0.00001)
				break;
			chi2Before = chi2After;
		}
		
		System.out.println("Done fixing slam graph after " + numIterations + " iterations.");
	}

	public void packageAndPublish(){
		
		slam_vector_t pose_out = new slam_vector_t();
		int numPoses = poseNodes.getNumNodes();

		xyt_t[] pose_out_xyts = new xyt_t[numPoses];
		
		pose_out.numPoses = numPoses;

		for(int i = 0; i < numPoses; i++){

			pose_out_xyts[i] = new xyt_t();

			pose_out_xyts[i].xyt = g.nodes.get(poseNodes.getNodeGraphIndex(i)).state;

			pose_out_xyts[i].utime = poseNodes.utimes.get(i);

		}
		
		pose_out.xyt = pose_out_xyts;

		int numFeatures = featureNodes.getNumNodes();
		
		pose_out.numTriangles = numFeatures;
		pose_out.triangles = new double[numFeatures][2];
		
		for(int i = 0; i < numFeatures; i++){
			
			pose_out.triangles[i] = g.nodes.get(featureNodes.getNodeGraphIndex(i)).state;

		}
		
		lcm.publish("6_SLAM_POSES", pose_out);
		
	}


	public static void main(String args[]) throws Exception{
		new MapSLAM();
		while(true){
			Thread.sleep(1000);
		}
	}
	
}

















