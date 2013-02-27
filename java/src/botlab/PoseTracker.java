package botlab;

import java.io.*;
import java.util.*;

import lcm.lcm.*;
import botlab.lcmtypes.*;
import april.jmat.*;

/**
 * Subscribes to pose_t and find the pose_t whose timestamp is closest
 * to the requested value.
 **/
public class PoseTracker implements LCMSubscriber
{
    static String           channel     = "6_POSE";
    LCM                     lcm         = LCM.getSingleton();

    LinkedList<bot_status_t>      queue       = new LinkedList<bot_status_t>();

    // how long back in time should we remember poses?

    public double           time        = 10.0;

    boolean                 warned;

    // don't use messages that are older than this... (seconds)
    public double           maxTimeErr  = 3;

    static PoseTracker pt;
    static final int MAX_QUEUE_SIZE     = 10000;

    public static PoseTracker getSingleton()
    {
    
        if (pt == null)
            pt = new PoseTracker(channel, 10);

        return pt;
    }

    public PoseTracker(String channel, double time)
    {
        this.channel = channel;
        this.time = time;

        lcm.subscribe(channel, this);
    }

    public synchronized void clear()
    {
        queue.clear();
    }

    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            bot_status_t p = new bot_status_t(ins);
            queue.add(p);

            // emergency shrinkage.
            while (queue.size() > MAX_QUEUE_SIZE) {
                queue.removeFirst();
                if (!warned) {
                    System.out.println("PoseTracker queue too large");
                    warned = true;
                }
            }

            while (true) {
                bot_status_t first = queue.getFirst();
                bot_status_t last = queue.getLast();

                if (Math.abs(last.utime - first.utime) > time*1000000)
                    queue.removeFirst();
                else
                    break;
            }

        } catch (IOException ex) {
            System.out.println("Exception: " + ex);
        }
    }

    public synchronized bot_status_t get()
    {
        if (queue.size() == 0)
            return null;
        return queue.getLast();
    }

    public synchronized bot_status_t get(long utime)
    {
        bot_status_t p0 = null, p1 = null; // two poses bracketing the desired utime

        for (bot_status_t p : queue) {
            if (p.utime < utime && (p0 == null || p.utime > p0.utime))
                p0 = p;
            if (p.utime > utime && (p1 == null || p.utime < p1.utime))
                p1 = p;
        }

        if (p0 != null && Math.abs(utime - p0.utime) > maxTimeErr*1000000)
            p0 = null;

        if (p1 != null && Math.abs(utime - p1.utime) > maxTimeErr*1000000)
            p1 = null;

        if (p0 != null && p1 != null) {

            if (p0.utime == p1.utime)
                return p0;

            // interpolate
            double err0 = Math.abs(p0.utime - utime);
            double err1 = Math.abs(p1.utime - utime);

            double w0 = err1 / (err0 + err1);
            double w1 = err0 / (err0 + err1);

            assert(!Double.isNaN(w0));

            bot_status_t p = new bot_status_t();
            p.utime = utime;
            p.xyt = LinAlg.add(LinAlg.scale(p0.xyt, w0),
                               LinAlg.scale(p1.xyt, w1));
            p.xyt_dot = LinAlg.add(LinAlg.scale(p0.xyt_dot, w0),
                               LinAlg.scale(p1.xyt_dot, w1));
	    p.cov = LinAlg.add(LinAlg.scale(p0.cov, w0),
				LinAlg.scale(p1.cov, w1));
            return p;
        }

        if (p0 != null)
            return p0;

        if (p1 != null)
            return p1;

        return null;
    }
}


