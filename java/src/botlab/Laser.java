package botlab;

import orc.DigitalOutput;
import orc.Orc;

import java.io.*;
import java.util.*;
import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import javax.swing.*;
import lcm.lcm.*;
import botlab.lcmtypes.*;
public class Laser implements LCMSubscriber
{

	Orc orc;
	DigitalOutput laser;
	
	LCM lcm;	
	
	Laser(){
		lcm = LCM.getSingleton();
		orc = Orc.makeOrc();
		laser = new DigitalOutput(orc, 0);
		lcm.subscribe("6_LASER", this);
	}
	
	public void messageReceived(LCM lcm, String channel, LCMDataInputStream dins)
	{
		try{
			if(channel.equals("6_LASER"))
			{
				laser_t laserlcm = new laser_t(dins);
				shoot(laserlcm.num);
			}
		}
		catch(IOException e){
			e.printStackTrace();
		}
	}

	public void turnOn(){
		laser.setValue(true);
	}

	public void turnOff(){
		laser.setValue(false);
	}

	public void shoot(){
		shoot(3);
	}

	public void shoot(int numTimes){
		try{
			for(int i = 0; i < numTimes; i++){
				turnOn();
				Thread.sleep(150);
				turnOff();
				Thread.sleep(150);
			}
		}catch(InterruptedException e){
			e.printStackTrace();
		}
	}

	public static void main(String args[]){
		Laser laser = new Laser();
		
		while (true) {
		    try {
			Thread.sleep(1000);
		    } catch (InterruptedException ex) {
		    }
		}
	}

}
