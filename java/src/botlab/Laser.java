package botlab;

import orc.DigitalOutput;
import orc.Orc;

public class Laser
{

	Orc orc;
	DigitalOutput laser;

	Laser(){
		orc = Orc.makeOrc();
		laser = new DigitalOutput(orc, 0);
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
			System.out.println("Shooting...");
			laser.shoot();
			Thread.sleep(1000);
		    } catch (InterruptedException ex) {
		    }
		}
	}

}
