package botlab;

import orc.DigitalOutput;
import orc.Orc;

public class Laser
{
	DigitalOutput laser;
	Orc orc;
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

	public static void main(String args[]){
		
		Laser laser = new Laser();

		while (true) {
		    try {
			laser.turnOn();
			System.out.println("true");
			Thread.sleep(1000);
			laser.turnOff();
			System.out.println("false");
			Thread.sleep(1000);
		    } catch (InterruptedException ex) {
		    }
		}
	}

}
