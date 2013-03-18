package botlab.util;

import java.util.*;

public class PidController 
{
	double Kp;
	double Ki;
	double Kd;

	//double error;
	double prevError; //static

	double integral; //static

	double prevUtime;

	PidController(double Kp, double Ki, double Kd)
	{
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		prevUtime = 0;
		prevError = 0;
		integral = 0;
	}
	
	void resetController()
	{
		prevUtime = 0;
		prevError = 0;
		integral = 0;
	}

	double getOutput(double error)
	{
		double currUtime = TimeUtil.utime();

		//for first time the dt should be 0
		double dt;
		if(prevUtime == 0)
			dt = 0
		else
			dt = currUtime - prevUtime;

		integral += error*dt;
		
		double derivative = (error - prevError)/dt;

		double output = Kp * error + Ki * integral + Kd * derivative;

		prevError = error;
		prevUtime = currUtime();

		return;
	}

	
}


