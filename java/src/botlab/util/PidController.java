package botlab.util;

import java.util.*;
import april.util.*;
import april.jmat.*;

public class PidController
{
	double Kp;
	double Ki;
	double Kd;

	//double error;
	double prevError; //static

	public double integral; //static

	double prevUtime;

    boolean isClampIntegrator;
    double clamp;

	public PidController(double Kp, double Ki, double Kd)
	{
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		prevUtime = 0;
		prevError = 0;
		integral = 0;
	}

	public void resetController()
	{
		prevUtime = 0;
		prevError = 0;
		integral = 0;
	}

    public void setIntegratorClamp(double clamp)
    {
        this.clamp = clamp;
        isClampIntegrator = true;
    }

	public double getOutput(double error)
	{
		double currUtime = TimeUtil.utime();

		//for first time the dt should be 0
		double dt;
		if(prevUtime == 0)
			dt = 0;
		else
			dt = currUtime - prevUtime;

		//integral += error*dt;
		integral += error;

        if(isClampIntegrator){
            integral = LinAlg.clamp(integral, -clamp, clamp);
        }

		//double derivative = (error - prevError)/dt;
		double derivative = (error - prevError);

		double output = Kp * error + Ki * integral + Kd * derivative;

		prevError = error;
		prevUtime = currUtime;

		return(output);
	}


}


