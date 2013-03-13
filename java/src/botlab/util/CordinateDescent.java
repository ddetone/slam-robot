package botlab.util;

public abstract class CordinateDescent
{
	int n;
	double[] p;
	double[] dp;
	double tolerance;

	public CordinateDescent(double[] _p, double[] _dp, double _tolerance)
	{
		if(!(_p.length == _dp.length))
			System.out.println("Warning param and dparam vectors not of equal length. Will take length of param");
		
		n = _p.length;
		p = new double[n];
		dp = new double[n];


		//deep copy, other methods exist
		for(int i = 0; i < n; i++)
		{
			p[i]  = _p[i];
			dp[i] = _dp[i];
		}

		tolerance = _tolerance;

	}

	public abstract double error(double[] parameters);

	public double[] getBestParams()
	{
		double bestErr = error(p);

		while(sum(dp) > tolerance)
		{
			for(int i = 0; i < n; i++)
			{
				p[i] += dp[i];
				double err = error(p);

				if(err < bestErr)
				{
					bestErr = err;
					dp[i] *= 1.1;
				}
				else
				{
					p[i] -= 2*dp[i];
					err = error(p);
					if(err < bestErr)
					{
						bestErr = err;
						dp[i] *= 1.1;						
					}
					else
					{
						p[i] += dp[i];
						dp[i] *= 0.9;
					}
				}
			}
		}
        return(p);
	}

	private double sum(double[] dp)
	{
		double s = 0;
		for(int i = 0; i < n; i++)
		{
			s += dp[i];
		}
		return s;
	}


}
