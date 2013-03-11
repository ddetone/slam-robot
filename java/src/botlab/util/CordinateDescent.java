package botlab.util;

public class CordinateDescent()
{
	int n;
	double[] p;
	double[] dp;

	public CordinateDescent(double[] _p, double[] _dp)
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

	}

	public double error(double[] parameters);

	public double[] getBestParams()
	{
		double bestErr = error(p);

		for(int i = 0; i < n; i++)
		{
			p[i] += dp[i];
			double err = error(p);

			if(err < bestErr){
				bestErr = error;
				dp[i] *= 1.1;
			}

		}
	}


}
