import ilog.concert.*;
import ilog.cplex.*;
import java.util.Arrays;
import java.math.BigDecimal;
import java.math.RoundingMode;

public class TSP_2 
{
	public static void main(String[] args)
	{
		modelsolve(10);
	}
	
	public static int findIndex(int arr[], int t) 
    { 
        int index = Arrays.binarySearch(arr, t); 
        return (index < 0) ? -1 : index; 
    }
	
	public static void modelsolve(int n) 
	{
		//Random data
		double[] xloc = new double[n];
		double[] yloc = new double[n];
		for(int i=0; i<n; i++) 
		{
			BigDecimal xl = new BigDecimal(Math.random()*100);
	        BigDecimal xls = xl.setScale(2, RoundingMode.HALF_UP);
			xloc[i]=xls.doubleValue();
			BigDecimal yl = new BigDecimal(Math.random()*100);
	        BigDecimal yls = yl.setScale(2, RoundingMode.HALF_UP);
			yloc[i]=yls.doubleValue();
		}
		
		double[][] c = new double[n][n];
		for(int i=0; i<n; i++) 
		{
			for(int j=0; j<n; j++) 
			{
				c[i][j] = Math.sqrt(Math.pow(xloc[i]-xloc[j], 2)+Math.pow(yloc[i]-yloc[j], 2));
			}
		}
		
		//Model
		try {
			IloCplex cplex = new IloCplex();
			
			//Variables
			IloNumVar[][] x = new IloNumVar[n][];
			for(int i=0; i<n; i++) 
			{
				x[i] = cplex.boolVarArray(n);
			}
			IloNumVar[] u = cplex.numVarArray(n, 0, Double.MAX_VALUE);
			
			//Objective
			IloLinearNumExpr obj = cplex.linearNumExpr();
			for(int i=0; i<n; i++) 
			{
				for(int j=0; j<n; j++) 
				{
					if(i != j) 
					{
						obj.addTerm(c[i][j], x[i][j]);
					}
				}
			}
			cplex.addMinimize(obj);
			
			//Constraints
			for(int j=0; j<n; j++) 
			{
				IloLinearNumExpr expr = cplex.linearNumExpr();
				for(int i=0; i<n; i++) 
				{
					if(i != j) 
					{
						expr.addTerm(1.0, x[i][j]);
					}
				}
				cplex.addEq(expr, 1.0);
			}
			for(int i=0; i<n; i++) 
			{
				IloLinearNumExpr expr = cplex.linearNumExpr();
				for(int j=0; j<n; j++) 
				{
					if(j != i) 
					{
						expr.addTerm(1.0, x[i][j]);
					}
				}
				cplex.addEq(expr, 1.0);
			}
			
			for(int i=1; i<n; i++) 
			{
				for(int j=1; j<n; j++) 
				{
					if(i != j) 
					{
						IloLinearNumExpr expr = cplex.linearNumExpr();
						expr.addTerm(1.0, u[i]);
						expr.addTerm(-1.0, u[j]);
						expr.addTerm(n-1, x[i][j]);
						cplex.addLe(expr, n-2);
					}
				}
			}
			
			//Solve Model
			cplex.solve();
			
			//Results
			
			//Location
			System.out.println("\nPlace\txloc\tyloc");
			for(int i=0; i<n; i++) 
			{
				System.out.println((i+1)+"\t"+xloc[i]+"\t"+yloc[i]);
			}
			
			//Distance Matrix
			System.out.println("\nDistance Matrix");
			for(int i=0; i<n; i++) 
			{
				for(int j=0; j<n; j++) 
				{
					if(i != j) 
					{
						System.out.printf("\t%.2f",c[i][j]);
					}
					else 
					{
						System.out.print("\t-");
					}
				}
				System.out.println();
			}
			
			//X Matrix
			System.out.println("\nX Matrix");
			for(int i=0; i<n; i++) 
			{
				for(int j=0; j<n; j++) 
				{
					if(i != j) 
					{
						System.out.print("\t" + Math.abs(cplex.getValue(x[i][j])));
					}
					else {
						System.out.print("\t-");
					}
				}
				System.out.println();
			}
			
			//Route
			int[] rx = new int[n];
			int[] ry = new int[n];
			System.out.println("\nRoute");
			for(int i=0; i<n; i++) 
			{
				for(int j=0; j<n; j++) 
				{
					if(i != j) 
					{
						if(cplex.getValue(x[i][j])>0.1) 
						{
							System.out.println(i+"\t"+j);
							rx[i]=i;
							ry[i]=j;
						}
					}
				}
			}
			
			System.out.println("\nOrdered Route");
			int[] R = new int[n+1];
			R[0]=rx[0];
			R[1]=ry[0];
			int m;
			for(int i=0; i<n-1; i++)
			{
				m=findIndex(rx,R[i+1]);
				R[i+2]=ry[m];
			}
			for(int i=0; i<R.length; i++)
				System.out.println(R[i]);
						
			//End
			cplex.close();
			
		}
		catch(IloException exc) 
		{
			exc.printStackTrace();
		}
	}
}
