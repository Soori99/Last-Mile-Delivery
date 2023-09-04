import java.io.FileInputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.poi.ss.usermodel.Sheet;
import org.apache.poi.ss.usermodel.Workbook;
import org.apache.poi.ss.usermodel.WorkbookFactory;

import ilog.concert.*;
import ilog.cplex.*;

public class VRP_1 
{
	public static int findIndex(int arr[], int t) 
    { 
  
        int index = Arrays.binarySearch(arr, t); 
        return (index < 0) ? -1 : index; 
    }
	
	public static void file(String fp) throws Exception 
	{
		//Excel Distance matrix Input
		FileInputStream fis = new FileInputStream(fp);
		Workbook wb = WorkbookFactory.create(fis);
		Sheet sh = wb.getSheet("Sheet3");
		int n = sh.getLastRowNum();

		String[] loc = new String[n];
		double[][] c = new double[n][n];
		for(int i=0; i<n; i++) 
		{
			for(int j=0; j<n; j++) 
			{
				if(i==0&&j>=0)
					loc[j]=String.valueOf(sh.getRow(i).getCell(j+1));
				if(i != j) 
				{
					c[i][j] = Double.valueOf(String.valueOf(sh.getRow(i+1).getCell(j+1)));
				}
				else
					c[i][j] = 0.0;
			}
		}
		
		//Excel Demand input
		Sheet sh1 = wb.getSheet("Sheet2");
		int nr = sh1.getLastRowNum();
		double[] dem = new double[nr];
		ArrayList<String> til=new ArrayList<String>();
		int nc=0;
		for(int i=0; i<100; i++) 
		{
			if(sh1.getRow(0).getCell(i)==null) 
			{
				break;
			}
			else 
			{
				nc=nc+1;
			}
		}
		
		for(int i=0; i<nc; i++) 
		{
			String[] temp = new String[nr];
			til.add(String.valueOf(sh1.getRow(0).getCell(i)));
			for(int j=0; j<nr; j++) 
			{
				temp[j]= String.valueOf(sh1.getRow(j+1).getCell(i));
				if(til.get(i).equals("Demand")) 
				{
					dem[j]=Double.parseDouble(temp[j]);
				}
			}
		}
		
		//Vehicle capacity
		double Q = 60;
		
		//Number of vehicles
		int k = 3;
		
		//Model
		try 
		{
			IloCplex cplex = new IloCplex();
			
			//Variables
			IloNumVar[][] x = new IloNumVar[n][];
			for(int i=0; i<n; i++) 
			{
				x[i] = cplex.boolVarArray(n);
			}
			
			IloNumVar[][] f = new IloNumVar[n][];
			for(int i=0; i<n; i++) 
			{
				f[i]=cplex.numVarArray(n, 0, Q);
			}
			
			//Objective
			IloLinearNumExpr obj = cplex.linearNumExpr();
			for(int i=0; i<n; i++) 
			{
				for(int j=0; j<n; j++) 
				{
					if(i != j) {
						obj.addTerm(c[i][j], x[i][j]);
					}
				}
			}
			cplex.addMinimize(obj);
			
			//Constraints
			for(int j=1; j<n; j++) 
			{
				IloLinearNumExpr expr1 = cplex.linearNumExpr();
				for(int i=0; i<n; i++) 
				{
					if(i != j) 
					{
						expr1.addTerm(1.0, x[i][j]);
					}
				}
				cplex.addEq(expr1, 1.0);
			}
			
			for(int i=1; i<n; i++) 
			{
				IloLinearNumExpr expr2 = cplex.linearNumExpr();
				for(int j=0; j<n; j++) 
				{
					if(j != i) 
					{
						expr2.addTerm(1.0, x[i][j]);
					}
				}
				cplex.addEq(expr2, 1.0);
			}

			for(int i=1; i<n; i++) 
			{
				IloLinearNumExpr expr3 = cplex.linearNumExpr();
				for(int j=0; j<n; j++) 
				{
					if(i != j) 
					{
						expr3.addTerm(1.0, f[i][j]);
						expr3.addTerm(-1.0, f[j][i]);
					}
				}
				cplex.addEq(expr3, dem[i]);
			}

			for(int i=0; i<n; i++) 
			{
				for(int j=0; j<n; j++) 
				{
					IloLinearNumExpr expr4 = cplex.linearNumExpr();
					expr4.addTerm(Q, x[i][j]);
					cplex.addLe(f[i][j], expr4);
				}
			}
			
			//Solve
			cplex.solve();

			//Results
			
			//Objective
			System.out.println("\nObjective value : "+cplex.getObjValue());
			
			//System.out.println(cplex.getCplexStatus());
			System.out.println(cplex.getStatus());
			
			//Vehicle Capacity
			System.out.println("\nVehicle Capacity : " + Q);
			
			//X Matrix
			System.out.println("\nX Matrix");
			for(int i=0; i<n; i++) 
			{
				for(int j=0; j<n; j++) 
				{
					if(i != j) 
					{
						System.out.printf("\t%.1f",Math.abs(cplex.getValue(x[i][j])));
					}
					else 
					{
						System.out.print("\t-");
					}
				}
				System.out.println();
			}
			
			//F Matrix
			System.out.println("\nF Matrix");
			for(int i=0; i<n; i++) 
			{
				for(int j=0; j<n; j++) 
				{
					if(i != j) 
					{
						System.out.printf("\t%.1f",Math.abs(cplex.getValue(f[i][j])));
					}
					else 
					{
						System.out.print("\t-");
					}
				}
				System.out.println();
			}
			
			//Route
			List<Integer> rx = new ArrayList<Integer>();
			List<Integer> ry = new ArrayList<Integer>();
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
							rx.add(i);
							ry.add(j);
						}
					}
				}
			}
			
			//Number of routes
			System.out.println("\nNumber of Routes : " + k);
			
			//Ordered routes
			System.out.println("\nOrdered Route");
			for(int l=0; l<k; l++) 
			{
				System.out.println("\nRoute : "+(l+1));
				List<Integer> R = new ArrayList<Integer>();
				R.add(rx.get(l));
				R.add(ry.get(l));
				int m=1;
				for(int i=0; i<rx.size(); i++) 
				{
					if(ry.get(m) != 0) 
					{
						m=rx.indexOf(R.get(i+1));
						R.add(ry.get(m));
					}
				}
				
				for(int i=0; i<R.size(); i++)
					System.out.println(R.get(i)+" - "+loc[R.get(i)]);
			}
			
			//End
			cplex.close();
		}
		catch(IloException exc) 
		{
			exc.printStackTrace();
		}
	}
}
