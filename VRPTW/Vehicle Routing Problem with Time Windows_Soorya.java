import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Scanner;
import ilog.concert.*;
import ilog.cplex.*;

class Data
{
	int vertex_num;
	double ET;
	double LT;
	int vehicle_num;
	double vehicle_capacity;
	int vertices [][];
	int demands [];
	int vehicles [];
	double a[];
	double b[];
	double s[];
	int arcs [][];
	double dist [][];
	double gap = 1e-6;
	  
	public double truncate(double v)
	{
		int iv = (int) v;
		if (iv+1 - v <= 0.000000000001)
			return iv+1;
		double dv = (v-iv)*10;
		int idv = (int) dv;
		double rv = iv + idv/10.0;
		return rv;
	}
}

class Solution 
{
	double epsilon = 0.0001;
	Data data = new Data();
	ArrayList<ArrayList<Integer>> routes = new ArrayList<>();
	ArrayList<ArrayList<Double>> servicetimes = new ArrayList<>();
	
	public Solution(Data data, ArrayList<ArrayList<Integer>> routes, ArrayList<ArrayList<Double>> servicetimes)
	{
		super();
		this.data = data;
		this.routes = routes;
		this.servicetimes = servicetimes;
	}

	public int compare(double v1, double v2)
	{
		if (v1< v2-epsilon)
		{
			return -1;
		}
		if (v1> v2+epsilon)
		{
			return 1;
		}
		return 0;
	}
	
	public void feasible() throws IloException
	{
		if (routes.size()>data.vehicle_num)
		{
			System.out.println("Error: Number of routes greater than Number of available vehicles");
			System.exit(0);
		}
		
		for(int k=0; k<routes.size();k++)
		{
			ArrayList<Integer> route = routes.get(k);
			double capacity = 0;
			for (int i= 0; i<route.size(); i++)
			{
				capacity = capacity + data.demands[route.get(i)];
			}
			if (capacity > data.vehicle_capacity)
			{
				System.out.println("Error: Capacity greater than Vehicle Capacity");
				System.exit(0);
			}	
		}
		for(int k=0; k<routes.size();k++)
		{
			ArrayList<Integer> route = routes.get(k);
			ArrayList<Double> servicetime = servicetimes.get(k);
			double capacity = 0;
			for (int i=0; i<route.size()-1;i++)
			{
				int origin = route.get(i);
				int destination = route.get(i+1);
				double si = servicetime.get(i);
				double sj = servicetime.get(i+1);
				if (si<data.a[origin]&& si>data.b[origin])
				{
					System.out.println("Error: Servicetime outside time window");
					System.exit(0);
				}
				if (compare(si + data.dist[origin][destination], data.b[destination])>0)
				{
					System.out.println(origin + ": [" + data.a[origin] + "," + data.b[origin]+ "]"+ " "+ si);
					System.out.println(destination + ": [" + data.a[destination] + "," + data.b[destination] + "]" + " " + sj);
					System.out.println(data.dist[origin][destination]);
					System.out.println(destination + ":");
					System.out.println("Error: Forward Servicetime");
					System.exit(0);			
				}
				if (compare(sj - data.dist[origin][destination], data.a[origin])<0)
				{
					System.out.println(origin + ": [" + data.a[origin] + "," + data.b[origin]+ "]"+ " "+ si);
					System.out.println(destination + ": [" + data.a[destination] + "," + data.b[destination] + "]" + " " + sj);
					System.out.println(data.dist[origin][destination]);
					System.out.println(destination + ":");
					System.out.println("Error: Bakcward Servicetime");
					System.exit(0);			
				}
			}
			if (capacity>data.vehicle_capacity)
			{
				System.out.println("Error: Load greater than vehicle capacity");
				System.exit(0);
			}
		}
	}
}

public class VRPTW
{
	Data data;
	IloCplex model;
	public IloNumVar x[][][];
	public IloNumVar t[][];
	double cost;
	Solution solution;
	
	public VRPTW(Data data)
	{
		super();
		this.data = data;
	}
	
	public void solve() throws IloException
	{
		ArrayList<ArrayList<Integer>> routes = new ArrayList<>();
		ArrayList<ArrayList<Double>> servicetimes = new ArrayList<>();
		
		for (int k=0; k<data.vehicle_num; k++)
		{
			ArrayList<Integer> r = new ArrayList<>();
			ArrayList<Double> t = new ArrayList<>();
			routes.add(r);
			servicetimes.add(t);	
		}
		
		if (model.solve() == false)
		{
			System.out.println("Problem cannot be solved");
			return;
		}
		
		else
		{
			for (int k=0; k<data.vehicle_num;k++)
			{
				int i=0;
				boolean terminate = true;
				routes.get(k).add(0);
				servicetimes.get(k).add(0.0);
				while (terminate)
				{
					for (int j=0; j<data.vertex_num; j++)
					{
						if (data.arcs[i][j]>=0.5 && model.getValue(x[i][j][k])>=0.5)
						{
							routes.get(k).add(j);
							servicetimes.get(k).add(model.getValue(t[j][k]));
							i=j;
							break;
						}
					}
					if (i == data.vertex_num-1)
					{
						terminate = false;
					}
				}
			}
		}
		solution = new Solution(data,routes,servicetimes);
		cost = model.getObjValue();
		System.out.println("routes="+solution.routes);
	}

	private void build_model() throws IloException
	{
		//Model
		model = new IloCplex();
		model.setOut(null);
	
		//Variables
		x = new IloNumVar[data.vertex_num][data.vertex_num][data.vehicle_num];
		t = new IloNumVar[data.vertex_num][data.vehicle_num];
	
		for (int i=0; i<data.vertex_num;i++)
		{
			for ( int k=0;k<data.vehicle_num;k++)
			{
				t[i][k] = model.numVar(0,1e15,IloNumVarType.Float,"w"+i+","+k);
			}
			for (int j=0; j<data.vertex_num;j++)
			{
				for (int k=0; k<data.vehicle_num;k++)
				{
					x[i][j][k] = model.numVar(0, 1,IloNumVarType.Int,"x"+i+","+j+","+k);
				}
			}
		}
	
		//Objective
		IloNumExpr obj = model.numExpr();
		for (int i=0; i<data.vertex_num;i++)
		{
			for (int j=0; j<data.vertex_num;j++)
			{
				if (data.arcs[i][j]==0)
				{
					continue;
				}
				for (int k=0; k<data.vehicle_num;k++)
				{
					obj = model.sum(obj,model.prod(data.dist[i][j], x[i][j][k]));
				}
			}
		}
		model.addMinimize (obj);
	
		//Constraints
		//Constraint 1
		for (int i=1;i<data.vertex_num-1;i++)
		{
			IloNumExpr expr1 = model.numExpr();
			for (int k=0; k<data.vehicle_num;k++)
			{
				for(int j=1; j<data.vertex_num;j++)
				{
					if(data.arcs[i][j]==1)
					{
						expr1 = model.sum(expr1,x[i][j][k]);
					}
				}
			}
			model.addEq(expr1,1);
		}
	
		//Constraint 2
		for (int k=0; k<data.vehicle_num;k++)
		{
			IloNumExpr expr2 = model.numExpr();
			for (int j=1; j<data.vertex_num;j++)
			{
				if(data.arcs[0][j]==1)
				{
					expr2 = model.sum(expr2,x[0][j][k]);
				}
			}
			model.addEq(expr2, 1);
		}

		//Constraint 3
		for (int k=0; k<data.vehicle_num;k++)
		{
			for (int j=1; j<data.vertex_num-1;j++)
			{
				IloNumExpr expr3 = model.numExpr();
				IloNumExpr subExpr1 = model.numExpr();
				IloNumExpr subExpr2 = model.numExpr();
				for (int i=0; i<data.vertex_num; i++)
				{
					if (data.arcs[i][j]==1)
					{
						subExpr1 = model.sum(subExpr1,x[i][j][k]);
					}
					if (data.arcs[j][i]==1)
					{
						subExpr2 = model.sum(subExpr2,x[j][i][k]);
					}
				}
				expr3 = model.sum(subExpr1,model.prod(-1,subExpr2));
				model.addEq(expr3, 0);
			}
		}
	
		//Constraint 4
		for (int k=0; k<data.vehicle_num;k++)
		{
			IloNumExpr expr4 = model.numExpr();
			for (int i=0; i< data.vertex_num-1;i++)
			{
				if(data.arcs[i][data.vertex_num-1]==1)
				{
					expr4 = model.sum(expr4,x[i][data.vertex_num-1][k]);				
				}
			}
			model.addEq(expr4, 1);
		}
	
		//Constraint 5
		double M = 1e5;
		for (int k=0; k<data.vehicle_num;k++)
		{
			for (int i=0; i<data.vertex_num;i++)
			{
				for (int j=0; j<data.vertex_num; j++)
				{
					if (data.arcs[i][j]==1)
					{
						IloNumExpr expr5 = model.numExpr();
						IloNumExpr expr6 = model.numExpr();
						expr5 = model.sum(t[i][k],data.s[i]+data.dist[i][j]);
						expr5 = model.sum(expr5, model.prod(-1, t[j][k]));
						expr6 = model.prod(M, model.sum(1,model.prod(-1, x[i][j][k])));
						model.addLe(expr5, expr6);
					}
				}
			}
		}
	
		//Constraint 6
		for (int k=0; k<data.vehicle_num;k++)
		{
			for (int i=1; i<data.vertex_num-1; i++)
			{
				IloNumExpr expr7 = model.numExpr();
				for (int j=0; j<data.vertex_num; j++)
				{
					if (data.arcs[i][j]==1)
					{
						expr7 = model.sum(expr7,x[i][j][k]);
					}
				}
				model.addLe(model.prod(data.a[i], expr7), t[i][k]);
				model.addLe(t[i][k], model.prod(data.b[i], expr7));
			}
		}	
	
		//Constraint 7
		for (int k=0; k<data.vehicle_num; k++)
		{
			model.addLe(data.ET,  t[0][k]);
			model.addLe(data.ET, t[data.vertex_num-1][k]);
			model.addLe(t[0][k], data.LT);
			model.addLe(t[data.vertex_num-1][k],data.LT);
		}
	
		//Constraint 8
		for(int k =0; k<data.vehicle_num;k++)
		{
			IloNumExpr expr8 = model.numExpr();
			for (int i=1; i<data.vertex_num-1;i++)
			{
				IloNumExpr expr9 = model.numExpr();
				for (int j=0; j<data.vertex_num;j++)
				{
					if(data.arcs[i][j]==1)
					{
						expr9 = model.sum(expr9, x[i][j][k]);
					}
				}
				expr8 = model.sum(expr8, model.prod(data.demands[i], expr9));
			}
			model.addLe(expr8, data.vehicle_capacity);
		}	
	}
	
	public static void process_file(String s, Data data, int vertex_num) throws Exception
	{
		String line = null;
		String substr [] = null;
		Scanner cin = new Scanner(new BufferedReader(new FileReader(s)));
		for (int i=0; i<4;i++)
		{
			line = cin.nextLine();
		}
		line = cin.nextLine();
		line.trim();
		substr = line.split(("\\s+"));
		data.vertex_num = vertex_num;
		data.vehicle_num = Integer.parseInt(substr[1]);
		data.vehicle_capacity = Integer.parseInt(substr[2]);
		data.vertices = new int[data.vertex_num][2];
		data.demands = new int[data.vertex_num];
		data.vehicles = new int[data.vertex_num];
		data.a = new double[data.vertex_num];
		data.b = new double[data.vertex_num];
		data.s = new double[data.vertex_num];
		data.arcs = new int[data.vertex_num][data.vertex_num];
		data.dist = new double[data.vertex_num][data.vertex_num];
		
		for (int i=0;i<4;i++)
		{
			line = cin.nextLine();
		}
		
		for(int i=0; i<data.vertex_num-1;i++)
		{
			line=cin.nextLine();
			line.trim();
			substr = line.split("\\s+");
			data.vertices[i][0] = Integer.parseInt(substr[2]);
			data.vertices[i][1] = Integer.parseInt(substr[3]);
			data.demands[i] = Integer.parseInt(substr[4]);
			data.a[i] = Integer.parseInt(substr[5]);
			data.b[i] = Integer.parseInt(substr[6]);
			data.s[i] = Integer.parseInt(substr[7]);
		}
		cin.close();
		data.vertices[data.vertex_num-1]= data.vertices[0];
		data.demands[data.vertex_num-1]=0;
		data.a[data.vertex_num-1] = data.a[0];
		data.b[data.vertex_num-1] = data.b[0];
		data.ET = data.a[0];
		data.LT = data.b[0];
		data.s[data.vertex_num-1] = 0;
		double min1 = 1e15;
		double min2 = 1e15;
		
		for(int i=0; i<data.vertex_num;i++)
		{
			for (int j=0;j< data.vertex_num;j++)
			{
				if(i==j)
				{
					data.dist[i][j]=0;
					continue;
				}
				data.dist[i][j] = Math.sqrt((data.vertices[i][0]-data.vertices[j][0])*(data.vertices[i][0]-data.vertices[j][0])+ (data.vertices[i][1]-data.vertices[j][1])*(data.vertices[i][1]-data.vertices[j][1]));
				data.dist[i][j] = data.truncate(data.dist[i][j]);	
			}
		}
		
		data.dist[0][data.vertex_num-1] = 0;
		data.dist[data.vertex_num-1][0] = 0;
		
		for (int k=0;k<data.vertex_num;k++)
		{
			for (int i=0;i<data.vertex_num;i++)
			{
				for (int j=0; j<data.vertex_num;j++)
				{
					if (data.dist[i][j]>data.dist[i][k]+data.dist[k][j])
					{
						data.dist[i][j] = data.dist[i][k]+ data.dist[k][j];
					}
				}
			}
		}
			
		for (int i=0; i<data.vertex_num; i++)
		{
			for (int j=0; j<data.vertex_num;j++)
			{
				if (i!=j)
				{
					data.arcs[i][j]=1;
				}
				else
				{
					data.arcs[i][j]=0;
				}
			}
		}
		
		for (int i=0; i<data.vertex_num;i++)
		{
			for (int j=0; j<data.vertex_num;j++)
			{
				if(i==j)
				{
					continue;
				}
				if ((data.a[i]+data.s[i]+data.dist[i][j]>data.b[j]) || (data.demands[i]+data.demands[j]>data.vehicle_capacity))
				{
					data.arcs[i][j]=0;
				}
				if (data.a[0]+data.s[i]+data.dist[0][i]+data.dist[i][data.vertex_num-1]>data.b[data.vertex_num-1])
				{
					System.out.println("The route generated is incorrect");
				}
			}
		}
		
		for (int i=1; i<data.vertex_num-1;i++)
		{
			if(data.b[i]-data.dist[0][i]<min1)
			{
				min1 = data.b[i]-data.dist[0][i];
			}
			if (data.a[i]+data.s[i]+data.dist[i][data.vertex_num-1]<min2)
			{
				min2 = data.a[i]+data.s[i]+data.dist[i][data.vertex_num-1];
			}
		}
		
		if (data.ET>min1||data.LT<min2)
		{
			System.out.println("DurationFalse!");
			System.exit(0);
		}
		
		data.arcs[data.vertex_num-1][0]=0;
		data.arcs[0][data.vertex_num-1]=1;
		
		for (int i=1;i<data.vertex_num-1;i++)
		{
			data.arcs[data.vertex_num-1][i]=0;
		}
		
		for (int i=1;i<data.vertex_num-1;i++)
		{
			data.arcs[i][0]=0;
		}
	}
				
	public static void main(String[] args)throws Exception
	{
		Data data = new Data();
		int vertex_num = 102;
		String s = "./c101_1.txt";
		process_file(s,data,vertex_num);
		System.out.println("Input Sucess");
		System.out.println("Cplex Process Working");
		VRPTW cplex = new VRPTW(data);
		cplex.build_model();
		double cplex_time1 = System.nanoTime();
		cplex.solve();
		cplex.solution.feasible();
		double cplex_time2 = System.nanoTime();
		double cplex_time = (cplex_time2 - cplex_time1)/1e9;
		System.out.println("Cplex_Time" + cplex_time + "Best Cost" + cplex.cost);
	}
}