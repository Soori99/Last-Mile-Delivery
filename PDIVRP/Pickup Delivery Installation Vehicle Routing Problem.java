import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;
import java.util.List;

import org.apache.poi.ss.usermodel.Cell;
import org.apache.poi.ss.usermodel.Row;
import org.apache.poi.ss.usermodel.Sheet;
import org.apache.poi.ss.usermodel.Workbook;
import org.apache.poi.ss.usermodel.WorkbookFactory;

import ilog.concert.*;
import ilog.cplex.*;

class Data_Check
{
	int pickup_num;
	int drop_num; 
	int vertex_num;
	int crew_num;
	double prodid [];
	String vertex_name [];
	double vertex_node[];
	double pickup_node[];
	double drop_node[];
	double ET;
	double LT;
	//int vehicle_num;
	int vehicle_capacity = 200;
	double a[];
	double b[];
	double s[];
	double arcs [][];
	double dist [][];
	double cost [][][];
	double time [][];
	double load [];
	double installtime [][];
	double crewskills [][];
	double installation[];
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

class Solution_Check
{
	double epsilon = 0.0001;
	Data_Check data = new Data_Check();
	ArrayList<ArrayList<Integer>> routes = new ArrayList<>();
	ArrayList<ArrayList<Double>> servicetimes = new ArrayList<>();
	ArrayList<ArrayList<String>> route_names = new ArrayList<>();
	ArrayList<ArrayList<Double>> loads = new ArrayList<>();
	ArrayList<ArrayList<Integer>> crews = new ArrayList<>();
	//ArrayList<ArrayList<Integer>> vehicles = new ArrayList<>();
	
	public Solution_Check(Data_Check data, ArrayList<ArrayList<Integer>> routes, ArrayList<ArrayList<Double>> servicetimes, ArrayList<ArrayList<String>> route_names, ArrayList<ArrayList<Double>> loads, ArrayList<ArrayList<Integer>> crews)
	{
		super();
		this.data = data;
		this.routes = routes;
		this.servicetimes = servicetimes;
		this.route_names = route_names;
		this.loads = loads;
		this.crews = crews;
		//this.vehicles = vehicles;
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
} 

public class PDI_Check
{
	Data_Check data;
	IloCplex model;
	public IloNumVar x[][][];
	public IloNumVar t[][];
	public IloNumVar l[][];
	double cost;
	Solution_Check solution;
	
	public PDI_Check(Data_Check data)
	{
		super();
		this.data = data;
	}
	
	public void solve() throws IloException
	{
		ArrayList<ArrayList<Integer>> routes = new ArrayList<>();
		ArrayList<ArrayList<Double>> servicetimes = new ArrayList<>();
		ArrayList<ArrayList<String>> route_names = new ArrayList<>();
		ArrayList<ArrayList<Double>> loads = new ArrayList<>();
		ArrayList<ArrayList<Integer>> crews = new ArrayList<>();
		//ArrayList<ArrayList<Integer>> vehicles = new ArrayList<>();
		
		for (int c=0; c<data.crew_num; c++)
		{
			ArrayList<Integer> r = new ArrayList<>();
			ArrayList<Double> t = new ArrayList<>();
			ArrayList<String> n = new ArrayList<>();
			ArrayList<Double> l = new ArrayList<>();
			ArrayList<Integer> s = new ArrayList<>();
			//ArrayList<Integer> v = new ArrayList<>();
			
			routes.add(r);
			servicetimes.add(t);
			route_names.add(n);
			loads.add(l);
			crews.add(s);
			//vehicles.add(v);
		}
		
		if (model.solve() == false)
		{
			System.out.println("Problem cannot be solved");
			return;
		}
		
		else
		{
			for (int c=0; c<data.crew_num;c++)
			{
				int i=0;
				boolean terminate = true;
				crews.get(c).add(c+1);
				routes.get(c).add(0);
				servicetimes.get(c).add(0.0);
				route_names.get(c).add(data.vertex_name[0]);
				loads.get(c).add(0.0);
				while (terminate)
				{
					for (int j=0; j<data.vertex_num; j++)
					{
						if (data.arcs[i][j]>0.5 && model.getValue(x[i][j][c])>=0.5)
						{
							if (j==data.vertex_num-1)
							{
								routes.get(c).add(0);
								servicetimes.get(c).add(model.getValue(t[j][c]));
								route_names.get(c).add(data.vertex_name[j]);
								loads.get(c).add(data.load[j]);
								i=j;
								break;
							}
							routes.get(c).add(j);
							servicetimes.get(c).add(model.getValue(t[j][c]));
							route_names.get(c).add(data.vertex_name[j]);
							loads.get(c).add(data.load[j]);
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
		solution = new Solution_Check(data,routes,servicetimes,route_names,loads,crews);
		cost = model.getObjValue();
		System.out.println("Crews = "+ solution.crews);
		//System.out.println("Vehicles = "+ solution.vehicles);
		System.out.println("Routes = "+ solution.routes);
		System.out.println("Servicetime = "+ solution.servicetimes);
		System.out.println("Route Locations = "+ solution.route_names);
		System.out.println("Loads = "+ solution.loads);	
	}
	
	private void build_model() throws IloException
	{
		//Model
		model = new IloCplex();
		model.setOut(null);
	
		//Variables
		x = new IloNumVar[data.vertex_num][data.vertex_num][data.crew_num];
		t = new IloNumVar[data.vertex_num][data.crew_num];
		l = new IloNumVar[data.vertex_num][data.crew_num];	
		
		for (int i=0; i<data.vertex_num;i++)
		{
			for ( int c=0;c<data.crew_num;c++)
			{
				t[i][c] = model.numVar(0,1e15,IloNumVarType.Float,"t"+i+","+c);
				l[i][c] = model.numVar(0,1e15,IloNumVarType.Float,"l"+i+","+c);
			}
			for (int j=0; j<data.vertex_num;j++)
			{
				for (int c=0; c<data.crew_num;c++)
				{
					x[i][j][c] = model.numVar(0, 1,IloNumVarType.Int,"x"+i+","+j+","+c);
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
				for (int c=0; c<data.crew_num;c++)
				{
					obj = model.sum(obj,model.prod(data.cost[i][j][c], x[i][j][c]));
				}
			}
		}
		model.addMinimize (obj);
		
		//Constraints
		//Constraint 1
		for (int i=1; i<data.pickup_num+1;i++)
		{
			IloNumExpr expr1 = model.numExpr();
			for (int c=0; c<data.crew_num; c++)
			{
				for (int j=1; j<data.vertex_num;j++)
				{
					if(data.arcs[i][j]==1)
					{
						expr1 = model.sum(expr1,x[i][j][c]);
					}
				}
			}
			model.addEq(expr1,1);
		}
		
		//Constraint 2
		for (int c=0;c<data.crew_num;c++)
		{
			IloNumExpr expr3 = model.numExpr();
			for (int j=1;j<data.vertex_num;j++)
			{
				if (data.arcs[0][j]==1)
				{
					expr3 = model.sum(expr3,x[0][j][c]);
				}
			}
			model.addEq(expr3,1);
		}
		
		//Constraint 3
		for (int c=0; c<data.crew_num;c++)
		{
			for (int i=1; i<data.pickup_num+1;i++)
			{
				IloNumExpr expr2 = model.numExpr();
				IloNumExpr subExpr1 = model.numExpr();
				IloNumExpr subExpr2 = model.numExpr();
				for (int j=0; j<data.vertex_num; j++)
				{
					if (data.arcs[i][j]==1)
					{
						subExpr1 = model.sum(subExpr1,x[i][j][c]);
					}
					if (data.arcs[j][i+data.pickup_num]==1)
					{
						subExpr2 = model.sum(subExpr2,x[j][i+data.pickup_num][c]);
					}
				}
				expr2 = model.sum(subExpr1,model.prod(-1,subExpr2));
				model.addEq(expr2, 0);
			}
		}
				
		//Constraint 4
		for (int c=0; c<data.crew_num;c++)
		{
			for (int j= 1; j<data.vertex_num-1;j++)
			{
				IloNumExpr expr4 = model.numExpr();
				IloNumExpr subExpr3 = model.numExpr();
				IloNumExpr subExpr4 = model.numExpr();
				for (int i=0; i<data.vertex_num-1;i++)
				{
					if (data.arcs[i][j]==1)
					{
						subExpr3 = model.sum(subExpr3,x[i][j][c]);
					}
				}
				for (int i=1; i<data.vertex_num;i++)
				{				
					if (data.arcs[j][i]==1)
					{
						subExpr4 = model.sum(subExpr4,x[j][i][c]);
					}
					
				}
				expr4 = model.sum(subExpr3,model.prod(-1,subExpr4));
				model.addEq(expr4, 0);
			}
		}
		
		//Constraint 5
		for (int c=0; c<data.crew_num;c++)
		{
			IloNumExpr expr5 = model.numExpr();
			for (int i=0; i< data.vertex_num-1;i++)
			{
				if(data.arcs[i][data.vertex_num-1]==1)
				{
					expr5 = model.sum(expr5,x[i][data.vertex_num-1][c]);				
				}
			}
			model.addEq(expr5, 1);
		}
		
			
		//Constraint 6
		double M = 1e5;
		for (int c=0; c<data.crew_num;c++)
		{
			for (int i=0; i<data.vertex_num;i++)
			{
				for (int j=0; j<data.vertex_num; j++)
				{
					if (data.arcs[i][j]==1)
					{
						IloNumExpr expr6 = model.numExpr();
						IloNumExpr expr7 = model.numExpr();
						expr6 = model.sum(t[i][c],data.s[i]+(data.time[i][j]));
						expr6 = model.sum(expr6, model.prod(-1, t[j][c]));
						if (data.installation[i]==1)
						{	
							int num = (int)(data.prodid[i]-1);
							expr6 = model.sum(expr6, data.installtime[c][num]);
						}
						expr7 = model.prod(M, model.sum(1,model.prod(-1, x[i][j][c])));
						model.addLe(expr6, expr7);
					}
				}
			}
		}
		
		//Constraint 7
		for (int c=0; c<data.crew_num;c++)
		{
			for (int i=0; i<data.vertex_num; i++)
			{
				for (int j=0; j<data.vertex_num; j++)
				{
					if (data.arcs[i][j]==1)
					{
						model.addLe(data.a[i], t[i][c]);
						model.addLe(t[i][c], data.b[i]);
					}
				}
			}
		}	
		
		
		//Constraint 8
		for (int c=0; c<data.crew_num; c++)
		{
			model.addLe(data.ET,  t[0][c]);
			model.addLe(data.ET, t[data.vertex_num-1][c]);
			model.addLe(t[0][c], data.LT);
			model.addLe(t[data.vertex_num-1][c],data.LT);
		}
		
		//Constraint 9
		for (int c=0; c<data.crew_num;c++)
		{
			IloNumExpr expr8 = model.numExpr();
			for (int i=1; i<data.pickup_num+1;i++)
			{
				expr8= model.sum(t[i][c], data.time[i+data.pickup_num][c]);
				model.addLe(expr8,t[i+data.pickup_num][c]);
			}
		}
		
		//Constraint 10
		for (int c=0; c<data.crew_num;c++)
		{
			IloNumExpr expr10 = model.numExpr();
			for (int i=1; i<data.pickup_num+1;i++)
			{
				expr10 = model.sum(expr10, l[i][c]);
				model.addLe(data.load[i], expr10);
			}
			model.addLe(expr10,data.vehicle_capacity);
		}
		
		//Constraint 11
		for (int c=0; c<data.crew_num;c++)
		{
			IloNumExpr expr11 = model.numExpr();
			for (int i=1; i<data.drop_num+1; i++)
			{
				expr11 = model.sum(expr11, l[data.drop_num+i][c]);
				model.addLe(0, expr11);
				model.addLe(expr11, data.vehicle_capacity + data.load[data.drop_num+i]);
			}
		}
	
		//Constraint 12
		for (int c=0; c<data.crew_num;c++)	
		{
			model.addEq(l[0][c], 0);
		}
		
		//Constraint 8
		for(int c =0; c<data.crew_num;c++)
		{
			IloNumExpr expr8 = model.numExpr();
			for (int i=1; i<data.vertex_num-1;i++)
			{
				IloNumExpr expr9 = model.numExpr();
				for (int j=0; j<data.vertex_num;j++)
				{
					if(data.arcs[i][j]==1)
					{
						expr9 = model.sum(expr9, x[i][j][c]);
					}
				}
				expr8 = model.sum(expr8, model.prod(data.load[i], expr9));
			}
			model.addLe(expr8, data.vehicle_capacity);
		}
		
		//Constraint 13
		for (int c=0; c<data.crew_num;c++)
		{
			IloNumExpr expr13 = model.numExpr(); 
			for (int i=0;i<data.vertex_num;i++)
			{
				int num = (int)data.prodid[i];
				for (int j=0;j<data.vertex_num;j++)
				{
					if(data.arcs[i][j]==1)
					{
						if(data.installation[i]==1)
						{	
							if(data.crewskills[c][num-1]==1)
							{
								expr13 = model.sum(expr13,x[i][j][c]);
							}
						}
					}
				}
			}
			model.addEq(expr13,1);
		}
	}
	
	public static double convert(String s) 
	{
		String[] array1 = s.split("\\�");
		int[] u = new int[2];
		for(int i=0; i<array1.length; i++) 
		{
			if(i==0)
				u[i] = Integer.parseInt(array1[i]);
			else {
				String[] array2 = array1[i].split("\\'");
				u[i] = Integer.parseInt(array2[i-1].replaceAll("\\s+"," ").trim());
			}
		}
		double v = u[1];
		return(u[0] + (v/60));
	}
		
	public static double distance(double lx1, double ly1, double lx2, double ly2) 
	{
		int R = 6371000;
		double t1 = lx1 * Math.PI/180;
		double t2 = lx2 * Math.PI/180;
		double dt = (lx2-lx1) * Math.PI/180;
		double dl = (ly1-ly2) * Math.PI/180;

		double a = Math.sin(dt/2) * Math.sin(dt/2) + 
		          Math.cos(t1) * Math.cos(t2) *
		          Math.sin(dl/2) * Math.sin(dl/2);
		double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
		double d = R * c;
		return(d/1000);
	}
	
	public static void process_file(String s, Data_Check data) throws Exception 
	{
		//Excel Input
		FileInputStream fis = new FileInputStream(s);
		Workbook wb = WorkbookFactory.create(fis);
			
		//Number of Pickups, Drops and Total Vertices
		Sheet sh1 = wb.getSheet("ProductReq");
		int nr1 = sh1.getLastRowNum();
		int nc1=0;
		for(int i=0; i<10; i++) 
		{
			if(sh1.getRow(0).getCell(i)==null) 
			{
				break;
			}
			else 
			{
				nc1=nc1+1;
			}
		}		
				
		data.vertex_num = 2*nr1+2;
		data.pickup_num = nr1;
		data.drop_num= nr1;
				
		/*System.out.println(data.vertex_num);
		System.out.println(data.pickup_num);
		System.out.println(data.drop_num);*/
				
		ArrayList<String> til1=new ArrayList<String>();
		data.pickup_node = new double[nr1];
		data.drop_node = new double[nr1];
		data.installation = new double[data.vertex_num];
		double demandid[]= new double[nr1];
		double delorins[] = new double[nr1];
				
		for (int i=0;i<nc1;i++)
		{
			String[] temp1 = new String[nr1];
			til1.add(String.valueOf(sh1.getRow(0).getCell(i)));
			for(int j=0; j<nr1; j++) 
			{
				temp1[j]= String.valueOf(sh1.getRow(j+1).getCell(i));
				if(til1.get(i).equals("ProductID")) 
				{
					demandid[j] = Double.parseDouble(temp1[j]);
				}
				else if(til1.get(i).equals("PickupID"))
				{
					data.pickup_node[j] = Double.parseDouble(temp1[j]);
				}
				else if(til1.get(i).equals("CustomerID"))
				{
					data.drop_node[j] = Double.parseDouble(temp1[j]);
				}
				else if(til1.get(i).equals("Del/Ins"))
				{
					delorins[j] = Double.parseDouble(temp1[j]);
				}
			}
		}
				
		data.vertex_node = new double[data.vertex_num];
		data.vertex_node[0] = 1;
		data.vertex_node[data.vertex_num-1]=1;
				
		for (int j=1; j<=nr1;j++)
		{
			data.vertex_node[j] = data.pickup_node[j-1];
			data.vertex_node[j+data.pickup_num] = data.drop_node[j-1];
		}
				
		/*for (int j=0; j<data.vertex_num;j++)
		{
			System.out.println(data.vertex_node[j]);
		}
				
		for (int j=0; j<nr1;j++)
		{
			System.out.println(data.pickup_node[j]);
		}
				
		for (int j=0; j<nr1;j++)
		{
			System.out.println(data.drop_node[j]);
		}*/
				
		data.installation[0] = 0;
		data.installation[data.vertex_num-1] = 0;
				
		for (int j=0;j<data.pickup_num;j++)
		{
			data.installation[j+1] = 0;
		}
		for (int j=0; j<data.drop_num;j++)
		{
			data.installation[j+1+data.pickup_num] = delorins[j];
		}
						
		/*for (int j=0;j<data.vertex_num;j++)
		{
			System.out.println(data.installation[j]);
		}*/
				
		//Demand Input 
		Sheet sh2 = wb.getSheet("ProductDetails");
		int nr2 = sh2.getLastRowNum();
		int nc2=0;
		for(int i=0; i<10; i++) 
		{
			if(sh2.getRow(0).getCell(i)==null) 
			{
				break;
			}
			else 
			{
				nc2=nc2+1;
			}
		}
		
		data.load = new double[data.vertex_num];
		data.load[0] = 0;
		data.load[data.vertex_num-1]=0;
				
		double demand[] = new double[nr2];
		ArrayList<String> til2=new ArrayList<String>();
				
		for (int i=0;i<nc2;i++)
		{
			String[] temp2 = new String[nr2];
			til2.add(String.valueOf(sh2.getRow(0).getCell(i)));
			for(int j=0; j<nr2; j++) 
			{
				temp2[j]= String.valueOf(sh2.getRow(j+1).getCell(i));
				if(til2.get(i).equals("Product Capacity")) 
				{
					demand[j] = Double.parseDouble(temp2[j]);
				}
			}
		}
				
		data.prodid = new double[data.vertex_num];
		data.prodid[0] = 0;
		data.prodid[data.vertex_num-1] = 0;
				
		for (int i=1;i<=nr1;i++)
		{
			if (demandid[i-1]==1.0)
			{
				data.load[i] = demand[0];
				data.prodid[i] = demandid[i-1];
				data.load[nr1+i] = -1*demand[0];
				data.prodid[nr1+i] = demandid[i-1];	
			}
			else if (demandid[i-1]==2.0)
			{
				data.load[i] = demand[1];
				data.prodid[i] = demandid[i-1];
				data.load[nr1+i] = -1*demand[1];
				data.prodid[nr1+i] = demandid[i-1];
			}
			else if (demandid[i-1]==3.0)
			{
				data.load[i] = demand[2];
				data.prodid[i] = demandid[i-1];
				data.load[nr1+i] = -1*demand[2];
				data.prodid[nr1+i] = demandid[i-1];
			}
		}
				
		/*for (int j=0; j<data.vertex_num;j++)
		{
			//System.out.println(data.load[j]);
			//System.out.println(data.prodid[j]);
		}*/
				
		//Distance Calculation between nodes
		Sheet sh = wb.getSheet("LatLong");
		int nr = sh.getLastRowNum();
		int nc=0;
		for(int i=0; i<10; i++) 
		{
			if(sh.getRow(0).getCell(i)==null) 
			{
				break;
			}
			else 
			{
				nc=nc+1;
			}
		}
				
		//Conversion
		ArrayList<String> til=new ArrayList<String>();
		Hashtable<String, List> locationdata = new Hashtable<String, List>();
		double[] xloc = new double[nr];
		double[] yloc = new double[nr];
		String[] loc = new String[nr];
		for(int i=0; i<nc; i++) 
		{
			String[] temp = new String[nr];
			til.add(String.valueOf(sh.getRow(0).getCell(i)));
			for(int j=0; j<nr; j++) 
			{
				temp[j]=String.valueOf(sh.getRow(j+1).getCell(i));
				if(til.get(i).equals("Latitude")) 
				{
					xloc[j] = convert(temp[j]);
				}
				if(til.get(i).equals("Longitude"))
				{
					yloc[j] = convert(temp[j]);
				}
				if(til.get(i).equals("Location")) 
				{
					loc[j] = temp[j];
				}
			}
			List<String> lis = (List<String>) Arrays.asList(temp);
			locationdata.put(til.get(i), lis);
		}
						
		double[] xloc_final = new double[data.vertex_num];
		double[] yloc_final = new double[data.vertex_num];
		String[] loc_final = new String[data.vertex_num];
		data.vertex_name = new String[data.vertex_num];
				
		for (int k=0;k<data.vertex_num;k++)
		{
			for(int i=0; i<xloc.length; i++) 
			{
				if(data.vertex_node[k]==i+1)
				{
					loc_final[k] = loc[i];
					data.vertex_name[k] = loc_final[k];
					xloc_final[k] = xloc[i];
					yloc_final[k] = yloc[i];
				}
			}	
		}
									
		//Distance Matrix
		data.dist= new double[data.vertex_num][data.vertex_num];
		for(int i=0; i<data.vertex_num; i++) 
		{
			for(int j=0; j<data.vertex_num; j++) 
			{
				data.dist[i][j] = distance(xloc_final[i],yloc_final[i],xloc_final[j],yloc_final[j]);
			}
		}
				
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

		//Excel output
		Sheet sh4 = wb.getSheet("DistanceMatrix");
		for(int i=0; i<=data.vertex_num; i++) 
		{
			Row row=sh4.createRow(i);
			for(int j=0; j<=data.vertex_num;j++) 
			{
				Cell cell=row.createCell(j);
				if(i==0&&j>0) 
				{
					cell.setCellValue(loc_final[j-1]);					
				}
				if(i>0&&j==0) 
				{
					cell.setCellValue(loc_final[i-1]);
				}
				if(i>0&&j>0) 
				{
					cell.setCellValue(data.dist[i-1][j-1]);	
				}
			}
		}
				
		//Number of Crews and their skills
		Sheet sh7 = wb.getSheet("CrewDetails");
		int nr7 = sh7.getLastRowNum();
		data.crew_num=nr7;
		data.crewskills = new double[data.crew_num][data.crew_num];
		int nc7=0;
		double[] crewcostperday = new double[data.crew_num];
		ArrayList<String> til7=new ArrayList<String>();
				
		for(int i=0; i<10; i++) 
		{
			if(sh7.getRow(0).getCell(i)==null) 
			{
				break;
			}
			else 
			{
				nc7=nc7+1;
			}
		}
				
		for (int i=0;i<nc7;i++)
		{
			String[] temp7 = new String[nr7];
			til7.add(String.valueOf(sh7.getRow(0).getCell(i)));
			for(int j=0; j<nr7; j++) 
			{
				temp7[j]= String.valueOf(sh7.getRow(j+1).getCell(i));
				if(til7.get(i).equals("S1")) 
				{
					data.crewskills[i-1][j] = Double.parseDouble(temp7[j]);
				}
				if(til7.get(i).equals("S2"))
				{
					data.crewskills[i-1][j] = Double.parseDouble(temp7[j]);
				}
				if(til7.get(i).equals("S3")) 
				{
					data.crewskills[i-1][j] = Double.parseDouble(temp7[j]);
				}
				if(til7.get(i).equals("Crew Cost")) 
				{
					crewcostperday[j] = Double.parseDouble(temp7[j]);
				}			
			}
		}
				
		/*for (int j=0; j<3;j++)
		{
			for (int i=0; i<3; i++)
			{
				System.out.println(data.crewskills[i][j]);
			}
		}*/
					
		//Crew Installation Time Matrix
		Sheet sh8 = wb.getSheet("TimeDetails");
		int nr8 = sh7.getLastRowNum();
		data.installtime= new double[data.crew_num][data.crew_num];
		int nc8=0;
		ArrayList<String> til8=new ArrayList<String>();
				
		for(int i=0; i<10; i++) 
		{
			if(sh7.getRow(0).getCell(i)==null) 
			{
				break;
			}
			else 
			{
				nc8=nc8+1;
			}
		}
				
		for (int i=0;i<nc8;i++)
		{
			String[] temp8 = new String[nr8];
			til8.add(String.valueOf(sh8.getRow(0).getCell(i)));
			for(int j=0; j<nr8; j++) 
			{
				temp8[j]= String.valueOf(sh8.getRow(j+1).getCell(i));
				if(til8.get(i).equals("ST1")) 
				{
					data.installtime[i-1][j] = Double.parseDouble(temp8[j]);
				}
				if(til8.get(i).equals("ST2"))
				{
					data.installtime[i-1][j] = Double.parseDouble(temp8[j]);
				}
				if(til8.get(i).equals("ST3")) 
				{
					data.installtime[i-1][j] = Double.parseDouble(temp8[j]);
				}
			}
		}
				
		/*for (int j=0; j<3;j++)
		{
			for (int i=0; i<3; i++)
			{
				System.out.println(data.installtime[i][j]);
			}
		}*/
				
		//Time window and service time
		data.a = new double[data.vertex_num];
		data.b = new double[data.vertex_num];
		data.s = new double[data.vertex_num];
				
		Sheet sh5 = wb.getSheet("MasterDetails");
		int nr5 = sh5.getLastRowNum();
		int nc5=0;
					
		double starttime[] = new double[nr5];
		double endtime[] = new double[nr5];
		double servicetime[] = new double[nr5];
				
		for(int i=0; i<10; i++) 
		{
			if(sh5.getRow(0).getCell(i)==null) 
			{
				break;
			}
			else 
			{
				nc5=nc5+1;
			}
		}
				
		ArrayList<String> til5=new ArrayList<String>();
				
		for(int i=0; i<nc5; i++) 
		{
			String[] temp5 = new String[nr5];
			til5.add(String.valueOf(sh5.getRow(0).getCell(i)));
			for(int j=0; j<nr5; j++) 
			{
				temp5[j]= String.valueOf(sh5.getRow(j+1).getCell(i));
				if(til5.get(i).equals("ReadyTime")) 
				{
					starttime[j] = Double.parseDouble(temp5[j]);
				}
				else if(til5.get(i).equals("DueTime"))
				{
					endtime[j] = Double.parseDouble(temp5[j]);
				}
				else if(til5.get(i).equals("ServiceTime"))
				{
					servicetime[j] = Double.parseDouble(temp5[j]);
				}
			}
		}
						
		for (int k=0;k<data.vertex_num;k++)
		{
			for(int i=0; i<nr5; i++) 
			{
				if(data.vertex_node[k]==i+1)
				{
					data.a[k] = starttime[i];
					data.b[k] = endtime[i];
					data.s[k] = servicetime[i];
				}
			}	
		}
			
		data.a[data.vertex_num-1] = data.a[0];
		data.b[data.vertex_num-1] = data.b[0];
		data.ET = data.a[0];
		data.LT = data.b[0];
		data.s[data.vertex_num-1] = 0;
				
		/*for (int j=0; j<data.vertex_num;j++)
		{
			System.out.println(data.a[j]);
		}*/
				
		/*for (int j=0; j<data.vertex_num;j++)
		{
			System.out.println(data.b[j]);
		}*/
				
		/*for (int j=0; j<data.vertex_num;j++)
		{
			System.out.println(data.s[j]);
		}*/
				
		//Time Matrix
		Sheet sh6 = wb.getSheet("TimeMatrix");
		data.time = new double [data.vertex_num][data.vertex_num];
		for(int i=0; i<=data.vertex_num; i++) 
		{
			Row row=sh6.createRow(i);
			for(int j=0; j<=data.vertex_num;j++) 
			{
				Cell cell=row.createCell(j);
				if(i==0&&j>0) 
				{
					cell.setCellValue(loc_final[j-1]);					
				}
				if(i>0&&j==0) 
				{
					cell.setCellValue(loc_final[i-1]);
				}
				if(i>0&&j>0) 
				{
					data.time[i-1][j-1]= (data.dist[i-1][j-1]/50)*60;
					cell.setCellValue(data.time[i-1][j-1]);	
				}
			}
		}
				
		//Number of Vehicles and their capacity
		/*Sheet sh3 = wb.getSheet("VehicleDetails");
		int nr3 = sh3.getLastRowNum();
		data.vehicle_num=nr3;
		data.vehicle_capacity = new double[data.vehicle_num];
		int nc3=0;
		ArrayList<String> til3=new ArrayList<String>();
				
		for(int i=0; i<10; i++) 
		{
			if(sh3.getRow(0).getCell(i)==null) 
			{
				break;
			}
			else 
			{
				nc3=nc3+1;
			}
		}
		//double vehiclecostperkm[] = new double[data.vehicle_num];
		
		for(int i=0; i<nc3; i++) 
		{
			String[] temp3 = new String[nr3];
			til3.add(String.valueOf(sh3.getRow(0).getCell(i)));
			for(int j=0; j<nr3; j++) 
			{
				temp3[j]= String.valueOf(sh3.getRow(j+1).getCell(i));
				if(til3.get(i).equals("Vehicle Capacity")) 
				{
					data.vehicle_capacity[j]=Double.parseDouble(temp3[j]);
				}
				if(til3.get(i).equals("Vehicle Cost"))
				{
					vehiclecostperkm[j] = Double.parseDouble(temp3[j]);
				}
			}
		}*/
		
		//Cost Matrix
		data.cost = new double [data.vertex_num][data.vertex_num][data.crew_num];
		for (int i=0; i<data.vertex_num;i++)
		{
			for (int j=0;j<data.vertex_num;j++)
			{
				for (int c=0;c<data.crew_num;c++)
				{
					data.cost[i][j][c]= data.dist[i][j]*50+crewcostperday[c];
				}
			}
		}
				
		double min1 = 1e15;
		double min2 = 1e15;
			
		data.arcs = new double[data.vertex_num][data.vertex_num];
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
				
		FileOutputStream fos = new FileOutputStream(s);
		wb.write(fos);
		fos.flush();
		fos.close();		
		System.out.println("Done");	
	}	
	
	public static void main(String[] args)throws Exception
	{
		Data_Check data = new Data_Check();
		String s = "./distance_matrix_lmd_install_changeincrewcost.xlsx";
		process_file(s,data);
		System.out.println("Input Sucess");
		System.out.println("Cplex Process Working");
		PDI_Check cplex = new PDI_Check(data);
		cplex.build_model();
		double cplex_time1 = System.nanoTime();
		cplex.solve();
		double cplex_time2 = System.nanoTime();
		double cplex_time = (cplex_time2 - cplex_time1)/1e9;
		System.out.println("Cplex_Time " + cplex_time + " " + "Best Cost " + cplex.cost);
	}
}