using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Timers;

namespace TSP
{
    public class Solution
    {
        public ArrayList path = new ArrayList();
        public double cost;
        public int solutions;

        public Solution(ArrayList path, int solutions)
        {
            this.path = path;
            cost = costOfRoute();
            this.solutions = solutions;
        }
        public Solution(State newState, City[] cities, int solutions, double cost)
        {
            foreach (int i in newState.getRoute())
            {
                path.Add(cities[i]);
            }
            this.cost = cost;
            this.solutions = solutions + 1;
        }
        public double costOfRoute()
        {
            // go through each edge in the route and add up the cost. 
            int x;
            City here;
            double cost = 0D;

            for (x = 0; x < path.Count - 1; x++)
            {
                here = path[x] as City;
                cost += here.costToGetTo(path[x + 1] as City);
            }

            // go from the last city to the first. 
            here = path[path.Count - 1] as City;
            cost += here.costToGetTo(path[0] as City);
            return cost;
        }
    }
    public class BBoundStateSolver
    {
        private City[] cities;
        private Solution bssf;
        private HeapPriorityQueue queue;
        private double[,] originalDistMatrix;
        private int numPrunedStates = 0;
        private int numCreatedStates = 1;

        public BBoundStateSolver(City[] cities)
        {
            this.cities = cities;
        }

        public Solution getBSSF()
        {
            return this.bssf;
        }

        /*
        *
        * Solves the current traveling salesman problem. Note that the best path,
        * number of computed solutions and other variables must be requested
        * by their functions, as all cannot be passed back at once.
        * @param time_limit The upper bound time limit for this solve function.
        * If the program has not finished by the indicated time, it will stop
        * 
        */
        public string[] solve(int time_limit)
        {
            Stopwatch timer = new Stopwatch();
            timer.Start();
            queue = new HeapPriorityQueue();
            State initialState = new State(cities);
            queue.add(initialState);
            bssf = getInitialBSSF();
            originalDistMatrix = createInitialAdjancencyMatrix(cities);
            
            while (timer.ElapsedMilliseconds < time_limit && !queue.isEmpty())
            {
                State state = queue.deleteMin();
                //Check that the newest item is still a valid search option.
                if (state.getLowerBound() > bssf.cost)
                {
                    numPrunedStates++;
                    continue;
                }
                putChildrenOnQueue(state);
            }
            return returnSolution(bssf.cost, timer.Elapsed.ToString(), bssf.solutions);
        }

        private Solution getInitialBSSF()
        {
            GreedySolver solver = new GreedySolver(cities);
            Solution sol = solver.solve();
            if (sol.cost == double.PositiveInfinity)
            {
                sol = solver.defaultSolveProblem(cities);
            }
            return sol;
        }

        private string[] returnSolution(double cost, string time, int solutions)
        {
            string[] returnString = new string[3];
            returnString[0] = cost.ToString();
            returnString[1] = time;
            returnString[2] = solutions.ToString();
            Console.WriteLine("TOTAL NUMBER OF STATES: " + numCreatedStates.ToString());
            Console.WriteLine("TOTAL NUMBER OF PRUNED STATES: " + (numPrunedStates+queue.getCount()).ToString());
            Console.WriteLine("MAX SIZE OF QUEUE " + queue.getMaxOnQueue());
            return returnString;
        }
        private void putChildrenOnQueue(State state)
        {
            foreach (int i in state.getUnvisitedCities())
            {
                State newState = new State(state, i);
                numCreatedStates++;
                if (newState.getLowerBound() > bssf.cost)
                {
                    numPrunedStates++;
                    continue;
                }
                //If no children remain, this is a leaf node, and we need to see if its better than the current bssf
                if (newState.getUnvisitedCities().Count <= 0)
                {
                    double cost;
                    //If the newState is better than our current best, update it
                    if ((cost = getTotalCostOfPath(newState)) < bssf.cost)
                    {
                        bssf = new Solution(newState, cities, bssf.solutions, cost);
                    }
                    //Otherwise, throw it away
                    else
                    {
                        newState = null;
                    }
                }
                else
                {
                    queue.add(newState);
                }
            }
        }

        private double getTotalCostOfPath(State state)
        {
            double cost = 0D;
            List<int> route = state.getRoute();
            for (int i = 0; i < route.Count-1; i++)
            {
                cost += originalDistMatrix[route[i], route[i + 1]];
            }
            cost += originalDistMatrix[route[route.Count-1], route[0]];
            return cost;
        }
        private double[,] createInitialAdjancencyMatrix(City[] cities)
        {
            double[,] mat = new double[cities.Length, cities.Length];
            for (int i = 0; i < cities.Length; i++)
            {
                for (int j = 0; j < cities.Length; j++)
                {
                    if (i == j)
                    {
                        mat[i, j] = double.PositiveInfinity;
                    }
                    else
                    {
                        mat[i, j] = cities[i].costToGetTo(cities[j]);
                    }
                }
            }
            return mat;
        }
    }
    public class State
    {
        private double lowerbound;
        private List<int>
               Route = new List<int>();
        private double[,] leastCostMatrix;
        private HashSet<int> unvisitedCities = new HashSet<int>();

        State() { }

        public State(City[] cities)
        {
            setupInitialLeastCostMatrix(cities);
            Route.Add(0);
        }
        public State(State pastState, int CityToExpandTo)
        {
            leastCostMatrix = (double[,])pastState.getLeastCostMatrix().Clone();
            Route = new List<int>(pastState.getRoute());
            lowerbound = pastState.getLowerBound();
            unvisitedCities = new HashSet<int>(pastState.getUnvisitedCities());
            expandOn(pastState.getRoute()[pastState.getRoute().Count()-1], CityToExpandTo);
        }
        

        public double getLowerBound()
        {
            return lowerbound;
        }

        public double[,] getLeastCostMatrix()
        {
            return leastCostMatrix;
        }

        public List<int> getRoute()
        {
            return Route;
        }

        public HashSet<int> getUnvisitedCities()
        {
            return unvisitedCities;
        }
        

        /*
        * This function determines how "important" a given state is to solving the TSP problem
        * The two primary factors in finding this is the lower bound for a given state (as a smaller
        * lower bound implies a given state is more likely to find a better solution) and the depth
        * of the state (as it is beneficial to finish a path, in order to trim the tree for states
        * that are worst than the current best
        */
        public int getPriority()
        {
            return (int)lowerbound - ((int)lowerbound / (unvisitedCities.Count()));

            //EXAMPLES:

            //Comparing a lowerbound:10,depth:1 ........ PRIORITY = 7
            //with a lowerbound:12, depth:3 ........ PRIORITY = 0 (Heavily prioritizes nodes that are almost completed)
            //10 - (10/(4-1)) = 10 - (10/3) = 20/3 = 7
            //12 - (12/(4-3)) = 12 - (12/1) = 0

            //Comparing a lowerbound:5,depth:1 ........ PRIORITY = 3.33 (As both nodes are similar in depth, picks the smaller one)
            //with a lowerbound:7, depth:2 ........ PRIORITY = 3.5
            //5 - (5/(4-1)) = 5 - (5/3) = 10/3 = 3.33
            //7 - (7/(4-2)) = 7 - (7/2) = 7/2 = 3.5
        }

        private void setupInitialLeastCostMatrix(City[] cities)
        {
            leastCostMatrix = new double[cities.Length, cities.Length];
            for (int i = 0; i < cities.Length; i++)
            {
                unvisitedCities.Add(i);
                for (int j = 0; j < cities.Length; j++)
                {
                    if (i == j)
                    {
                        leastCostMatrix[i, j] = double.PositiveInfinity;
                    }
                    else
                    {
                        leastCostMatrix[i, j] = cities[i].costToGetTo(cities[j]);
                    }
                }
            }
            zeroOutLeastCostMatrix();
            unvisitedCities.Remove(0);
        }

        private void expandOn(int fromCity, int toCity)
        {
            lowerbound += leastCostMatrix[fromCity, toCity];
            for (int i = 0; i < leastCostMatrix.GetLength(0); i++)
            {
                leastCostMatrix[fromCity,i] = double.PositiveInfinity;
                leastCostMatrix[i, toCity] = double.PositiveInfinity;
            }
            leastCostMatrix[toCity, fromCity] = double.PositiveInfinity;
            zeroOutLeastCostMatrix();
            Route.Add(toCity);
            unvisitedCities.Remove(toCity);
        }

        private void zeroOutLeastCostMatrix()
        {
            for (int i = 0; i < leastCostMatrix.GetLength(0); i++)
            {
                zeroOutRow(i);
            }
            for (int j = 0; j < leastCostMatrix.GetLength(1); j++)
            {
                zeroOutCol(j);
            }
        }
        private void zeroOutRow(int i)
        {
            double min = double.PositiveInfinity;
            for (int j = 0; j < leastCostMatrix.GetLength(1); j++)
            {
                if (leastCostMatrix[i, j] < min)
                {
                    min = leastCostMatrix[i, j];
                }
            }
            if (min == double.PositiveInfinity)
            {
                return;
            }
            lowerbound += min;
            for (int j = 0; j < leastCostMatrix.GetLength(1); j++)
            {
                if (leastCostMatrix[i,j] != double.PositiveInfinity)
                {
                    leastCostMatrix[i, j] -= min;
                }
            }
        }
        private void zeroOutCol(int j)
        {
            double min = double.PositiveInfinity;
            for (int i = 0; i < leastCostMatrix.GetLength(0); i++)
            {
                if (leastCostMatrix[i, j] < min)
                {
                    min = leastCostMatrix[i, j];
                }
            }
            if (min == double.PositiveInfinity)
            {
                return;
            }
            lowerbound += min;
            for (int i = 0; i < leastCostMatrix.GetLength(0); i++)
            {
                if (leastCostMatrix[i, j] != double.PositiveInfinity)
                {
                    leastCostMatrix[i, j] -= min;
                }
            }
        }
        
    }
    class HeapPriorityQueue
    {
        List<State> stateTree = new List<State>();
        private int maxOnQueue = 0;
        public HeapPriorityQueue(State initialState)
        {
            add(initialState);
        }
        public HeapPriorityQueue() { }
        public void add(State s)
        {
            stateTree.Add(s);
            maxOnQueue = Math.Max(maxOnQueue, stateTree.Count);
            checkBalanceUp(stateTree.Count - 1);
        }

        public State deleteMin()
        {
            State s = stateTree[0];
            stateTree[0] = stateTree[stateTree.Count - 1];
            stateTree.RemoveAt(stateTree.Count - 1);
            checkBalanceDown(0);
            return s;
        }
        public int getMaxOnQueue()
        {
            return maxOnQueue;
        }
        private void checkBalanceUp(int i)
        {
            while (i > 0)
            {
                //If current is smaller than our parent, we have a problem
                
                if (stateTree[i].getPriority() < stateTree[i / 2].getPriority())
                {
                    swap(i, i / 2);
                }
                i /= 2;
                
            }
        }
        private void checkBalanceDown(int i)
        {
            while ((i * 2) + 1 < stateTree.Count-1)
            {
                int leftChildIndex = i * 2 + 1;

                int rightChildIndex = leftChildIndex + 1;

                int minIndex = 0;
                if (stateTree[leftChildIndex].getPriority() < stateTree[rightChildIndex].getPriority())
                {
                    minIndex = leftChildIndex;
                }
                else
                {
                    minIndex = rightChildIndex;
                }

                if (stateTree[i].getPriority() > stateTree[minIndex].getPriority())
                {
                    swap(i, minIndex);
                }
                i = minIndex;
            }
        }
        private void swap(int i1, int i2)
        {
            State v1 = stateTree[i1];
            State v2 = stateTree[i2];
            stateTree[i1] = stateTree[i2];
            stateTree[i2] = v1;
        }
        public int getCount()
        {
            return stateTree.Count;
        }

        public bool isEmpty()
        {
            return stateTree.Count <= 0;
        }

    }
}
