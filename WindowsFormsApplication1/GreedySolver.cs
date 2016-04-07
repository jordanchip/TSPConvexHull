using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TSP
{
    class GreedySolver
    {
        City[] cities;
        private ArrayList path = new ArrayList();
        private double[,] adjacencyMatrix;
        private Solution bssf;
        public GreedySolver(City[] cities)
        {
            this.cities = cities;
        }
        public string[] solve(int timeLimit)
        {
            bssf = solve();
            
            return returnSolution(bssf.cost, 0, 0);
        }
        public Solution solve()
        {
            HashSet<int> unvisitedCities = new HashSet<int>();
            for (int i = 1; i < cities.Length; i++)
            {
                unvisitedCities.Add(i);
            }
            path.Add(cities[0]);
            while (path.Count < cities.Length)
            {
                City curCity = path[path.Count-1] as City;
                double minDist = double.PositiveInfinity;
                int nextCity = -1;
                foreach (int i in unvisitedCities)
                {
                    double otherDist = curCity.costToGetTo(cities[i]);
                    if (otherDist < minDist)
                    {
                        minDist = otherDist;
                        nextCity = i;
                    }
                }
                path.Add(cities[nextCity]);
                unvisitedCities.Remove(nextCity);
            }

            return new Solution(path, 1);
        }

        public Solution defaultSolveProblem(City[] cities)
        {
            Solution sol;
            int i, swap, temp, count = 0;
            string[] results = new string[3];
            int[] perm = new int[cities.Length];
            ArrayList Route = new ArrayList();
            Random rnd = new Random();
            do
            {
                for (i = 0; i < perm.Length; i++)                                 // create a random permutation template
                    perm[i] = i;
                for (i = 0; i < perm.Length; i++)
                {
                    swap = i;
                    while (swap == i)
                        swap = rnd.Next(0, cities.Length);
                    temp = perm[i];
                    perm[i] = perm[swap];
                    perm[swap] = temp;
                }
                Route.Clear();
                for (i = 0; i < cities.Length; i++)                            // Now build the route using the random permutation 
                {
                    Route.Add(cities[perm[i]]);
                }
                sol = new Solution(Route, 0);
                count++;
            } while (sol.costOfRoute() == double.PositiveInfinity);                // until a valid route is found
            return sol;
        }

        public Solution getBSSF()
        {
            return bssf;
        }
        private double getTotalCostOfPath(State state)
        {
            City here;
            double cost = 0D;

            for (int x = 0; x < state.getRoute().Count - 1; x++)
            {
                here = cities[x];
                cost += here.costToGetTo(cities[x + 1]);
            }

            // go from the last city to the first. 
            here = cities[cities.Count() - 1];
            cost += here.costToGetTo(cities[0]);
            //state.setTotalCostOfTour(cost);
            return cost;
        }
        private string[] returnSolution(double cost, double time, int solutions)
        {
            string[] returnString = new string[3];
            returnString[0] = cost.ToString();
            returnString[1] = time.ToString();
            returnString[2] = solutions.ToString();
            return returnString;
        }
    }
}
