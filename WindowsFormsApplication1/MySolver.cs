using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TSP
{
    public class MySolver
    {
        Solution bssf;
        List<City> cities = new List<City>();
        HashSet<int> remainingCities;
        public MySolver(City[] cities)
        {
            foreach (City city in cities)
            {
                this.cities.Add(city);
            }
        }
        public string[] solve(int timeout)
        {
            Stopwatch timer = new Stopwatch();
            timer.Start();
            bssf = Solve(cities);
            return returnSolution(bssf.cost, timer.Elapsed.ToString(), 0);
        }
        public Solution getBSSF()
        {
            return bssf;
        }
        /**
        public ConvexHullSolver(System.Drawing.Graphics g, System.Windows.Forms.PictureBox pictureBoxView)
        {
            this.g = g;
            this.pictureBoxView = pictureBoxView;
        }

        public void Refresh()
        {
            // Use this especially for debugging and whenever you want to see what you have drawn so far
            pictureBoxView.Refresh();
        }

        public void Pause(int milliseconds)
        {
            // Use this especially for debugging and to animate your algorithm slowly
            pictureBoxView.Refresh();
            System.Threading.Thread.Sleep(milliseconds);
        }
        **/

        public Solution Solve(List<City> pointList)
        {
            //Sorting normally takes (n log(n)) time
            pointList.Sort((x, y) => x.X.CompareTo(y.X));
            LinkedCList hull = convexSort(pointList, 0);
            remainingCities = getRemainingCitiesBad(hull, pointList.Count - 1);
            connectInnerToOuter(hull, remainingCities);
            ArrayList cityHull = new ArrayList();
            LinkedCList.CPoint cur = hull.root;
            do
            {
                cityHull.Add(cities[cur.cityInt]);
                cur = cur.next;
            } while (!cur.Equals(hull.root));

            return new Solution(cityHull, 0);
        }

        

/*
* This part of our algorithm divides the problem in half,
* only stopping once we've hit a list of 2 or 3 points.
* The base cases also take constant time
* O(1)
*/
        private LinkedCList convexSort(List<City> points, int beginIndex)
        {
            int size = points.Count;
            if (size < 4)
            {
                if (size == 2)
                {
                    return make2PointList(points, beginIndex);
                }
                else if (size == 3)
                {
                    return make3PointList(points, beginIndex);
                }
                else
                {
                    //Size shouldn't be 1!
                    throw new InvalidOperationException();
                }
            }
            //Divide problem in half
            return merge(convexSort(points.GetRange(0, (size - 1) / 2 + 1), beginIndex), convexSort(points.GetRange((size - 1) / 2 + 1, size / 2),beginIndex+((size-1)/2+1)));
            //beginIndex+(size+1)/2-1)
            //endIndex
        }
        /*
        * This is the work that is done to join together each piece of recursion
        * Most work is constant, but we do have while loops checking for upper
        * and lower tangents.  Theoritically, each of these two separate loops
        * could end up checking every point in these 2 convex hulls.
        * Note that we are only sending back the points that are in the convex hull.
        * At worst, this is all of the points in the list, but on average will likely
        * be log n.
        * O(n)
        */
        private LinkedCList merge(LinkedCList list1, LinkedCList list2)
        {
            LinkedCList.CPoint upperLeftTan = list1.right;
            LinkedCList.CPoint upperRightTan = list2.root;
            //while both upper and lower tangents are not set
            while (!isUpperSet(upperLeftTan, upperRightTan))
            {
                //while upper tanget is not set
                while (!isUpperLeftSet(upperLeftTan, upperRightTan))
                {
                    upperLeftTan = upperLeftTan.prev;
                }
                //while right tanget is not set
                while (!isUpperRightSet(upperLeftTan, upperRightTan))
                {
                    upperRightTan = upperRightTan.next;
                }
            }
            LinkedCList.CPoint lowerLeftTan = list1.right;
            LinkedCList.CPoint lowerRightTan = list2.root;
            while (!isLowerSet(lowerLeftTan, lowerRightTan))
            {
                while (!isLowerLeftSet(lowerLeftTan, lowerRightTan))
                {
                    lowerLeftTan = lowerLeftTan.next;
                }
                while (!isLowerRightSet(lowerLeftTan, lowerRightTan))
                {
                    lowerRightTan = lowerRightTan.prev;
                }
            }
            //Link up the tangents, leaving behind points not needed
            upperLeftTan.next = upperRightTan;
            upperRightTan.prev = upperLeftTan;
            lowerLeftTan.prev = lowerRightTan;
            lowerRightTan.next = lowerLeftTan;
            list1.right = list2.right;
            return list1;
        }
        /*
        *   The work for this is constant
        *   O(1)
        */
        private bool isUpperSet(LinkedCList.CPoint left, LinkedCList.CPoint right)
        {
            return isUpperLeftSet(left, right) && isUpperRightSet(left, right);
        }
        /*
        *   The work for this is constant
        *   O(1)
        */
        private bool isLowerSet(LinkedCList.CPoint left, LinkedCList.CPoint right)
        {
            return isLowerLeftSet(left, right) && isLowerRightSet(left, right);
        }
        /*
        *   The work for this is constant
        *   O(1)
        */
        private bool isUpperLeftSet(LinkedCList.CPoint left, LinkedCList.CPoint right)
        {
            double curSlope = (right.y - left.y) / (right.x - left.x);
            double nextLeftSlope = (right.y - left.prev.y) / (right.x - left.prev.x);
            return nextLeftSlope < curSlope;
        }
        /*
        *   The work for this is constant
        *   O(1)
        */
        private bool isUpperRightSet(LinkedCList.CPoint left, LinkedCList.CPoint right)
        {
            double curSlope = (right.y - left.y) / (right.x - left.x);
            double nextRightSlope = (right.next.y - left.y) / (right.next.x - left.x);
            //If nextRightSlope is less than curSlope, we are as far as we can go on the right
            return nextRightSlope > curSlope;
        }
        /*
        *   The work for this is constant
        *   O(1)
        */
        private bool isLowerLeftSet(LinkedCList.CPoint left, LinkedCList.CPoint right)
        {
            double curSlope = (right.y - left.y) / (right.x - left.x);
            double nextLeftSlope = (right.y - left.next.y) / (right.x - left.next.x);
            return nextLeftSlope > curSlope;
        }
        /*
        *   The work for this is constant
        *   O(1)
        */
        private bool isLowerRightSet(LinkedCList.CPoint left, LinkedCList.CPoint right)
        {
            double curSlope = (right.y - left.y) / (right.x - left.x);
            double nextRightSlope = (right.prev.y - left.y) / (right.prev.x - left.x);
            return nextRightSlope < curSlope;
        }
        /*
        *   The work for this is constant
        *   O(1)
        */
        private LinkedCList make2PointList(List<City> points, int begin)
        {
            LinkedCList list = new LinkedCList();
            LinkedCList.CPoint left = new LinkedCList.CPoint(points[0].X, points[0].Y, begin);
            LinkedCList.CPoint right = new LinkedCList.CPoint(points[1].X, points[1].Y, begin+1);
            left.next = right;
            left.prev = right;
            right.next = left;
            right.prev = left;
            list.root = left;
            list.right = right;
            return list;
        }
        /*
        *   The work for this is constant
        *   O(1)
        */
        private LinkedCList make3PointList(List<City> points, int begin)
        {
            LinkedCList list = new LinkedCList();
            LinkedCList.CPoint left = new LinkedCList.CPoint(points[0].X, points[0].Y, begin);
            LinkedCList.CPoint point1 = new LinkedCList.CPoint(points[1].X, points[1].Y,begin+1);
            LinkedCList.CPoint point2 = new LinkedCList.CPoint(points[2].X, points[2].Y,begin+2);
            //Start of list is leftmost point
            list.root = left;
            //Right is set to rightmost point
            list.right = point2;

            double slope1 = (point1.y - left.y) / (point1.x - left.x);
            double slope2 = (point2.y - left.y) / (point2.x - left.x);
            if (slope1 < slope2)
            {
                //Setup pointers correctly
                left.next = point1;
                left.prev = point2;
                point1.next = point2;
                point1.prev = left;
                point2.next = left;
                point2.prev = point1;
            }
            else
            {
                left.next = point2;
                left.prev = point1;
                point1.next = left;
                point1.prev = point2;
                point2.next = point1;
                point2.prev = left;
            }
            return list;
        }
        /*
        *   Draw points does constant work for each node in the convex hull.
        *   Time: O(n)
        */
        /**
        private void drawPoints(LinkedCList list, int delay)
        {
            LinkedCList.CPoint cur = list.root;
            do
            {
                this.Pause(delay);
                Pen pen = new Pen(Color.Black, 2);
                g.DrawLine(pen, (float)cur.x, (float)cur.y, (float)cur.next.x, (float)cur.next.y);
                cur = cur.next;
            } while (!cur.Equals(list.root));
        }
        **/

        private string[] returnSolution(double cost, String time, int solutions)
        {
            string[] returnString = new string[3];
            returnString[0] = cost.ToString();
            returnString[1] = time;
            returnString[2] = solutions.ToString();
            return returnString;
        }
        private LinkedCList connectInnerToOuter(LinkedCList hull, HashSet<int> remainingCities)
        {
            while (remainingCities.Count > 0)
            {
                Dictionary<int, double> cityCost = new Dictionary<int, double>();
                Dictionary<int, LinkedCList.CPoint> cityToNode = new Dictionary<int, LinkedCList.CPoint>();
                foreach (int cityI in remainingCities)
                {
                    City city = cities[cityI];
                    LinkedCList.CPoint cur = hull.root;
                    double minDist = double.PositiveInfinity;
                    LinkedCList.CPoint minNode = null;
                    do
                    {
                        double ij = cities[cur.cityInt].costToGetTo(city);
                        double jk = city.costToGetTo(cities[cur.next.cityInt]);
                        double ik = cities[cur.cityInt].costToGetTo(cities[cur.next.cityInt]);
                        if ((ij + jk - ik) < minDist)
                        {
                            minDist = ij + jk - ik;
                            minNode = cur;
                        }
                        cur = cur.next;
                    } while (!cur.Equals(hull.root));
                    cityCost[cityI] = minDist;
                    cityToNode[cityI] = minNode;
                }
                double minVal = double.PositiveInfinity;
                int minI = -1;
                foreach (KeyValuePair<int, double> entry in cityCost)
                {
                    if (entry.Value < minVal)
                    {
                        minVal = entry.Value;
                        minI = entry.Key;
                    }
                }
                LinkedCList.CPoint mid = new LinkedCList.CPoint(cities[minI].X, cities[minI].Y, minI);
                mid.prev = cityToNode[minI];
                mid.next = cityToNode[minI].next;
                cityToNode[minI].next.prev = mid;
                cityToNode[minI].next = mid;
                remainingCities.Remove(minI);
            }
            return hull;
        }
        private HashSet<int> getRemainingCitiesBad(LinkedCList hull, int totalCities)
        {
            HashSet<int> remainingCities = new HashSet<int>();
            for (int i = 0; i < totalCities; i++)
            {
                bool seen = false;
                LinkedCList.CPoint cur = hull.root;
                do
                {
                    if (cur.cityInt == i)
                    {
                        seen = true;
                        break;
                    }
                    cur = cur.next;
                } while (!cur.Equals(hull.root));
                if (!seen)
                {
                    remainingCities.Add(i);
                }
            }
            return remainingCities;
        }
    }

    class LinkedCList
    {
        public CPoint root;
        public CPoint right;

        public class CPoint
        {
            public readonly double x;
            public readonly double y;
            public CPoint next;
            public CPoint prev;
            public int cityInt;

            public CPoint(double x, double y, int cityInt)
            {
                this.x = x;
                this.y = y;
                this.cityInt = cityInt;
            }
        }

    }
}
