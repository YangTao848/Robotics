/* A 2D robot arm with 3 links and the length of links are 2, 3 and 1 with fixed base (0.0) . The arm configuration is specified by theta1, theta2, theta3.

- Write a function to compute the coordinate of the end effector (e.g. the end point of the arm) for any given configuration(theta1, theta2, theta3).
- Given an end effector location (4, 0), compute theta1, theta2, theta3.

*/
#include <iostream>
#include <cmath>
using namespace std;
const double PI = 3.1415926539;

struct config
{
	config(double t1, double t2, double t3) : theta1(t1), theta2(t2), theta3(t3) {}
	double theta1, theta2, theta3;
};

pair<double, double> FK(config& con)
{
	pair<double, double> ef;    //end-effector config(theta1,theta2,theta3)---> pair(x,y)
	ef.first = 2.0*cos(con.theta1) + 3.0*cos(con.theta1 + con.theta2) + cos(con.theta1 + con.theta2 + con.theta3);
	ef.second = 2.0*sin(con.theta1) + 3.0*sin(con.theta1 + con.theta2) + sin(con.theta1 + con.theta2 + con.theta3);
	return ef;
}

config IK(pair<double, double> ef)
{
	//redundant, there are infinite solutions, I just fix theta1, try all possible values for theta1
	double t1 = 0.0, step = PI / 50;
	for (int i = 0; i < 100; i++)
	{
		t1 += step;
		double x = ef.first - 2.0*cos(t1), y = ef.second - 2.0*sin(t1);
		//Then the equation becomes 3c1+c2=x, 3s1+s2=y. Where 1:=theta1+theta2    2:=theta1+theta2+theta3
		//9+1+6c1c2+6s1s2=x^2+y^2
		//6cos(theta3)=x^2+y^2-10
		//We need 16.0>=x^2+y^2>=4.0 to get solution
		double ab = fabs(x*x + y*y - 10.0);
		if (ab - 6.0 < -1e-3)          //solution found
		{
			double t3 = acos((x*x + y*y - 10.0) / 6.0);
			// 3cos(t1+t2)+cos(t1+t2)cos(t3)-sin(t1+t2)sin(t3)=x
			// 3sin(t1+t2)+sin(t1+t2)cos(t3)+sin(t3)cos(t1+t2)=y
			// equation2 * (3+cost3) - equation1* sint3
			double t2 = asin((y*(3.0 + cos(t3)) - x*sin(t3)) / (10.0 + 6.0*cos(t3)));
			t2 -= t1;           //Because the t2 above is actually t1+t2
			return config(t1, t2, t3);
		}
	}
	return config(0.0, 0.0, 0.0);      //not reachable
}

	// To execute C++, please define "int main()"
	int main()
	{
		auto ef = make_pair(4.0, 1.0);
		auto c = IK(ef);
		auto ef_new = FK(c);
		cout << "x: " << ef_new.first << ", y: " << ef_new.second << endl;
		getchar();
		return 0;
	}