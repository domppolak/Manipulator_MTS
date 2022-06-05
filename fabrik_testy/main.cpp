#include <math.h>
#include <iostream>

#define tol 2 // tolerancja

using namespace std;

double distance1(const double p1[3], const double p2[3]);

int newPositions(double p[][3], const double t[3]);

int main()
{
	double p[3][3] = {{35, 0, 176}, {185, 0, 188}, {200, 0, 163}};
	double t[3] = {50, 150, 183};

	newPositions(p, t);

	cout << "main:" << endl
		 << "P1: " << p[0][0] << "\t" << p[0][1] << "\t" << p[0][2] << endl
		 << "P2: " << p[1][0] << "\t" << p[1][1] << "\t" << p[1][2] << endl
		 << "P3: " << p[2][0] << "\t" << p[2][1] << "\t" << p[2][2] << endl;
}

double distance1(const double p1[3], const double p2[3])
{
	double temp[2];
	double sum;

	temp[0] = p2[0] - p1[0];
	temp[1] = p2[1] - p1[1];

	temp[0] = pow(temp[0], 2);
	temp[1] = pow(temp[1], 2);

	sum = temp[0] + temp[1];
	sum = sqrt(sum);

	return sum;
}

int newPositions(double p[][3], const double t[3])
{
	double d[2];

	if (t[2] < 163)
	{
		cout << "Niżej nie da rady" << endl;
		return 1;
	}

	p[0][2] = t[2] + 13;

	double d2 = distance1(p[0], p[1]);
	double d3 = distance1(p[1], p[2]);
	double dist = distance1(p[0], t);

	d[0] = d2;
	d[1] = d3;

	if (dist > (d2 + d3))
	{
		for (int i = 1; i < 2; i++)
		{
			double r = distance1(t, p[i]);
			double lambda = d[i] / r;
			p[i + 1][0] = (1 - lambda) * p[i][0] + lambda * t[0];
			p[i + 1][1] = (1 - lambda) * p[i][1] + lambda * t[1];
		}
	}
	else
	{
		double b[2];
		b[0] = p[1][0];
		b[1] = p[1][1];

		double difA = distance1(t, p[2]);
		while (difA > tol)
		{
			p[2][0] = t[0];
			//cout << p[2][0] << endl;
			p[2][1] = t[1];
			//cout << p[2][1] << endl;
			for (int i = 1; i > 0; i--)
			{
				double r = distance1(p[i + 1], p[i]);
				double lambda = d[i] / r;
				p[i][0] = (1 - lambda) * p[i + 1][0] + lambda * p[i][0];
				//cout << p[i][0] << endl;
				p[i][1] = (1 - lambda) * p[i + 1][1] + lambda * p[i][1];
				//cout << p[i][1] << endl;
			}
			p[1][0] = b[0];
			p[1][1] = b[1];
			for (int i = 1; i < 2; i++)
			{
				double r = distance1(p[i + 1], p[i]);
				double lambda = d[i] / r;
				p[i + 1][0] = (1 - lambda) * p[i][0] + lambda * p[i + 1][0];
				p[i + 1][1] = (1 - lambda) * p[i][1] + lambda * p[i + 1][1];
			}
			difA = distance1(p[2], t);
			
			cout<<"newPosition:"<<endl
			<<"P1: "<<p[0][0]<<"\t"<<p[0][1]<<"\t"<<p[0][2]<<endl
			<<"P2: "<<p[1][0]<<"\t"<<p[1][1]<<"\t"<<p[1][2]<<endl
			<<"P3: "<<p[2][0]<<"\t"<<p[2][1]<<"\t"<<p[2][2]<<endl;
		}
	}
	return 0;
}