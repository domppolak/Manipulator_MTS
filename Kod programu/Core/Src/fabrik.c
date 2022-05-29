/*
 * fabrik.c
 *
 *  Created on: 27 maj 2022
 *      Author: maciej
 */

#include "fabrik.h"
#include <math.h>

#define tol 1 // tolerancja

float distance(const float p1[3], const float p2[3])
{
	float temp[3];

	temp[0]=p2[0]-p1[0];
	temp[1]=p2[1]-p1[1];
	temp[2]=p2[2]-p1[2];

	return abs(pow(pow(temp[0],2)+pow(temp[1],2)+pow(temp[2],2),1/3));
}

void newPositions(float p[3][3], float t[3])
{
	float root[3]={0,0,0};
	float d[3];

	float d1=distance(root,p[0]);
	float d1max=380;
	float d2=distance(p[0],p[1]);
	float d3=distance(p[1],p[2]);
	float dist=distance(root,t);

	d[0]=d1;
	d[1]=d2;
	d[2]=d3;

	if(dist>d1max+d2+d3)
	{
		for (int i=0;i<2;i++)
			{
				float r=distance(t,p[i]);
				float lambda=d[i]/r;
				p[i+1][0]=(1-lambda)*p[i][0]+lambda*t[0];
				p[i+1][1]=(1-lambda)*p[i][1]+lambda*t[1];
				p[i+1][2]=(1-lambda)*p[i][2]+lambda*t[2];
			}
	}
	float b[3];
	b[0]=p[0][0];
	b[1]=p[0][1];
	b[2]=p[0][2];

	float difA=distance(t,p[2]);
	while(difA>tol)
	{
		p[2][0]=t[0];
		p[2][1]=t[1];
		p[2][2]=t[2];
		for(int i=1;i>=0;i--)
		{
			float r=distance(p[i+1],p[i]);
			float lambda = d[i]/r;
			p[i][0]=(1-lambda)*p[i+1][0]+lambda*p[i][0];
			p[i][1]=(1-lambda)*p[i+1][1]+lambda*p[i][1];
			p[i][2]=(1-lambda)*p[i+1][2]+lambda*p[i][2];
		}
		p[0][0]=b[0];
		p[0][1]=b[1];
		p[0][2]=b[2];
		for(int i=0;i<2;i++)
		{
			float r=distance(p[i+1],p[i]);
			float lambda=d[i]/r;
			p[i+1][0]=(1-lambda)*p[i][0]+lambda*p[i+1][0];
			p[i+1][1]=(1-lambda)*p[i][1]+lambda*p[i+1][1];
			p[i+1][2]=(1-lambda)*p[i][2]+lambda*p[i+1][2];
		}
		difA=distance(p[2],t);
	}
}
