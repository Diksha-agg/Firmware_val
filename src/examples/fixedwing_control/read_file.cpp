#include<iostream>
#include<fstream>
#include <vector>
#include <cstring>
#include <string>

#include <cstring>
#include <float.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <time.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
using namespace std;


double read_file()
{
	fstream filemy;
	filemy.open("test.csv",ios::in);
	string line,param_name;

	while(filemy)
	{cout<<getline(filemy,line);
	cout<<line;}
	double thh=0;	
	return thh;

}


