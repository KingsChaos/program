#include <iostream>
#include <cmath>
using namespace std;

int main(int argc, char const *argv[])
{
	int i, n;
	float x,y,year;
	cin >>n;

	for (int i = 0; i < n; i++)
	{
		cin>>x>>y;
		year=M_PI*(x*x+y*y)/100;
		cout<<"Property "<<i+1<<": This property will begin eroding in year "<<int(year)+1<<endl;
	}
	cout<<"END OF OUTPUT."<<endl;

	return 0;
}