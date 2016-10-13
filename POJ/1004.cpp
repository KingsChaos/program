#include <iostream>
using namespace std ;

int main(int argc, char const *argv[])
{
	float sum=0, temp; 
	for (int i = 0; i < 12; ++i)
	{
		cin>>temp
		sum+=temp;
	}
	cout<<setprecision(2)<<sum/12.0<<endl;

	return 0;
}