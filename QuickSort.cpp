#include <iostream>
#include <vector>
#include <iterator>

using namespace std;

bool quickSort(vector<int> v,vector<int>::iterator beg,vector<int>::iterator end)
{
	int t,temp;
	vector<int>::iterator i,j;
	
	temp = *beg;
	i=beg;
	j=end;
	
	if(i>j)
		return;

	while(i != j)
	{
		while( i!=j && *j >= temp)
			j--;
		while(i!=j && *i <= temp )
			i++;
	
		if(i < j)
		{
			temp = *i;
			*i = *j;
			*j = temp;
		}
	}

	*beg = v[i];
	v[i] = temp;

	quickSort(v, beg, i-1 );
	quickSort(v, i, end );

	return true;
}

int main(int argc, char const *argv[])
{
	cout<<"Input numbers:"<<endl;
	int temp;
	vector<int> ivec;
	while(cin>>temp)
	{
		ivec.push_back(temp);
	}

	quickSort(ivec, ivec.begin(), ivec.end() );

	fot(auto i:ivec)
	cout<<i<<" ";
	cout<<endl;

	return 0;
}
