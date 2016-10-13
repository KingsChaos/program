	#include <iostream>
	using namespace std;

	int main(int argc, char const *argv[])
	{
		float n;
		cin >> n;
		float length=0,i=1;
		while(n>0 && n<5.2 && length<n)
		{
			i++;
			length = length + 1/i;
			
		}
		cout << i-1<<" card(s)"<<endl;

		return 0;
	}