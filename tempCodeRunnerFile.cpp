#include<iostream>
using namespace std;
int main()
{
    int nums[7]={2,3,4,5,6,7,3};
    int val=3;
    int count=0;
        for(int i=0;i<7;i++)
        {
            if(nums[i]!=val)
            {
                nums[count]=nums[i];
                count++;
            }
            
        }
            cout<<count<<endl;

     for(int i=0;i<sizeof(nums)/sizeof(nums[0]);i++)
     {
        
        cout<<nums[i];
     }
             return 0;
}