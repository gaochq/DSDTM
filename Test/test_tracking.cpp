#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;


void printResult(vector<int>& vecInt, int t[]){
    for(int i = 0; i < vecInt.size(); ++i){
        if(vecInt[i] == 1){
            cout << t[i] << " ";
        }
    }
    cout << endl;
}

bool compare(int a, int b){
    if(a > b){
        return true;
    }else{
        return false;
    }
}

void combination(int t[], int c, int total){
    //initial first combination like:1,1,0,0,0  
    vector<int> vecInt(total,0);
    for(int i = 0; i < c; ++i){
        vecInt[i] = 1;
    }

    printResult(vecInt, t);

    for(int i = 0; i < total - 1; ++i)
    {
        if(vecInt[i] == 1 && vecInt[i+1] == 0)
        {
            //1. first exchange 1 and 0 to 0 1  
            swap(vecInt[i], vecInt[i+1]);

            //2.move all 1 before vecInt[i] to left  
            sort(vecInt.begin(),vecInt.begin() + i, compare);

            //after step 1 and 2, a new combination is exist  
            printResult(vecInt, t);

            //try do step 1 and 2 from front  
            i = -1;
        }
    }

}


int main(int argc, char* argv[])
{
    const int N = 100;
    int t[100]={0};
    for (int i = 0; i < 100; ++i)
    {
        t[i] = i+1;
    }
    combination(t, 8, N);
    system("pause");
    return 0;
}  