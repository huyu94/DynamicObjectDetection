#include <iostream>
#include <Eigen/Eigen>
#include <munkres.h>
#include <time.h>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
using namespace std;

void func(float x,float y, float z){
    cout << x << " " << y << " " << z << endl;

}



int main(int argc, char** argv){
    vector<int> c(10,2);

    int &a = c[3];

    a = 5;
    for(auto &t : c)
    {
        cout << t << endl;
    }


    return 0;

}