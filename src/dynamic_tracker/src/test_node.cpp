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

    Eigen::Vector3d a(1,2,3);
    func(a[0],a[1],a[2]);

    return 0;

}