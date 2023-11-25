#include <iostream>
#include <Eigen/Eigen>
#include <munkres.h>
#include <time.h>

int main(){
    /*
    Matrix<float> matrix_cost(6,5);
    matrix_cost(0,0) = 20;
    matrix_cost(0,1) = 1000000;
    matrix_cost(0,2) = 18;
    matrix_cost(0,3) = 24;
    matrix_cost(0,4) = 25;
    matrix_cost(1,0) = 18;
    matrix_cost(1,1) = 20;
    matrix_cost(1,2) = 12;
    matrix_cost(1,3) = 14;
    matrix_cost(1,4) = 15;
    matrix_cost(2,0) = 21;
    matrix_cost(2,1) = 23;
    matrix_cost(2,2) = 25;
    matrix_cost(2,3) = 27;
    matrix_cost(2,4) = 26;
    matrix_cost(3,0) = 17;
    matrix_cost(3,1) = 18;
    matrix_cost(3,2) = 21;
    matrix_cost(3,3) = 23;
    matrix_cost(3,4) = 22;
    matrix_cost(4,0) = 19;
    matrix_cost(4,1) = 22;
    matrix_cost(4,2) = 16;
    matrix_cost(4,3) = 21;
    matrix_cost(4,4) = 20;
    matrix_cost(5,0) = 39;
    matrix_cost(5,1) = 22;
    matrix_cost(5,2) = 6;
    matrix_cost(5,3) = 21;
    matrix_cost(5,4) = 15;
    */
    Matrix<float> matrix_cost(2,6);
    matrix_cost(0,0) = 20;
    matrix_cost(0,1) = 1000000;
    matrix_cost(0,2) = 18;
    matrix_cost(0,3) = 24;
    matrix_cost(0,4) = 25;
    matrix_cost(0,5) = 50;
    matrix_cost(1,0) = 1000000;
    matrix_cost(1,1) = 999999;
    matrix_cost(1,2) = 1000000;
    matrix_cost(1,3) = 1000000;
    matrix_cost(1,4) = 1000000;
    matrix_cost(1,5) = 1000000;
    Munkres<float> munkres_solver;
    clock_t t1,t2;
    t1 = clock();
    munkres_solver.solve(matrix_cost);
    t2 = clock();
    std::cout << "Time taken by Munkres: " << (double)(t2-t1)/CLOCKS_PER_SEC << std::endl;
    for(size_t row=0;row < matrix_cost.rows();row++){
        for(size_t col=0;col < matrix_cost.columns();col++){
            if(matrix_cost(row,col) == 0.0){
                std::cout << "(" << row << "," << col << ") ";
            }
            std::cout << matrix_cost(row,col) << " ";
        }
            std::cout << std::endl;
    }
        
}