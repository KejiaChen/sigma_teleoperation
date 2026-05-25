/** ------------------------- Revision Code History -------------------
*** Programming Language: C++
*** Description: Data Recorde
*** Released Date: Feb. 2021
*** Hamid Sadeghian
*** h.sadeghian@eng.ui.ac.ir
----------------------------------------------------------------------- */

#pragma once

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>

using namespace std;
using namespace Eigen;

class Recorder {
   public:
    Recorder(double t_rec, double sampleTime, int NoDataRec = 10, std::string name = "DATA");
    ~Recorder();

    void addToRec(int value);
    void addToRec(double value);
    void addToRec(double array[], int sizeofarray);
    void addToRec(std::array<double, 3> array);
    void addToRec(std::array<double, 6> array);
    void addToRec(std::array<double, 7> array);

    void addToRec(Vector3d& vector);
    void saveData();
    void next();

    void getDAT(Matrix<double, Dynamic, 1>& _Buffer, int rowNum);

   private:
    int _index;
    int _columnindex;
    int _rowindex;
    double _t_rec;
    int _NoDataRec;
    std::string _name;
    Matrix<double, Dynamic, Dynamic> _DAT;
};