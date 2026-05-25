#include "franka_control/recorder.h"

Recorder::Recorder(double t_rec, double sampleTime, int NoDataRec, std::string name) {
    _DAT.resize((int)(t_rec / sampleTime + 2), NoDataRec);
    _DAT.setZero();
    _rowindex = 0;
    _columnindex = 0;
    _t_rec = t_rec;
    _name = name;
    _NoDataRec = NoDataRec;
};
Recorder::~Recorder() { saveData(); };

void Recorder::getDAT(Matrix<double, Dynamic, 1>& _Buffer, int rowNum) { _Buffer = _DAT.row(rowNum); }

void Recorder::addToRec(int value) {
    _DAT(_rowindex, _columnindex) = value;
    _columnindex++;
}
void Recorder::addToRec(double value) {
    _DAT(_rowindex, _columnindex) = value;
    _columnindex++;
}
void Recorder::addToRec(double array[], int sizeofarray) {
    // cout << "TODO: size of array is manual" << endl;
    for (int i = 0; i < sizeofarray; i++) {
        _DAT(_rowindex, _columnindex) = array[i];
        _columnindex++;
    }
};
void Recorder::addToRec(std::array<double, 7> array) {
    // cout << "TODO: size of array is manual" << endl;
    for (int i = 0; i < 7; i++) {
        _DAT(_rowindex, _columnindex) = array[i];
        _columnindex++;
    }
};

void Recorder::addToRec(std::array<double, 6> array) {
    // cout << "TODO: size of array is manual" << endl;
    for (int i = 0; i < 6; i++) {
        _DAT(_rowindex, _columnindex) = array[i];
        _columnindex++;
    }
};
void Recorder::addToRec(std::array<double, 3> array) {
    // cout << "TODO: size of array is manual" << endl;
    for (int i = 0; i < 3; i++) {
        _DAT(_rowindex, _columnindex) = array[i];
        _columnindex++;
    }
};
void Recorder::addToRec(Vector3d& vector) {
    for (int i = 0; i < vector.size(); i++) {
        _DAT(_rowindex, _columnindex) = vector[i];
        _columnindex++;
    }
};

void Recorder::saveData() {
    std::ofstream myfile;
    myfile.open(_name + ".m");
    myfile << _name << "m"
           << "=[" << _DAT << "];\n";
    myfile.close();
    cout << "\nRecorder: Data was written successfully\n";
};
void Recorder::next() {
    _rowindex++;
    _columnindex = 0;
}