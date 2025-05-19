#ifndef CODEPAMI_MATRIX22_H
#define CODEPAMI_MATRIX22_H
#include "Position.h"


class Matrix22 {
    PRECISION_DATA_TYPE* a = nullptr;
public:
    Matrix22(PRECISION_DATA_TYPE a11, PRECISION_DATA_TYPE a12, PRECISION_DATA_TYPE a21, PRECISION_DATA_TYPE a22);
    //Copy constructor
    Matrix22(const Matrix22& matrix);
    //Move constructor
    Matrix22(Matrix22&& matrix);

    Position operator*(const Position& position) const;

    static const Matrix22 getRotationMatrix(PRECISION_DATA_TYPE theta);

    bool operator==(const Matrix22 &rhs) const;

    bool operator!=(const Matrix22 &rhs) const;


};


#endif //CODEPAMI_MATRIX22_H
