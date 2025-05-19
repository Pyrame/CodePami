#include "Matrix22.h"

Matrix22::Matrix22(PRECISION_DATA_TYPE a11, PRECISION_DATA_TYPE a12, PRECISION_DATA_TYPE a21, PRECISION_DATA_TYPE a22) {
    a = new PRECISION_DATA_TYPE[4];
    a[0*2+0] = a11;
    a[0*2+1] = a12;
    a[1*2+0] = a21;
    a[1*2+1] = a22;
}

Matrix22::Matrix22(const Matrix22 &matrix) {
    this->a  = new PRECISION_DATA_TYPE[4];
    for(int i = 0; i < 4; i++)
        this->a[i] = matrix.a[i];
}

Matrix22::Matrix22(Matrix22 &&matrix) {
    this->a = matrix.a;
    matrix.a = nullptr;
}

Position Matrix22::operator*(const Position &position) const {
    return {this->a[0] * position.getX() + this->a[1] * position.getY(), this->a[2] * position.getX() + this->a[3] * position.getY()};
}

const Matrix22 Matrix22::getRotationMatrix(PRECISION_DATA_TYPE theta) {
    return {cos(theta), -sin(theta), sin(theta), cos(theta)};
}

bool Matrix22::operator==(const Matrix22 &rhs) const {
    return a == rhs.a;
}

bool Matrix22::operator!=(const Matrix22 &rhs) const {
    return !(rhs == *this);
}
