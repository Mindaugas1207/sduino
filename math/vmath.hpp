
#ifndef INC_VMATH_HPP_
#define INC_VMATH_HPP_

#include "pico/stdlib.h"
#include <math.h>
#include <tuple>
#include <algorithm>
#include <cstdint>

namespace vmath
{
    template <typename T = double, std::size_t iterations = 2> inline T Inv_sqrt(T x)
    {
        static_assert(std::is_floating_point<T>::value, "T must be floating point");
        static_assert(iterations > 0, "itarations must be more than 0");
        typedef typename std::conditional<sizeof(T) == 8, std::int64_t, std::int32_t>::type Tint;

        T y = x;
        T x2 = y * (T)0.5;
        Tint i = *(Tint *)&y;
        i = (sizeof(T) == 8 ? 0x5fe6eb50c7b537a9 : 0x5f3759df) - (i >> 1);
        y = *(T *)&i;

        std::size_t it = iterations;
        do
        {
            y = y * ((T)1.5 - (x2 * y * y));
        } while (it--);

        return y;
    }

    inline double ConstrainAngle(double x)
    {
        x = std::fmod(x + M_PI, 2 * M_PI);

        if (x < 0) return x + M_PI;
        return x - M_PI;
    }

    inline double  UnwrapAngle(double previous_angle, double new_angle)
    {
        double d = new_angle - previous_angle;
        d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
        return previous_angle + d;
    }

    template <size_t size>
    struct vector_s
    {
        double data[size];

        const size_t Size = size;

        double& operator[] (const size_t& index) { return data[index]; }
        const double& operator[] (const size_t& index) const { return data[index]; }

        //Math operators
        vector_s& operator=  (const vector_s& rhs) {for (size_t i = 0; i < size; i++) { this->data[i]  = rhs.data[i]; } return *this; }; //static_assert(std::is_same<vector_s<size>, decltype(rhs)>::value, "vector mismatch"); 
        vector_s& operator+= (const vector_s& rhs) {for (size_t i = 0; i < size; i++) { this->data[i] += rhs.data[i]; } return *this; }; //static_assert(std::is_same<vector_s<size>, decltype(rhs)>::value, "vector mismatch"); 
        vector_s& operator-= (const vector_s& rhs) {for (size_t i = 0; i < size; i++) { this->data[i] -= rhs.data[i]; } return *this; }; //static_assert(std::is_same<vector_s<size>, decltype(rhs)>::value, "vector mismatch"); 
        vector_s& operator*= (const vector_s& rhs) {for (size_t i = 0; i < size; i++) { this->data[i] *= rhs.data[i]; } return *this; }; //static_assert(std::is_same<vector_s<size>, decltype(rhs)>::value, "vector mismatch"); 
        vector_s& operator/= (const vector_s& rhs) {for (size_t i = 0; i < size; i++) { this->data[i] /= rhs.data[i]; } return *this; }; //static_assert(std::is_same<vector_s<size>, decltype(rhs)>::value, "vector mismatch"); 
        vector_s& operator=  (const double& rhs) {for (size_t i = 0; i < size; i++) { this->data[i]  = rhs; } return *this; };
        vector_s& operator+= (const double& rhs) {for (size_t i = 0; i < size; i++) { this->data[i] += rhs; } return *this; };
        vector_s& operator-= (const double& rhs) {for (size_t i = 0; i < size; i++) { this->data[i] -= rhs; } return *this; };
        vector_s& operator*= (const double& rhs) {for (size_t i = 0; i < size; i++) { this->data[i] *= rhs; } return *this; };
        vector_s& operator/= (const double& rhs) {for (size_t i = 0; i < size; i++) { this->data[i] /= rhs; } return *this; };
        friend vector_s operator+ (vector_s lhs, const vector_s& rhs) { lhs += rhs; return lhs; }
        friend vector_s operator- (vector_s lhs, const vector_s& rhs) { lhs -= rhs; return lhs; }
        friend vector_s operator* (vector_s lhs, const vector_s& rhs) { lhs *= rhs; return lhs; }
        friend vector_s operator/ (vector_s lhs, const vector_s& rhs) { lhs /= rhs; return lhs; }
        friend vector_s operator+ (vector_s lhs, const double& rhs) { lhs += rhs; return lhs; }
        friend vector_s operator- (vector_s lhs, const double& rhs) { lhs -= rhs; return lhs; }
        friend vector_s operator* (vector_s lhs, const double& rhs) { lhs *= rhs; return lhs; }
        friend vector_s operator/ (vector_s lhs, const double& rhs) { lhs /= rhs; return lhs; }

        //double Length(void) {}

        vector_s Normalise(void) { double sqrsum = 0; for (size_t i = 0; i < size; i++) { double tmp = this->data[i]; sqrsum += tmp * tmp; } return *this * Inv_sqrt(sqrsum); };
    };

    inline vector_s<4> HamiltonProduct(vector_s<4> lhs, const vector_s<4>& rhs)
    {
        lhs[0] = lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2] - lhs[3] * rhs[3];
        lhs[1] = lhs[0] * rhs[1] + lhs[1] * rhs[0] + lhs[2] * rhs[3] - lhs[3] * rhs[2];
        lhs[2] = lhs[0] * rhs[2] - lhs[1] * rhs[3] + lhs[2] * rhs[0] + lhs[3] * rhs[1];
        lhs[3] = lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1] + lhs[3] * rhs[0];
        return lhs;
    }

    inline vector_s<4> HamiltonProduct(vector_s<4> lhs, const vector_s<3>& rhs)
    {
        lhs[0] = -lhs[1] * rhs[0] - lhs[2] * rhs[1] - lhs[3] * rhs[2];
        lhs[1] =  lhs[0] * rhs[0] + lhs[2] * rhs[2] - lhs[3] * rhs[1];
        lhs[2] =  lhs[0] * rhs[1] - lhs[1] * rhs[2] + lhs[3] * rhs[0];
        lhs[3] =  lhs[0] * rhs[2] + lhs[1] * rhs[1] - lhs[2] * rhs[0];
        return lhs;
    }

    //Vector 3 specialization
    // template <> template <> inline vector_s<3>& vector_s<3>::operator=  (const vector_s<3>& rhs) { this->data[0]  = rhs.data[0]; this->data[1]  = rhs.data[1]; this->data[2]  = rhs.data[2]; return *this; }
    // template <> template <> inline vector_s<3>& vector_s<3>::operator+= (const vector_s<3>& rhs) { this->data[0] += rhs.data[0]; this->data[1] += rhs.data[1]; this->data[2] += rhs.data[2]; return *this; }
    // template <> template <> inline vector_s<3>& vector_s<3>::operator-= (const vector_s<3>& rhs) { this->data[0] -= rhs.data[0]; this->data[1] -= rhs.data[1]; this->data[2] -= rhs.data[2]; return *this; }
    // template <> template <> inline vector_s<3>& vector_s<3>::operator*= (const vector_s<3>& rhs) { this->data[0] *= rhs.data[0]; this->data[1] *= rhs.data[1]; this->data[2] *= rhs.data[2]; return *this; }
    // template <> template <> inline vector_s<3>& vector_s<3>::operator/= (const vector_s<3>& rhs) { this->data[0] /= rhs.data[0]; this->data[1] /= rhs.data[1]; this->data[2] /= rhs.data[2]; return *this; }
    // template <> template <typename Trhs> inline vector_s<3>& vector_s<3>::operator=  (const Trhs& rhs) { this->data[0]  = rhs; this->data[1]  = rhs; this->data[2]  = rhs; return *this; }
    // template <> template <typename Trhs> inline vector_s<3>& vector_s<3>::operator+= (const Trhs& rhs) { this->data[0] += rhs; this->data[1] += rhs; this->data[2] += rhs; return *this; }
    // template <> template <typename Trhs> inline vector_s<3>& vector_s<3>::operator-= (const Trhs& rhs) { this->data[0] -= rhs; this->data[1] -= rhs; this->data[2] -= rhs; return *this; }
    // template <> template <typename Trhs> inline vector_s<3>& vector_s<3>::operator*= (const Trhs& rhs) { this->data[0] *= rhs; this->data[1] *= rhs; this->data[2] *= rhs; return *this; }
    // template <> template <typename Trhs> inline vector_s<3>& vector_s<3>::operator/= (const Trhs& rhs) { this->data[0] /= rhs; this->data[1] /= rhs; this->data[2] /= rhs; return *this; }

    // template <> inline vector_s<3> vector_s<3>::Normalise(void) { return *this * Inv_sqrt(this->data[0] * this->data[0] + this->data[1] * this->data[1] + this->data[2] * this->data[2]); }

    //Vector 4 specialization
    // template <> template <> inline vector_s<4>& vector_s<4>::operator=  (const vector_s<4>& rhs) { this->data[0]  = rhs.data[0]; this->data[1]  = rhs.data[1]; this->data[2]  = rhs.data[2]; this->data[3]  = rhs.data[3]; return *this; }
    // template <> template <> inline vector_s<4>& vector_s<4>::operator+= (const vector_s<4>& rhs) { this->data[0] += rhs.data[0]; this->data[1] += rhs.data[1]; this->data[2] += rhs.data[2]; this->data[3] += rhs.data[3]; return *this; }
    // template <> template <> inline vector_s<4>& vector_s<4>::operator-= (const vector_s<4>& rhs) { this->data[0] -= rhs.data[0]; this->data[1] -= rhs.data[1]; this->data[2] -= rhs.data[2]; this->data[3] -= rhs.data[3]; return *this; }
    // template <> template <> inline vector_s<4>& vector_s<4>::operator*= (const vector_s<4>& rhs) { this->data[0] *= rhs.data[0]; this->data[1] *= rhs.data[1]; this->data[2] *= rhs.data[2]; this->data[3] *= rhs.data[3]; return *this; }
    // template <> template <> inline vector_s<4>& vector_s<4>::operator/= (const vector_s<4>& rhs) { this->data[0] /= rhs.data[0]; this->data[1] /= rhs.data[1]; this->data[2] /= rhs.data[2]; this->data[3] /= rhs.data[3]; return *this; }
    // template <> template <typename Trhs> inline vector_s<4>& vector_s<4>::operator=  (const Trhs& rhs) { this->data[0]  = rhs; this->data[1]  = rhs; this->data[2]  = rhs; this->data[3]  = rhs; return *this; }
    // template <> template <typename Trhs> inline vector_s<4>& vector_s<4>::operator+= (const Trhs& rhs) { this->data[0] += rhs; this->data[1] += rhs; this->data[2] += rhs; this->data[3] += rhs; return *this; }
    // template <> template <typename Trhs> inline vector_s<4>& vector_s<4>::operator-= (const Trhs& rhs) { this->data[0] -= rhs; this->data[1] -= rhs; this->data[2] -= rhs; this->data[3] -= rhs; return *this; }
    // template <> template <typename Trhs> inline vector_s<4>& vector_s<4>::operator*= (const Trhs& rhs) { this->data[0] *= rhs; this->data[1] *= rhs; this->data[2] *= rhs; this->data[3] *= rhs; return *this; }
    // template <> template <typename Trhs> inline vector_s<4>& vector_s<4>::operator/= (const Trhs& rhs) { this->data[0] /= rhs; this->data[1] /= rhs; this->data[2] /= rhs; this->data[3] /= rhs; return *this; }

    // template <> inline vector_s<4> vector_s<4>::Normalise(void) { return *this * Inv_sqrt(this->data[0] * this->data[0] + this->data[1] * this->data[1] + this->data[2] * this->data[2] + this->data[3] * this->data[3]); }
    

    template <size_t rows, size_t columns>
    struct matrix_s
    {
        double data[rows][columns];

        const size_t Rows = rows;
        const size_t Columns = columns;
        const size_t Size = rows * columns;

        double& operator() (const size_t& row, const size_t& column) { return data[row][column]; }
        const double& operator() (const size_t& row, const size_t& column) const { return data[row][column]; }
        double* operator[] (const size_t& row) { return data[row]; }
        const double* operator[] (const size_t& row) const { return data[row]; }

        friend vector_s<rows> operator* (const matrix_s<rows, columns>& lhs, const vector_s<columns>& rhs)
        {
            vector_s<rows> result;
            for (size_t column = 0; column < columns; column++)
            {
                double tmp = 0;
                for (size_t row = 0; row < rows; row++)
                {
                    tmp += lhs.data[row][column] * rhs[row];
                }
                result[0] = tmp;
            }
            return result;
        }

        matrix_s Transpose(void)
        {
            matrix_s result;
            
            for (size_t row = 0; row < rows; row++)
            {
                for (size_t column = 0; column < columns; column++)
                {
                    result.data[row][column] = data[column][row];
                }
            }

            return result;
        }

    };

    inline matrix_s<3,3> RotationMatrix(const vector_s<4>& quaternion)
    {
        matrix_s<3,3> matrix;
        double xx, yy, zz, xy, xz, yz, xw, yw, zw;

        xw = quaternion[0] * quaternion[0];
        xx = quaternion[0] * quaternion[1];
        xy = quaternion[0] * quaternion[2];
        xz = quaternion[0] * quaternion[3];
        
        yw = quaternion[2] * quaternion[0];
        yy = quaternion[2] * quaternion[2];
        yz = quaternion[2] * quaternion[3];
        
        zw = quaternion[3] * quaternion[0];
        zz = quaternion[3] * quaternion[3];
        
        
        matrix(0,0) = 0.5 - yy - zz;
        matrix(0,1) =       xy - zw;
        matrix(0,2) =       xz + yw;

        matrix(1,0) =       xy + zw;
        matrix(1,1) = 0.5 - xx - zz;
        matrix(1,2) =       yz - xw;

        matrix(2,0) =       xz - yw;
        matrix(2,1) =       yz + xw;
        matrix(2,2) = 0.5 - xx - yy;

        return matrix;
    }

    struct euler_t
    {
        double roll;
        double pitch;
        double yaw;

        const double& Roll (void) { return roll; }
        const double& Pitch(void) { return pitch; }
        const double& Yaw  (void) { return yaw; }

        void Roll (const double& val) { roll = val; }
        void Pitch(const double& val) { pitch = val; }
        void Yaw  (const double& val) { yaw = val; }

        void FromQuaternion(const vector_s<4>& quaternion, const matrix_s<3,3>& rotation_matrix)
        {
            roll = atan2f(rotation_matrix(2,1), rotation_matrix(2,2));
            pitch = asinf(-2.0 * rotation_matrix(2,0));
            yaw = atan2f(rotation_matrix(1,0), rotation_matrix(0,0));
        }

        void FromQuaternion(const vector_s<4>& quaternion)
        {
            const matrix_s<3,3> rotation_matrix = RotationMatrix(quaternion);

            roll = atan2f(rotation_matrix(2,1), rotation_matrix(2,2));
            pitch = asinf(-2.0 * rotation_matrix(2,0));
            yaw = atan2f(rotation_matrix(1,0), rotation_matrix(0,0));
        }
    };

    struct MadgwickFilter
    {
        vector_s<4> q;
        double beta;

        void Init(void)
        {
            q[0] = 1;
            q[1] = 0;
            q[2] = 0;
            q[3] = 0;
        }

        void Init(double Beta)
        {
            q[0] = 1;
            q[1] = 0;
            q[2] = 0;
            q[3] = 0;

            beta = Beta;
        }

        void Reset(void)
        {
            q[0] = 1;
            q[1] = 0;
            q[2] = 0;
            q[3] = 0;
        }

        void    SetBeta(const double& Beta) { beta = Beta; }
        double& GetBeta(void)               { return beta; }

        vector_s<4> Compute(vector_s<3> gyro, vector_s<3> accel, double dt)
        {
            vector_s<4> qD, gradient;

            qD = HamiltonProduct(q, gyro);

            if(!(accel[0] == 0 && accel[1] == 0 && accel[2] == 0))
            {
                accel.Normalise();

                vector_s<3> Fg;
                Fg[0] = 2 * (q[1] * q[3] - q[0] * q[2]) - accel[0];
                Fg[1] = 2 * (q[0] * q[1] + q[2] * q[3]) - accel[1];
                Fg[2] = 2 * (0.5 - q[1] * q[1] - q[2] * q[2]) - accel[2];

                matrix_s<4, 3> Jg;
                Jg[0][0] = -2 * q[2];
                Jg[0][1] =  2 * q[3];
                Jg[0][2] = -2 * q[0];
                Jg[0][3] =  2 * q[1];

                Jg[1][0] =  2 * q[1];
                Jg[1][1] =  2 * q[0];
                Jg[1][2] =  2 * q[3];
                Jg[1][3] =  2 * q[2];

                Jg[2][0] =  0;
                Jg[2][1] = -4 * q[1];
                Jg[2][2] = -4 * q[2];
                Jg[2][3] =  0;

                gradient = Jg * Fg;

                qD -= gradient.Normalise() * beta;
            }

            q += qD * dt;

            return q.Normalise();
        }

        vector_s<4> Compute2(vector_s<3> gyro, vector_s<3> accel, double dt)
        {
            vector_s<4> qD, gradient;

            qD = HamiltonProduct(q, gyro);

            if(!(accel[0] == 0 && accel[1] == 0 && accel[2] == 0))
            {
                accel.Normalise();

                gradient[0] = 4 * q[0] * q[2] * q[2] + 2 * q[2] * accel[0] + 4 * q[0] * q[0] * q[0] - 2 * q[1] * accel[1];
                gradient[1] = 4 * q[1] * q[3] * q[3] - 2 * q[3] * accel[0] + 4 * q[0] * q[0] * q[0] - 2 * q[0] * accel[1] - 4 * q[1] + 8 * q[1] * q[1] * q[1] + 8 * q[1] * q[2] * q[2] + 4 * q[1] * accel[2];
                gradient[2] = 4 * q[0] * q[0] * q[2] + 2 * q[0] * accel[0] + 4 * q[2] * q[3] * q[3] - 2 * q[3] * accel[1] - 4 * q[2] + 8 * q[2] * q[1] * q[1] + 8 * q[2] * q[2] * q[2] + 4 * q[2] * accel[2];
                gradient[3] = 4 * q[1] * q[1] * q[3] - 2 * q[1] * accel[0] + 4 * q[2] * q[2] * q[3] - 2 * q[2] * accel[1];
                
                qD -= gradient.Normalise() * beta;
            }

            q += qD * dt;

            return q.Normalise();
        }

    };
}

#endif
