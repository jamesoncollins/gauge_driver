#ifndef QUATERNION_HPP
#define QUATERNION_HPP

struct Quaternion {
    float w, x, y, z;
};

struct EulerAngles {
    float roll, pitch, yaw;
};

constexpr Quaternion ToQuaternion(float roll, float pitch, float yaw);
constexpr EulerAngles ToEulerAngles(Quaternion q);
float dot(float v[], float u[]);
void cross(float v_A[], float v_B[], float c_P[]);
void rotate_vector_by_quaternion(float v[], Quaternion q, float vprime[]);
constexpr void calculateRotationMatrix(float yaw, float pitch, float roll, float rotationMatrix[3][3]);
constexpr void multiplyMatrixVector(float matrix[3][3], float vector[3], float result[3], bool transpose);
constexpr void rotateVectorByPitch(float vector[3], float pitch);
void rotateVectorKnownPitch(float vector[3], float cosPitch, float sinPitch);
constexpr void rotateVectorByYawPitchRoll (float vector[3], float yaw, float pitch,
                                 float roll);

#endif /* QUATERNION_HPP */
