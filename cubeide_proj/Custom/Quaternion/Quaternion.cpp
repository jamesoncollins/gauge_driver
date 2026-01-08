#include "Quaternion.hpp"
#include <math.h>


// todo: normalize quaternion?
constexpr Quaternion ToQuaternion (float roll, float pitch, float yaw) // roll (x), pitch (y), yaw (z), angles are in radians
{
  // Abbreviations for the various angular functions

  float cr = cos (roll * 0.5);
  float sr = sin (roll * 0.5);
  float cp = cos (pitch * 0.5);
  float sp = sin (pitch * 0.5);
  float cy = cos (yaw * 0.5);
  float sy = sin (yaw * 0.5);

  return
  {
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  };
}

// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
constexpr EulerAngles ToEulerAngles (Quaternion q)
{
//  EulerAngles angles;

  // roll (x-axis rotation)
  float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);

  // pitch (y-axis rotation)
  float sinp = std::sqrt (1. + 2. * (q.w * q.y - q.x * q.z));
  float cosp = std::sqrt (1. - 2. * (q.w * q.y - q.x * q.z));

  // yaw (z-axis rotation)
  float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);

  return
  {
    std::atan2(sinr_cosp, cosr_cosp),
    2.f * std::atan2(sinp, cosp) - (float)M_PI / 2.f,
    std::atan2(siny_cosp, cosy_cosp),
  };
}

float dot(float v[], float u[])
{
    float result = 0.0;
    for (int i = 0; i < 3; i++)
        result += v[i]*u[i];
    return result;
}

void cross (float v_A[], float v_B[], float c_P[])
{
  c_P[0] = v_A[1] * v_B[2] - v_A[2] * v_B[1];
  c_P[1] = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
  c_P[2] = v_A[0] * v_B[1] - v_A[1] * v_B[0];
}

void rotate_vector_by_quaternion(float v[], Quaternion q, float vprime[])
{
    // Extract the vector part of the quaternion
    float u[3] = {q.x, q.y, q.z};

    // Extract the scalar part of the quaternion
    float s = q.w;

    float crossprod[3];
    cross(u, v, crossprod);

    float dotprod = 2.0f * dot(u, v);
    float dotprod2 = (s*s - dot(u, u)) ;

    vprime[0] = dotprod * u[0]
          + dotprod2 * v[0]
          + 2.0f * s * crossprod[0];
    vprime[1] = dotprod * u[1]
          + dotprod2 * v[1]
          + 2.0f * s * crossprod[1];
    vprime[2] = dotprod * u[2]
          + dotprod2 * v[2]
          + 2.0f * s * crossprod[2];
}


constexpr void calculateRotationMatrix (float yaw, float pitch, float roll,
                              float rotationMatrix[3][3])
{
  // Convert angles to radians
  yaw = yaw * M_PI / 180.0;
  pitch = pitch * M_PI / 180.0;
  roll = roll * M_PI / 180.0;

  // Calculate sine and cosine values
  float sy = sin (yaw);
  float cy = cos (yaw);
  float sp = sin (pitch);
  float cp = cos (pitch);
  float sr = sin (roll);
  float cr = cos (roll);

  // Calculate rotation matrix
  rotationMatrix[0][0] = cy * cp;
  rotationMatrix[0][1] = cy * sp * sr - sy * cr;
  rotationMatrix[0][2] = cy * sp * cr + sy * sr;
  rotationMatrix[1][0] = sy * cp;
  rotationMatrix[1][1] = sy * sp * sr + cy * cr;
  rotationMatrix[1][2] = sy * sp * cr - cy * sr;
  rotationMatrix[2][0] = -sp;
  rotationMatrix[2][1] = cp * sr;
  rotationMatrix[2][2] = cp * cr;
}


/*
 * specify transpose if you want to do the inverse rotation
 */
constexpr void multiplyMatrixVector (float matrix[3][3], float vector[3],
                           float result[3], bool transpose)
{
  if (!transpose)
  {
    for (int i = 0; i < 3; i++)
    {
      result[i] = 0;
      for (int j = 0; j < 3; j++)
      {
        result[i] += matrix[i][j] * vector[j];
      }
    }
  }
  else
  {
    for (int i = 0; i < 3; i++)
    {
      result[i] = 0;
      for (int j = 0; j < 3; j++)
      {
        result[i] += matrix[j][i] * vector[j];
      }
    }
  }
}

constexpr void rotateVectorByPitch(float vector[3], float pitch) {
    float x = vector[0];
    float y = vector[1];
    float z = vector[2];

    float cosPitch = cos(pitch);
    float sinPitch = sin(pitch);

    vector[0] = x * cosPitch + z * sinPitch;
    vector[1] = y;
    vector[2] = -x * sinPitch + z * cosPitch;
}

void rotateVectorKnownPitch(float vector[3], float cosPitch, float sinPitch) {
    float x = vector[0];
    float y = vector[1];
    float z = vector[2];

    vector[0] = x * cosPitch + z * sinPitch;
    vector[1] = y;
    vector[2] = -x * sinPitch + z * cosPitch;
}

/*
 * rotated a vector by a given yaw-pitch-roll in degrees
 *
 * if any of yaw-pitch or roll are 0 then caluclations for that rotation are
 * skipped
 */
constexpr void rotateVectorByYawPitchRoll (float vector[3], float yaw, float pitch,
                                 float roll)
{
  // Check if any angle is 0
  if (yaw == 0 || pitch == 0 || roll == 0)
  {
    // Skip unnecessary operations if any angle is 0
    float cosYaw = cos (yaw * M_PI / 180.0);
    float sinYaw = sin (yaw * M_PI / 180.0);
    float cosPitch = cos (pitch * M_PI / 180.0);
    float sinPitch = sin (pitch * M_PI / 180.0);
    float cosRoll = cos (roll * M_PI / 180.0);
    float sinRoll = sin (roll * M_PI / 180.0);

    float x = vector[0];
    float y = vector[1];
    float z = vector[2];

    if (yaw != 0)
    {
      float tmpX = x;
      x = tmpX * cosYaw + y * sinYaw;
      y = -tmpX * sinYaw + y * cosYaw;
    }

    if (pitch != 0)
    {
      float tmpY = y;
      y = tmpY * cosPitch - z * sinPitch;
      z = tmpY * sinPitch + z * cosPitch;
    }

    if (roll != 0)
    {
      float tmpX = x;
      x = tmpX * cosRoll - z * sinRoll;
      z = tmpX * sinRoll + z * cosRoll;
    }

    vector[0] = x;
    vector[1] = y;
    vector[2] = z;
  }
  else
  {
    // Convert angles to radians
    yaw = yaw * M_PI / 180.0;
    pitch = pitch * M_PI / 180.0;
    roll = roll * M_PI / 180.0;

    // Calculate sine and cosine values
    float sy = sin (yaw);
    float cy = cos (yaw);
    float sp = sin (pitch);
    float cp = cos (pitch);
    float sr = sin (roll);
    float cr = cos (roll);

    // Perform rotation
    float x = vector[0];
    float y = vector[1];
    float z = vector[2];

    vector[0] = x * (cy * cp) + y * (cy * sp * sr - sy * cr)
        + z * (cy * sp * cr + sy * sr);
    vector[1] = x * (sy * cp) + y * (sy * sp * sr + cy * cr)
        + z * (sy * sp * cr - cy * sr);
    vector[2] = x * (-sp) + y * (cp * sr) + z * (cp * cr);
  }
}
