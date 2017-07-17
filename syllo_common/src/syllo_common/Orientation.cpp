#include <iostream>
#include <math.h>

#include "syllo_common/Orientation.h"
#define SQ( x ) ( ( x ) * ( x ) )

using std::cout;
using std::endl;

void quaternionToEuler_q(const double &q0, const double &q1, 
                       const double &q2, const double &q3,
                       double &roll, double &pitch, double &yaw)
{
     roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2) );
     pitch = asin(2*(q0*q2-q3*q1));
     yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3) );
}

void quaternionToEuler_xyzw(const double &x, const double &y, 
                       const double &z, const double &w,
                       double &roll, double &pitch, double &yaw)
{
     quaternionToEuler_q(w, x, y, z, roll, pitch, yaw);
}

void quaternionToEuler_xyzw_deg(const double &x, const double &y, 
                       const double &z, const double &w,
                       double &roll, double &pitch, double &yaw)
{
     quaternionToEuler_q(w, x, y, z, roll, pitch, yaw);

     roll *= 180.0/syllo::PI;
     pitch *= 180.0/syllo::PI;
     yaw *= 180.0/syllo::PI;     
}


void eulerToQuaternion_q(const double &roll, const double &pitch, 
                       const double &yaw,
                       double &q0, double &q1, 
                       double &q2, double &q3)
{
     q0 = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
     q1 = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
     q2 = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
     q3 = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
}

void eulerToQuaternion_xyzw(const double &roll, const double &pitch, 
                       const double &yaw,
                       double &x, double &y, 
                       double &z, double &w)
{
     eulerToQuaternion_q(roll, pitch, yaw, w, x, y, z);
}

void eulerToQuaternion_xyzw_deg(const double &roll, const double &pitch, 
                       const double &yaw,
                       double &x, double &y, 
                       double &z, double &w)
{
     eulerToQuaternion_q(roll*syllo::PI/180, pitch*syllo::PI/180, 
                         yaw*syllo::PI/180, w, x, y, z);     
}

/**
 * rfal_quaternion_from_inertial_to_body
 *
 * Takes vectors into the body frame using a quaternion as
 * a rotation operator.
 *
 * Reference
 *  Quaternions and Rotation Sequences (p.168) -- Jack B. Kuipers
 */
void rfal_quaternion_from_inertial_to_body( const double x_inertial[3], 
					    double x_body[3],
					    const double q[4] )
{
  
  double Q[3][3] = { { 2.0 * SQ( q[0] ) - 1.0 + 2.0 * SQ( q[1] ) , 
		       2.0 * q[1] * q[2] - 2.0 * q[0] * q[3] ,
		       2.0 * q[1] * q[3] + 2.0 * q[0] * q[2] },
		     { 2.0 * q[1] * q[2] + 2.0 * q[0] * q[3] ,
		       2.0 * SQ( q[0] ) - 1.0 + 2.0 * SQ( q[2] ),
		       2.0 * q[2] * q[3] - 2.0 * q[0] * q[1] } ,
		     { 2.0 * q[1] * q[3] - 2.0 * q[0] * q[2] ,
		       2.0 * q[2] * q[3] + 2.0 * q[0] * q[1] ,
		       2.0 * SQ( q[0] ) - 1.0 + 2.0 * SQ( q[3] ) } };

  for( unsigned int i = 0 ; i < 3 ; i++ )
    for( unsigned int j = 0 ; j < 3 ; j++ )
      x_body[i] += Q[i][j] * x_inertial[j];
}

/**
 * rfal_quaternion_from_body_to_inertial
 *
 * Takes vectors into the inertial frame using a quaternion as
 * a rotation operator.
 *
 * Reference
 *  Quaternions and Rotation Sequences (p.168) -- Jack B. Kuipers
 */
void rfal_quaternion_from_body_to_inertial( double x_inertial[3], 
					    const double x_body[3],
					    const double q[4] )
{
  
  double Q[3][3] = { { 2.0 * SQ( q[0] ) - 1.0 + 2.0 * SQ( q[1] ) , 
		       2.0 * q[1] * q[2] + 2.0 * q[0] * q[3] ,
		       2.0 * q[1] * q[3] - 2.0 * q[0] * q[2] },

		     { 2.0 * q[1] * q[2] - 2.0 * q[0] * q[3] ,
		       2.0 * SQ( q[0] ) - 1.0 + 2.0 * SQ( q[2] ),
		       2.0 * q[2] * q[3] + 2.0 * q[0] * q[1] } ,

		     { 2.0 * q[1] * q[3] + 2.0 * q[0] * q[2] ,
		       2.0 * q[2] * q[3] - 2.0 * q[0] * q[1] ,
		       2.0 * SQ( q[0] ) - 1.0 + 2.0 * SQ( q[3] ) } };

  for( unsigned int i = 0 ; i < 3 ; i++ )
    for( unsigned int j = 0 ; j < 3 ; j++ )
      x_inertial[i] += Q[i][j] * x_body[j];
}
