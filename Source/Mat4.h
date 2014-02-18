#ifndef MAT4_H_
#define MAT4_H_

const float Deg2Rad = 0.0174532925f;
const float Rad2Deg = 57.2957795f;

#include <cstdio>
#include <cmath>
#include "Vec3.h"
#include "Quat.h"

// TODO: Add a lookat() function

struct Mat4
{
	////////////////
	///   Data   ///
	////////////////

private:

	// Matrix data members should never be accessed directly from outside
	float values[4][4];

	////////////////////////
	///   Constructors   ///
	////////////////////////

public:

	// Constructor, default sets to identity matrix
	Mat4(	const float aa = 1, const float ba = 0, const float ca = 0, const float da = 0,
			const float ab = 0, const float bb = 1, const float cb = 0, const float db = 0,
			const float ac = 0, const float bc = 0, const float cc = 1, const float dc = 0,
			const float ad = 0, const float bd = 0, const float cd = 0, const float dd = 1 )
	{
		values[0][0] = aa; values[1][0] = ba; values[2][0] = ca; values[3][0] = da;
		values[0][1] = ab; values[1][1] = bb; values[2][1] = cb; values[3][1] = db;
		values[0][2] = ac; values[1][2] = bc; values[2][2] = cc; values[3][2] = dc;
		values[0][3] = ad; values[1][3] = bd; values[2][3] = cd; values[3][3] = dd;
	}

	///////////////////
	///   Methods   ///
	///////////////////

	// Returns the inverse of this matrix
	static void inverse( const Mat4& in, Mat4* const out )
	{
		// Seriously, fuck this shit, I had to type all this by hand
		// based off http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche23.html

		///////////////////
		///  COLUMN 1   ///
		///////////////////

		/// ROW 1
		out->set( 0, 0,
			in[1][1]*in[2][2]*in[3][3] +
			in[2][1]*in[3][2]*in[1][3] +
			in[3][1]*in[1][2]*in[2][3] -
			in[1][1]*in[3][2]*in[2][3] -
			in[2][1]*in[1][2]*in[3][3] -
			in[3][1]*in[2][2]*in[1][3] );

		/// ROW 2
		out->set( 0, 1,
			in[0][1]*in[3][2]*in[2][3] +
			in[2][1]*in[0][2]*in[3][3] +
			in[3][1]*in[2][2]*in[0][3] -
			in[0][1]*in[2][2]*in[3][3] -
			in[2][1]*in[3][2]*in[0][3] -
			in[3][1]*in[0][2]*in[2][3] );

		/// ROW 3
		out->set( 0, 2,
			in[0][1]*in[1][2]*in[3][3] +
			in[1][1]*in[3][2]*in[0][3] +
			in[3][1]*in[0][2]*in[1][3] -
			in[0][1]*in[3][2]*in[1][3] -
			in[1][1]*in[0][2]*in[3][3] -
			in[3][1]*in[1][2]*in[0][3] );

		/// ROW 4 (fixed)
		out->set( 0, 3,
			in[0][1]*in[2][2]*in[1][3] +
			in[1][1]*in[0][2]*in[2][3] +
			in[2][1]*in[1][2]*in[0][3] -
			in[0][1]*in[1][2]*in[2][3] -
			in[1][1]*in[2][2]*in[0][3] -
			in[2][1]*in[0][2]*in[1][3] );

		////////////////////
		///   COLUMN 2   ///
		////////////////////

		/// ROW 1
		out->set( 1, 0,
			in[1][0]*in[3][2]*in[2][3] +
			in[2][0]*in[1][2]*in[3][3] +
			in[3][0]*in[2][2]*in[1][3] -
			in[1][0]*in[2][2]*in[3][3] -
			in[2][0]*in[3][2]*in[1][3] -
			in[3][0]*in[1][2]*in[2][3] );

		/// ROW 2
		out->set( 1, 1,
			in[0][0]*in[2][2]*in[3][3] +
			in[2][0]*in[3][2]*in[0][3] +
			in[3][0]*in[0][2]*in[2][3] -
			in[0][0]*in[3][2]*in[2][3] -
			in[2][0]*in[0][2]*in[3][3] -
			in[3][0]*in[2][2]*in[0][3] );

		/// ROW 3
		out->set( 1, 2,
			in[0][0]*in[3][2]*in[1][3] +
			in[1][0]*in[0][2]*in[3][3] +
			in[3][0]*in[1][2]*in[0][3] -
			in[0][0]*in[1][2]*in[3][3] -
			in[1][0]*in[3][2]*in[0][3] -
			in[3][0]*in[0][2]*in[1][3] );

		/// ROW 4
		out->set( 1, 3,
			in[0][0]*in[1][2]*in[2][3] +
			in[1][0]*in[2][2]*in[0][3] +
			in[2][0]*in[0][2]*in[1][3] -
			in[0][0]*in[2][2]*in[1][3] -
			in[1][0]*in[0][2]*in[2][3] -
			in[2][0]*in[1][2]*in[0][3] );

		////////////////////
		///   COLUMN 3   ///
		////////////////////

		/// ROW 1
		out->set( 2, 0,
			in[1][0]*in[2][1]*in[3][3] +
			in[2][0]*in[3][1]*in[1][3] +
			in[3][0]*in[1][1]*in[2][3] -
			in[1][0]*in[3][1]*in[2][3] -
			in[2][0]*in[1][1]*in[3][3] -
			in[3][0]*in[2][1]*in[1][3] );

		/// ROW 2
		out->set( 2, 1,
			in[0][0]*in[3][1]*in[2][3] +
			in[2][0]*in[0][1]*in[3][3] +
			in[3][0]*in[2][1]*in[0][3] -
			in[0][0]*in[2][1]*in[3][3] -
			in[2][0]*in[3][1]*in[0][3] -
			in[3][0]*in[0][1]*in[2][3] );

		/// ROW 3
		out->set( 2, 2,
			in[0][0]*in[1][1]*in[3][3] +
			in[1][0]*in[3][1]*in[0][3] +
			in[3][0]*in[0][1]*in[1][3] -
			in[0][0]*in[3][1]*in[1][3] -
			in[1][0]*in[0][1]*in[3][3] -
			in[3][0]*in[1][1]*in[0][3] );

		/// ROW 4
		out->set( 2, 3,
			in[0][0]*in[2][1]*in[1][3] +
			in[1][0]*in[0][1]*in[2][3] +
			in[2][0]*in[1][1]*in[0][3] -
			in[0][0]*in[1][1]*in[2][3] -
			in[1][0]*in[2][1]*in[0][3] -
			in[2][0]*in[0][1]*in[1][3] );

		////////////////////
		///   COLUMN 4   ///
		////////////////////

		/// ROW 1
		out->set( 3, 0,
			in[1][0]*in[3][1]*in[2][2] +
			in[2][0]*in[1][1]*in[3][2] +
			in[3][0]*in[2][1]*in[1][2] -
			in[1][0]*in[2][1]*in[3][2] -
			in[2][0]*in[3][1]*in[1][2] -
			in[3][0]*in[1][1]*in[2][2] );

		/// ROW 2
		out->set( 3, 1,
			in[0][0]*in[2][1]*in[3][2] +
			in[2][0]*in[3][1]*in[0][2] +
			in[3][0]*in[0][1]*in[2][2] -
			in[0][0]*in[3][1]*in[2][2] -
			in[2][0]*in[0][1]*in[3][2] -
			in[3][0]*in[2][1]*in[0][2] );

		/// ROW 3
		out->set( 3, 2,
			in[0][0]*in[3][1]*in[1][2] +
			in[1][0]*in[0][1]*in[3][2] +
			in[3][0]*in[1][1]*in[0][2] -
			in[0][0]*in[1][1]*in[3][2] -
			in[1][0]*in[3][1]*in[0][2] -
			in[3][0]*in[0][1]*in[1][2] );

		/// ROW 4
		out->set( 3, 3,
			in[0][0]*in[1][1]*in[2][2] +
			in[1][0]*in[2][1]*in[0][2] +
			in[2][0]*in[0][1]*in[1][2] -
			in[0][0]*in[2][1]*in[1][2] -
			in[1][0]*in[0][1]*in[2][2] -
			in[2][0]*in[1][1]*in[0][2] );
	}

//bool gluInvertMatrix( const double m[16], double invOut[16] ) {
//    double inv[16], det;
//    int i;
//
//    inv[0] = m[5]  * m[10] * m[15] - 
//             m[5]  * m[11] * m[14] - 
//             m[9]  * m[6]  * m[15] + 
//             m[9]  * m[7]  * m[14] +
//             m[13] * m[6]  * m[11] - 
//             m[13] * m[7]  * m[10];
//
//    inv[4] = -m[4]  * m[10] * m[15] + 
//              m[4]  * m[11] * m[14] + 
//              m[8]  * m[6]  * m[15] - 
//              m[8]  * m[7]  * m[14] - 
//              m[12] * m[6]  * m[11] + 
//              m[12] * m[7]  * m[10];
//
//    inv[8] = m[4]  * m[9] * m[15] - 
//             m[4]  * m[11] * m[13] - 
//             m[8]  * m[5] * m[15] + 
//             m[8]  * m[7] * m[13] + 
//             m[12] * m[5] * m[11] - 
//             m[12] * m[7] * m[9];
//
//    inv[12] = -m[4]  * m[9] * m[14] + 
//               m[4]  * m[10] * m[13] +
//               m[8]  * m[5] * m[14] - 
//               m[8]  * m[6] * m[13] - 
//               m[12] * m[5] * m[10] + 
//               m[12] * m[6] * m[9];
//
//    inv[1] = -m[1]  * m[10] * m[15] + 
//              m[1]  * m[11] * m[14] + 
//              m[9]  * m[2] * m[15] - 
//              m[9]  * m[3] * m[14] - 
//              m[13] * m[2] * m[11] + 
//              m[13] * m[3] * m[10];
//
//    inv[5] = m[0]  * m[10] * m[15] - 
//             m[0]  * m[11] * m[14] - 
//             m[8]  * m[2] * m[15] + 
//             m[8]  * m[3] * m[14] + 
//             m[12] * m[2] * m[11] - 
//             m[12] * m[3] * m[10];
//
//    inv[9] = -m[0]  * m[9] * m[15] + 
//              m[0]  * m[11] * m[13] + 
//              m[8]  * m[1] * m[15] - 
//              m[8]  * m[3] * m[13] - 
//              m[12] * m[1] * m[11] + 
//              m[12] * m[3] * m[9];
//
//    inv[13] = m[0]  * m[9] * m[14] - 
//              m[0]  * m[10] * m[13] - 
//              m[8]  * m[1] * m[14] + 
//              m[8]  * m[2] * m[13] + 
//              m[12] * m[1] * m[10] - 
//              m[12] * m[2] * m[9];
//
//    inv[2] = m[1]  * m[6] * m[15] - 
//             m[1]  * m[7] * m[14] - 
//             m[5]  * m[2] * m[15] + 
//             m[5]  * m[3] * m[14] + 
//             m[13] * m[2] * m[7] - 
//             m[13] * m[3] * m[6];
//
//    inv[6] = -m[0]  * m[6] * m[15] + 
//              m[0]  * m[7] * m[14] + 
//              m[4]  * m[2] * m[15] - 
//              m[4]  * m[3] * m[14] - 
//              m[12] * m[2] * m[7] + 
//              m[12] * m[3] * m[6];
//
//    inv[10] = m[0]  * m[5] * m[15] - 
//              m[0]  * m[7] * m[13] - 
//              m[4]  * m[1] * m[15] + 
//              m[4]  * m[3] * m[13] + 
//              m[12] * m[1] * m[7] - 
//              m[12] * m[3] * m[5];
//
//    inv[14] = -m[0]  * m[5] * m[14] + 
//               m[0]  * m[6] * m[13] + 
//               m[4]  * m[1] * m[14] - 
//               m[4]  * m[2] * m[13] - 
//               m[12] * m[1] * m[6] + 
//               m[12] * m[2] * m[5];
//
//    inv[3] = -m[1] * m[6] * m[11] + 
//              m[1] * m[7] * m[10] + 
//              m[5] * m[2] * m[11] - 
//              m[5] * m[3] * m[10] - 
//              m[9] * m[2] * m[7] + 
//              m[9] * m[3] * m[6];
//
//    inv[7] = m[0] * m[6] * m[11] - 
//             m[0] * m[7] * m[10] - 
//             m[4] * m[2] * m[11] + 
//             m[4] * m[3] * m[10] + 
//             m[8] * m[2] * m[7] - 
//             m[8] * m[3] * m[6];
//
//    inv[11] = -m[0] * m[5] * m[11] + 
//               m[0] * m[7] * m[9] + 
//               m[4] * m[1] * m[11] - 
//               m[4] * m[3] * m[9] - 
//               m[8] * m[1] * m[7] + 
//               m[8] * m[3] * m[5];
//
//    inv[15] = m[0] * m[5] * m[10] - 
//              m[0] * m[6] * m[9] - 
//              m[4] * m[1] * m[10] + 
//              m[4] * m[2] * m[9] + 
//              m[8] * m[1] * m[6] - 
//              m[8] * m[2] * m[5];
//
//    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
//
//    if (det == 0)
//        return false;
//
//    det = 1.0 / det;
//
//    for (i = 0; i < 16; i++)
//        invOut[i] = inv[i] * det;
//
//    return true;
//}

	// Print the matrix to the console
	void display() const
	{
		// For each row
		for( int row = 0; row < 4; row++ ) {
			printf("| ");
			
			// For each column
			for( int col = 0; col < 4; col++ ) {
				// Print the value in that place
				printf( "%f  ", values[col][row] );
			}
			// Start a new line
			printf( " |\n" );
		}
		printf("\n\n\n");
	}

	/////////////////////
	///   Rendering   ///
	/////////////////////
	
	// Generates a perspective projection matrix
	static void perspective( const float hFOV, const float vFOV, const float zMin, const float zMax, Mat4* const out )
	{
		// Convert vertical and horizontal FOV to radians
		const float RadHFOV = hFOV * Deg2Rad;
		const float RadVFOV = vFOV * Deg2Rad;

		const float xMax = std::tanf( RadHFOV * 0.5f ) * zMin;
		const float xMin = -xMax;

		const float yMax = std::tanf( RadVFOV * 0.5f ) * zMin;
		const float yMin = -yMax;

		const float width = xMax - xMin;
		const float height = yMax - yMin;
		const float depth = zMax - zMin;
		
		*out = Mat4(
			2*zMin/width,	0,					(xMax+xMin)/width,		0,
			0,				2*zMin/height,		(yMax+yMin)/height,		0,
			0,				0,					-(zMax+zMin)/depth,		-2*zMax*zMin/depth,
			0,				0,					-1,						0 );
	}

	// Generates a persepctive projection matrix based off desired horizontal FOV and screen ratio
	static void perspectiveHFOV( const float hFOV, const float ratio, const float zMin, const float zMax, Mat4* const out )
	{
		// Convert hFOV to radians
		const float RadHFOV = hFOV * Deg2Rad;

		const float vFOV = Rad2Deg*2*atan( tan( hFOV * 0.5f ) * 1/ratio );

		perspective( hFOV, vFOV, zMin, zMax, out );
	}

	// Generates a perspective projection matrix based off the desired vertical FOV and screen ratio
	static void perspectiveVFOV( const float vFOV, const float ratio, const float zMin, const float zMax, Mat4* const out )
	{
		// Convert the vFOV to radians
		const float RadVFOV = vFOV * Deg2Rad;

		float hFOV = Rad2Deg*2*atan( tan( RadVFOV * 0.5f ) * ratio );

		perspective( hFOV, vFOV, zMin, zMax, out );
	}

	// Generates an orthographic projection matrix
	static void orthographic( const float xMin, const float xMax, const float yMin, const float yMax, const float zMin, const float zMax, Mat4* const out )
	{
		const float width = xMax - xMin;
		const float height = yMax - yMin;
		const float depth = zMax - zMin;

		*out = Mat4( 
			2/width,	0,			0,			-(xMax+xMin)/width,
			0,			2/height,	0,			-(yMax+yMin)/height,
			0,			0,			-2/depth,	-(zMax+zMin)/depth,
			0,			0,			0,			1 );
	}

	// Returns a translation matrix from a 3-length vector
	static void translate( const Vec3& vec, Mat4* const out )
	{
		*out = Mat4(
			1,	0,	0,	vec.x,
			0,	1,	0,	vec.y,
			0,	0,	1,	vec.z,
			0,	0,	0,	1 );
	}

	// Returns a scale matrix from a 3-length vector
	static void scale( const Vec3& vec, Mat4* const out )
	{
		*out = Mat4(
			vec.x,	0,	0,	0,
			0,	vec.y,	0,	0,
			0,	0,	vec.z,	0,
			0,	0,	0,	1	);
	}

	// Returns a rotation matrix from a quaternion
	static void rotate( const Quat& rot, Mat4* const out )
	{
		// Retreive the data members of rot
		float x, y, z, w;
		rot.retrieve( &x, &y, &z, &w );
		
		*out = Mat4(
			1 - 2*y*y - 2*z*z,	2*x*y + 2*z*w,		2*x*z - 2*y*w,		0,
			2*x*y - 2*z*w,		1 - 2*x*x - 2*z*z,	2*y*z + 2*x*w,		0,
			2*x*z + 2*y*w,		2*y*z - 2*x*w,		1 - 2*x*x - 2*y*y,	0,
			0,					0,					0,					1 );
	}

	///////////////////////////////
	///   Getters and Setters   ///
	///////////////////////////////

	// Get contents by row and column
	inline float get( const std::size_t column, const std::size_t row ) const
	{
		return values[column][row];
	}

	// Set contents by row and column
	inline void set( const std::size_t column, const std::size_t row, const float value )
	{
		values[column][row] = value;
	}

	/////////////////////
	///   Overloads   ///
	/////////////////////

	// Multiply by another matrix
	Mat4 operator*( const Mat4& rhs )
	{
		Mat4 total;

		// Cool algorithm I came up with for automatically doing this shit
		// Based off http://www.open.gl/transformations
		
		// For each row
		for( std::size_t row = 0; row < 4; row++ ) {
			// For each column
			for( std::size_t col = 0; col < 4; col++ ) {
				float value = 0;
				
				// For each addition
				for( std::size_t i = 0; i < 4; i++ ) {
					// add them up
					value += values[i][row] * rhs.get( col, i );
				}

				// Assign it to the new matrix
				total.set( col, row, value );
			}
		}

		// Return the product of the two matrices
		return total;
	}

	// Multiply by a 3-length vector
	Vec3 operator* ( const Vec3& rhs )
	{
		Vec3 result;
		result.x = values[0][0]*rhs.x + values[1][0]*rhs.y + values[2][0]*rhs.z + values[3][0];
		result.y = values[0][1]*rhs.x + values[1][1]*rhs.y + values[2][1]*rhs.z + values[3][1];
		result.z = values[0][2]*rhs.x + values[1][2]*rhs.y + values[2][2]*rhs.z + values[3][2];

		return result;
	}

	// Get contents as an array
	float* operator[]( const std::size_t index )
	{
		return values[index];
	}

	// Get contents as a const array
	const float* const operator[]( std::size_t index ) const
	{
		return values[index];
	}
};

#endif