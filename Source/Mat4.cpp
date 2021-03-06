// Mat4.cpp

#include "Mat4.h"

const float Deg2Rad = 0.0174532925f;
const float Rad2Deg = 57.2957795f;

////////////////////////
///   Constructors   ///
////////////////////////

// Constructor, default sets to identity matrix
Mat4::Mat4(	const float aa, const float ba, const float ca, const float da,
			const float ab, const float bb, const float cb, const float db,
			const float ac, const float bc, const float cc, const float dc,
			const float ad, const float bd, const float cd, const float dd )
{
	this->values[0][0] = aa; this->values[1][0] = ba; this->values[2][0] = ca; this->values[3][0] = da;
	this->values[0][1] = ab; this->values[1][1] = bb; this->values[2][1] = cb; this->values[3][1] = db;
	this->values[0][2] = ac; this->values[1][2] = bc; this->values[2][2] = cc; this->values[3][2] = dc;
	this->values[0][3] = ad; this->values[1][3] = bd; this->values[2][3] = cd; this->values[3][3] = dd;
}

///////////////////
///   Methods   ///
///////////////////

// Returns the inverse of this matrix
Mat4 Mat4::inverse()
{
	// Based off http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche23.html

	Mat4 result;

	///////////////////
	///  COLUMN 1   ///
	///////////////////

	/// ROW 1
	result.set( 0, 0,
		values[1][1]*values[2][2]*values[3][3] +
		values[2][1]*values[3][2]*values[1][3] +
		values[3][1]*values[1][2]*values[2][3] -
		values[1][1]*values[3][2]*values[2][3] -
		values[2][1]*values[1][2]*values[3][3] -
		values[3][1]*values[2][2]*values[1][3] );

	/// ROW 2
	result.set( 0, 1,
		values[0][1]*values[3][2]*values[2][3] +
		values[2][1]*values[0][2]*values[3][3] +
		values[3][1]*values[2][2]*values[0][3] -
		values[0][1]*values[2][2]*values[3][3] -
		values[2][1]*values[3][2]*values[0][3] -
		values[3][1]*values[0][2]*values[2][3] );

	/// ROW 3
	result.set( 0, 2,
		values[0][1]*values[1][2]*values[3][3] +
		values[1][1]*values[3][2]*values[0][3] +
		values[3][1]*values[0][2]*values[1][3] -
		values[0][1]*values[3][2]*values[1][3] -
		values[1][1]*values[0][2]*values[3][3] -
		values[3][1]*values[1][2]*values[0][3] );

	/// ROW 4 (fixed)
	result.set( 0, 3,
		values[0][1]*values[2][2]*values[1][3] +
		values[1][1]*values[0][2]*values[2][3] +
		values[2][1]*values[1][2]*values[0][3] -
		values[0][1]*values[1][2]*values[2][3] -
		values[1][1]*values[2][2]*values[0][3] -
		values[2][1]*values[0][2]*values[1][3] );

	////////////////////
	///   COLUMN 2   ///
	////////////////////

	/// ROW 1
	result.set( 1, 0,
		values[1][0]*values[3][2]*values[2][3] +
		values[2][0]*values[1][2]*values[3][3] +
		values[3][0]*values[2][2]*values[1][3] -
		values[1][0]*values[2][2]*values[3][3] -
		values[2][0]*values[3][2]*values[1][3] -
		values[3][0]*values[1][2]*values[2][3] );

	/// ROW 2
	result.set( 1, 1,
		values[0][0]*values[2][2]*values[3][3] +
		values[2][0]*values[3][2]*values[0][3] +
		values[3][0]*values[0][2]*values[2][3] -
		values[0][0]*values[3][2]*values[2][3] -
		values[2][0]*values[0][2]*values[3][3] -
		values[3][0]*values[2][2]*values[0][3] );

	/// ROW 3
	result.set( 1, 2,
		values[0][0]*values[3][2]*values[1][3] +
		values[1][0]*values[0][2]*values[3][3] +
		values[3][0]*values[1][2]*values[0][3] -
		values[0][0]*values[1][2]*values[3][3] -
		values[1][0]*values[3][2]*values[0][3] -
		values[3][0]*values[0][2]*values[1][3] );

	/// ROW 4
	result.set( 1, 3,
		values[0][0]*values[1][2]*values[2][3] +
		values[1][0]*values[2][2]*values[0][3] +
		values[2][0]*values[0][2]*values[1][3] -
		values[0][0]*values[2][2]*values[1][3] -
		values[1][0]*values[0][2]*values[2][3] -
		values[2][0]*values[1][2]*values[0][3] );

	////////////////////
	///   COLUMN 3   ///
	////////////////////

	/// ROW 1
	result.set( 2, 0,
		values[1][0]*values[2][1]*values[3][3] +
		values[2][0]*values[3][1]*values[1][3] +
		values[3][0]*values[1][1]*values[2][3] -
		values[1][0]*values[3][1]*values[2][3] -
		values[2][0]*values[1][1]*values[3][3] -
		values[3][0]*values[2][1]*values[1][3] );

	/// ROW 2
	result.set( 2, 1,
		values[0][0]*values[3][1]*values[2][3] +
		values[2][0]*values[0][1]*values[3][3] +
		values[3][0]*values[2][1]*values[0][3] -
		values[0][0]*values[2][1]*values[3][3] -
		values[2][0]*values[3][1]*values[0][3] -
		values[3][0]*values[0][1]*values[2][3] );

	/// ROW 3
	result.set( 2, 2,
		values[0][0]*values[1][1]*values[3][3] +
		values[1][0]*values[3][1]*values[0][3] +
		values[3][0]*values[0][1]*values[1][3] -
		values[0][0]*values[3][1]*values[1][3] -
		values[1][0]*values[0][1]*values[3][3] -
		values[3][0]*values[1][1]*values[0][3] );

	/// ROW 4
	result.set( 2, 3,
		values[0][0]*values[2][1]*values[1][3] +
		values[1][0]*values[0][1]*values[2][3] +
		values[2][0]*values[1][1]*values[0][3] -
		values[0][0]*values[1][1]*values[2][3] -
		values[1][0]*values[2][1]*values[0][3] -
		values[2][0]*values[0][1]*values[1][3] );

	////////////////////
	///   COLUMN 4   ///
	////////////////////

	/// ROW 1
	result.set( 3, 0,
		values[1][0]*values[3][1]*values[2][2] +
		values[2][0]*values[1][1]*values[3][2] +
		values[3][0]*values[2][1]*values[1][2] -
		values[1][0]*values[2][1]*values[3][2] -
		values[2][0]*values[3][1]*values[1][2] -
		values[3][0]*values[1][1]*values[2][2] );

	/// ROW 2
	result.set( 3, 1,
		values[0][0]*values[2][1]*values[3][2] +
		values[2][0]*values[3][1]*values[0][2] +
		values[3][0]*values[0][1]*values[2][2] -
		values[0][0]*values[3][1]*values[2][2] -
		values[2][0]*values[0][1]*values[3][2] -
		values[3][0]*values[2][1]*values[0][2] );

	/// ROW 3
	result.set( 3, 2,
		values[0][0]*values[3][1]*values[1][2] +
		values[1][0]*values[0][1]*values[3][2] +
		values[3][0]*values[1][1]*values[0][2] -
		values[0][0]*values[1][1]*values[3][2] -
		values[1][0]*values[3][1]*values[0][2] -
		values[3][0]*values[0][1]*values[1][2] );

	/// ROW 4
	result.set( 3, 3,
		values[0][0]*values[1][1]*values[2][2] +
		values[1][0]*values[2][1]*values[0][2] +
		values[2][0]*values[0][1]*values[1][2] -
		values[0][0]*values[2][1]*values[1][2] -
		values[1][0]*values[0][1]*values[2][2] -
		values[2][0]*values[1][1]*values[0][2] );

	return result;
}

// Print the matrix to the console
void Mat4::display() const
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
Mat4 Mat4::perspective( const float hFOV, const float vFOV, const float zMin, const float zMax )
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

	return Mat4(
		2*zMin/width,	0,					(xMax+xMin)/width,		0,
		0,				2*zMin/height,		(yMax+yMin)/height,		0,
		0,				0,					-(zMax+zMin)/depth,		-2*zMax*zMin/depth,
		0,				0,					-1,						0 );
}

// Generates a persepctive projection matrix based off desired horizontal FOV and screen ratio
Mat4 Mat4::perspectiveHFOV( const float hFOV, const float ratio, const float zMin, const float zMax )
{
	// Convert hFOV to radians
	const float RadHFOV = hFOV * Deg2Rad;

	const float vFOV = Rad2Deg*2*atan( tan( hFOV * 0.5f ) * 1/ratio );

	return perspective( hFOV, vFOV, zMin, zMax );
}

// Generates a perspective projection matrix based off the desired vertical FOV and screen ratio
Mat4 Mat4::perspectiveVFOV( const float vFOV, const float ratio, const float zMin, const float zMax )
{
	// Convert the vFOV to radians
	const float RadVFOV = vFOV * Deg2Rad;

	float hFOV = Rad2Deg*2*atan( tan( RadVFOV * 0.5f ) * ratio );

	return perspective( hFOV, vFOV, zMin, zMax );
}

// Generates an orthographic projection matrix
Mat4 Mat4::orthographic( const float xMin, const float xMax, const float yMin, const float yMax, const float zMin, const float zMax )
{
	const float width = xMax - xMin;
	const float height = yMax - yMin;
	const float depth = zMax - zMin;

	return Mat4( 
		2/width,	0,			0,			-(xMax+xMin)/width,
		0,			2/height,	0,			-(yMax+yMin)/height,
		0,			0,			-2/depth,	-(zMax+zMin)/depth,
		0,			0,			0,			1 );
}

// Returns a translation matrix from a 3-length vector
Mat4 Mat4::translate( const Vec3& vec )
{
	return Mat4(
		1,	0,	0,	vec.x,
		0,	1,	0,	vec.y,
		0,	0,	1,	vec.z,
		0,	0,	0,	1 );
}

// Returns a scale matrix from a 3-length vector
Mat4 Mat4::scale( const Vec3& vec )
{
	return Mat4(
		vec.x,	0,	0,	0,
		0,	vec.y,	0,	0,
		0,	0,	vec.z,	0,
		0,	0,	0,	1	);
}

// Returns a rotation matrix from a quaternion
Mat4 Mat4::rotate( const Quat& rot )
{
	// Retreive the data members of rot
	float x, y, z, w;
	rot.retrieve( &x, &y, &z, &w );

	return Mat4(
		1 - 2*y*y - 2*z*z,	2*x*y + 2*z*w,		2*x*z - 2*y*w,		0,
		2*x*y - 2*z*w,		1 - 2*x*x - 2*z*z,	2*y*z + 2*x*w,		0,
		2*x*z + 2*y*w,		2*y*z - 2*x*w,		1 - 2*x*x - 2*y*y,	0,
		0,					0,					0,					1 );
}

/////////////////////
///   Overloads   ///
/////////////////////

// Multiply by another matrix
Mat4 Mat4::operator*( const Mat4& rhs )
{
	Mat4 total;

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
Vec3 Mat4::operator* ( const Vec3& rhs )
{
	Vec3 result;
	result.x = values[0][0]*rhs.x + values[1][0]*rhs.y + values[2][0]*rhs.z + values[3][0];
	result.y = values[0][1]*rhs.x + values[1][1]*rhs.y + values[2][1]*rhs.z + values[3][1];
	result.z = values[0][2]*rhs.x + values[1][2]*rhs.y + values[2][2]*rhs.z + values[3][2];

	return result;
}
