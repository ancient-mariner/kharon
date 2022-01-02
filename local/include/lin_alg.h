#if !defined(LIN_ALG_H)
#define LIN_ALG_H
#include <stdint.h>
#include <math.h>
#include "pin_types.h"

// linear algebra routines as well as some general math stuff
// TODO rename this file to something more meaningful, or split content
//    into multiple files

#define R2D (180.0 / M_PI)
#define D2R (M_PI / 180.0)

#define R2D_FLT ((float) R2D)
#define D2R_FLT ((float) D2R)


////////////////////////////////////////////////////////////////////////
// tangent approximations
// basically, tan(theta) ~= theta for small theta
// these are only to be used for small theta. when theta is 25 degrees,
//    error is just under 7%, which is OK for most uses. error is 
//    smaller for <15 degrees, and much smaller for <10

// takes int32_t (bam32.sangle32)
#define TAN_APPROX32(x)      ((double) (x) * BAM32_TO_DEG * D2R)

// takes double value, in degrees
#define TAN_APPROX_DEG(x)      ((x) * D2R)
// returns approx atan in degrees
#define ATAN_APPROX_DEG(x)     ((x) * R2D)

// takes double value, in degrees
#define SIN_APPROX_DEG(x)      ((x) * D2R)


// TODO convert many of these to inline. see:
// https://stackoverflow.com/questions/10291581/how-to-properly-inline-for-static-libraries
// alternatively, use link-time optimization

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// functions with explicit unit tests

////////////////////////////////////
// initialization and set operations

// set vector to [a, b, c]
void vector_init(
      /*    out */       vector_type *vec,
      /* in     */ const double a,
      /* in     */ const double b,
      /* in     */ const double c);

// set matrix to [[row_1],[row_2],[row_3]]
void matrix_init(
      /*    out */       matrix_type *m,
      /* in     */ const vector_type *row_1,
      /* in     */ const vector_type *row_2,
      /* in     */ const vector_type *row_3);

// set matrix to [[row_1],[row_2],[row_3]] then transpose it
void matrix_init_transpose(
      /*    out */       matrix_type *m,
      /* in     */ const vector_type *row_1,
      /* in     */ const vector_type *row_2,
      /* in     */ const vector_type *row_3);

// set matrix to [[row_1],[row_2],[row_3]]
void matrix_init_raw(
      /*    out */       matrix_type *m,
      /* in     */ const double p00,
      /* in     */ const double p01,
      /* in     */ const double p02,
      /* in     */ const double p10,
      /* in     */ const double p11,
      /* in     */ const double p12,
      /* in     */ const double p20,
      /* in     */ const double p21,
      /* in     */ const double p22);

// sets matrix_type to zero
void zero_matrix(
      /* in out */       matrix_type *mat);

// stores 3x3 identity matrix_type in m
void identity_matrix(
      /*    out */       matrix_type *mat);

void copy_matrix(
      /* in     */ const matrix_type *mat,
      /*    out */       matrix_type *dup
      );

///////////////////////
// comparison functions

// returns 0 if values are too far apart and nonzero (1) if they're close
//    (close is within ~0.1%)
int close_enough(double x, double y);
int close_enough_absolute(double x, double y);
int close_enough_debug(double x, double y);


/////////////////////////
// mathematical functions

// v * m
// vec and result must be distinct
void mult_vector_matrix(
      /* in     */ const vector_type *vec,
      /* in     */ const matrix_type *m,
      /*    out */       vector_type *result);

// m * v
// vec and result must be distinct
void mult_matrix_vector(
      /* in     */ const matrix_type *m,
      /* in     */ const vector_type *vec,
      /*    out */       vector_type *result);

// squares m and stores result in m2
// m must be distinct from m2
void square_matrix(
      /* in     */ const matrix_type *m, 
      /*    out */ matrix_type *m2);

// m1 * m2 => res
// res must be distinct from m1,m2
void mult_matrix(
      /* in     */ const matrix_type *m1, 
      /* in     */ const matrix_type *m2, 
      /*    out */ matrix_type *res);

// performs a = a + s*b
void add_weighted_vector(
      /* in out */       vector_type *a, 
      /* in     */ const vector_type *b, 
      /* in     */ const double s
      );

////////////////////
// building matrices

// build_orthogonal_matrix_ab(), where a is major axis and b is minor
// creates an orthogonal matrix where the major row is the primary
//    vector, the minor row is the image of the secondary vector on
//    the plane normal to the primary, and the remaining axis orthogonal
//    to the other two
// NOTE: behavior indeterminate if the dot product of the primary 
//    and secondary vectors is zero
void build_orthogonal_matrix_xy(
      /* in      */ const vector_type *primary, 
      /* in      */ const vector_type *secondary,
      /*     out */       matrix_type *mat);

void build_orthogonal_matrix_yx(
      /* in      */ const vector_type *primary, 
      /* in      */ const vector_type *secondary,
      /*     out */       matrix_type *mat);

// Y is typically up and Z forward (north), so for many operations
//    this will be the function to use
void build_orthogonal_matrix_yz(
      /* in      */ const vector_type *primary, 
      /* in      */ const vector_type *secondary,
      /*     out */       matrix_type *mat);


// constructs matrix to remove tilt and roll 
// matrix describes rotation from positive Y axis to 'up' vector
void build_upright_matrix(
      /* in      */ const vector_type *up, 
      /*     out */       matrix_type *mat);


/////////////////////////
// unit length operations

// makes supplied vector_type unit-length
// returns previous vector_type length
double unitify(
      /* in out */ vector_type *v);

// stores unit-length version of vec in unit
void unit_vector(
      /* in     */ const vector_type *vec, 
      /*    out */ vector_type *unit);


//////////////////////
// rotation operations

// calculates rotation from a to b
void measure_rotation(
      /* in     */ const vector_type *prev,
      /* in     */ const vector_type *curr, 
      /*    out */       vector_type *axis, 
      /*    out */       degree_type *theta
      );

// rotate vector_type vec about axis, by theta degrees, and store result in res
void rotate_vector_about_axis(
      /* in     */ const vector_type *axis, 
      /* in     */ const vector_type *vec, 
      /* in     */ const degree_type theta, 
      /*    out */ vector_type *res);

/////////////////////
// heading/pitch/roll

degree_type get_pitch(
      /* in     */ const vector_type *acc
      );

// positive roll will lift leff (port) side and lower right
// negative roll lifts right side
// roll values valid on [-90,90] -- roll beyond that is invalid, and besides
//    that there will be bigger problems to worry about if that happens 
//    than accurate tracking
degree_type get_roll(
      /* in     */ const vector_type *acc
      );

// returns heading of ship in degrees
// corrects for attitude variations that, esp. near pole, where moderate
//    pitch/roll angles can induce errors
// return value on [0,360)
degree_type get_heading(
      /* in     */ const vector_type *acc,
      /* in     */ const vector_type *mag
      );

// returns direction of north, corrected for attitude variations
// return value on [0,360)
degree_type get_north(
      /* in     */ const vector_type *acc,
      /* in     */ const vector_type *mag
      );


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// functions WITHOUT dedicated unit tests

// returns unordered, smallest angle between headings a and b
// result on [0,180]
degree_type angle_between(
      /* in     */ const degree_type a,
      /* in     */ const degree_type b
      );


// transpose matrix (in place)
void matrix_transpose(
      /* in out */ matrix_type *m);

//// extract roll, pitch and yaw from a rotation matrix
//// NOTE: algorithm senstive to gimbal lock at ... TODO finish sentence
//void get_roll_pitch_yaw(
//      /* in     */ matrix_type *m,
//      /*    out */ double *roll,
//      /*    out */ double *pitch,
//      /*    out */ double *yaw);


// sets vector_type to zero
void zero_vector(
      /* in out */ vector_type *vec);


// DEPRECATED -- use zero_vector instead
void reset_vector(
      /* in out */ vector_type *vec);

// copy vector_type v to dup
void copy_vector(
      /* in     */ const vector_type *v, 
      /*    out */ vector_type *dup);

// copy vector_type v to dup and make dup unit length
void copy_vector_unitify(
      /* in     */ const vector_type *v, 
      /*    out */ vector_type *dup);

// returns length of vector_type v
double vector_len(
      /* in     */ const vector_type *v);

void mult_vector_scalar(
      /* in out */ vector_type *v,
      /* in     */ double scalar);

// project vector_type A onto B and returns length of resultant
// a must be a unit vector. b is implicitly normalized before operation
double dot_product(
      /* in     */ const vector_type *a, 
      /* in     */ const vector_type *b);

// calculates dot product for unit vectors a and b
double dot_product_unit(
      /* in     */ const vector_type *a, 
      /* in     */ const vector_type *b);

// computes cross product of a and b and returns it in cross
void cross_product(
      /* in     */ const vector_type *a, 
      /* in     */ const vector_type *b, 
      /*    out */       vector_type *cross);

//// computes cross product of a and b and returns it in cross
//// scales a and b to unit length for calculation
//// product of |a|*|b| is returned, for detecting if either are zero length
//double unit_cross_product(
//      /* in     */ const vector_type *a, 
//      /* in     */ const vector_type *b, 
//      /*    out */       vector_type *cross);

// apply rotation matrix_type r to v. mathematically, this is r * v'
void rotate_vector(
      /* in     */ const vector_type *vec, 
      /* in     */ const matrix_type *rot, 
      /*    out */ vector_type *res);

// project vector_type onto the plane defined by normal
// NOTE: it's safe if projection and vector point to the same structure
void project_onto_plane(
      /* in     */ const vector_type *normal, 
      /* in     */ const vector_type *vector,
      /*    out */ vector_type *projection);

// convert gyro (imu) rotational representation to a vector_type describing
//    axis of rotation and rotation amount (ie, quaternion)
// NOTE: gyro vector must be storing angles in units of degrees
//    (ie, degrees per sample)
void gyro_vector_to_rotation_axis(
      /* in     */ const vector_type *gyro, 
      /*    out */ vector_type *axis,
      /*    out */       degree_type *theta);

// convert rotation vector_type + rotation (ie, quaternion) to gyro (imu)
//    representation
// NOTE: gyro vector must be storing angles in units of degrees
//    (ie, degrees per sample)
void rotation_axis_to_gyro_vector(
      /* in     */ const vector_type *axis, 
      /* in     */ const degree_type theta,
      /*    out */ vector_type *gyro);

// rotate vector_type vec about axis, by theta degrees, and store result in res
void rotate_vector_about_axis_2(
      /* in     */ const vector_type *axis, 
      /* in     */ const vector_type *vec, 
      /* in     */ const double sn,
      /* in     */ const double cs,
      /*    out */ vector_type *res);

// rotate vector_type vec about imu rotation gyro and store result in res
// NOTE: gyro vector assumed to represent gyro reference frame (which is
//    the reverse of world-centric reference frames, e.g., acc,mag)
void rotate_vector_by_gyro(
      /* in     */ const vector_type *gyro, 
      /* in     */ const vector_type *vec,
      /*    out */ vector_type *res);

// rotate vector_type vec about imu rotation gyro and store result in vec
void rotate_vector_by_gyro_in_place(
      /* in     */ const vector_type *gyro, 
      /* in out */ vector_type *vec);

////////////////////////////////////////////////////////////////////////
// display contents of vectors/matrices to stdout
//
// prints contents of v to stdout, prepended by label, with no newline
void print_vec_cont(
      /* in     */ const vector_type *v, 
      /* in     */ const char *label);

// prints contents of v to stdout, preceded by label
void print_vec(
      /* in     */ const vector_type *v, 
      /* in     */ const char* label);

// prints contents of m to stdout, preceded by label
void print_mat(
      /* in     */ const matrix_type *m, 
      /* in     */ const char* label);
      
////////////////////////////////////////////////////////////////////////


// use series of very small rotations to approximate simultaneous
//    rotation on all axes
// this has been superceded by quaternion algorithm in 
//    rotate_vector_about_axis()
void slow_rotate(
      /* in     */ const vector_type *vec, 
      /* in     */ const vector_type *dtheta, 
      /*    out */ vector_type *out);

      /* in out */ 
// like slow_rotate(), but processes two vectors at same time
void slow_rotate2(
      /* in     */ const double vec1[3], 
      /* in     */ const double vec2[3], 
      /* in     */ const double dtheta[3], 
      /*    out */ double out1[3], 
      /*    out */ double out2[3]);


void axis_angle_to_rotation_matrix(
      /* in     */ const vector_type *axis,
      /* in     */ const degree_type angle,
      /*    out */       matrix_type *mat
      );

////////////////////////////////////////////////////////////////////////

// in case we move from GCC, or switch to using -pedantic as a compiler
//    flag:
// It's a GCC extension to use 0b-------- for binary representation as
//    that's not available in C presently (only C++). binary is much
//    better for verifying bits in an i2c register, so here's a workaround
//    to allow something close to 0b in standard c
// usage: BIN(0100, 0001) as replacement to 0b01000001
// derived from 
//https://stackoverflow.com/questions/18244726/why-doesnt-c-have-binary-literals

#define BX_0000 0
#define BX_0001 1
#define BX_0010 2
#define BX_0011 3
#define BX_0100 4
#define BX_0101 5
#define BX_0110 6
#define BX_0111 7
#define BX_1000 8
#define BX_1001 9
#define BX_1010 10
#define BX_1011 11
#define BX_1100 12
#define BX_1101 13
#define BX_1110 14
#define BX_1111 15

#define BX_CAT(x) BX_ ## x

#define BIN(x,y) ((uint8_t) ((BX_CAT(x) << 4 | BX_CAT(y))))

#endif   // LIN_ALG_H
