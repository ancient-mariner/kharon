#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <assert.h>  // for NDEBUG
//#include "pinet.h"
#include "lin_alg.h"
//#include "gyro.h"
//

// redefine close enough in shorth-hand macro
#define CE(a,b)  close_enough(a, b)
#define CA(a,b)  close_enough_absolute(a, b)

////////////////////////////////////////////////////////////////////////
// static definitions

//const static vector_type x_axis = { .v = { 1.0f, 0.0f, 0.0f } };
static const vector_type y_axis = { .v = { 0.0, 1.0, 0.0 } };
//const static vector_type z_axis = { .v = { 0.0f, 0.0f, 1.0f } };

static void _copy_vector(
      /* in     */ const vector_type *v, 
      /*    out */ vector_type *dup)
{
   memcpy(dup->v, v->v, 3*sizeof dup->v[0]);
}

static void _mult_vector_matrix(
      /* in      */ const vector_type *vec,
      /* in      */ const matrix_type *m,
      /*     out */       vector_type *result
      )
{
   result->v[0] = vec->v[0]*m->m[0] + vec->v[1]*m->m[3] + vec->v[2]*m->m[6];
   result->v[1] = vec->v[0]*m->m[1] + vec->v[1]*m->m[4] + vec->v[2]*m->m[7];
   result->v[2] = vec->v[0]*m->m[2] + vec->v[1]*m->m[5] + vec->v[2]*m->m[8];
}

static void _mult_matrix_vector(
      /* in      */ const matrix_type *m,
      /* in      */ const vector_type *vec,
      /*     out */       vector_type *result
      )
{
//print_mat(m, "matrix");
//print_vec(vec, "vector");
   result->v[0] = vec->v[0]*m->m[0] + vec->v[1]*m->m[1] + vec->v[2]*m->m[2];
   result->v[1] = vec->v[0]*m->m[3] + vec->v[1]*m->m[4] + vec->v[2]*m->m[5];
   result->v[2] = vec->v[0]*m->m[6] + vec->v[1]*m->m[7] + vec->v[2]*m->m[8];
//print_vec(result, "result");
}

static void _mult_matrix(
      /* in     */ const matrix_type *mat_1, 
      /* in     */ const matrix_type *mat_2, 
      /*    out */ matrix_type *res)
{
   // below operation is for m2 * m1 -- transpose instead of rewriting
   const double * restrict m1 = mat_2->m;
   const double * restrict m2 = mat_1->m;
   double * restrict r = res->m;
   r[0] = m1[0]*m2[0] + m1[3]*m2[1] + m1[6]*m2[2];
   r[1] = m1[1]*m2[0] + m1[4]*m2[1] + m1[7]*m2[2];
   r[2] = m1[2]*m2[0] + m1[5]*m2[1] + m1[8]*m2[2];
   //
   r[3] = m1[0]*m2[3] + m1[3]*m2[4] + m1[6]*m2[5];
   r[4] = m1[1]*m2[3] + m1[4]*m2[4] + m1[7]*m2[5];
   r[5] = m1[2]*m2[3] + m1[5]*m2[4] + m1[8]*m2[5];
   //
   r[6] = m1[0]*m2[6] + m1[3]*m2[7] + m1[6]*m2[8];
   r[7] = m1[1]*m2[6] + m1[4]*m2[7] + m1[7]*m2[8];
   r[8] = m1[2]*m2[6] + m1[5]*m2[7] + m1[8]*m2[8];
}

// makes supplied vector_type unit-length
static double _unitify(
      /* in out */ vector_type *vec)
{
   double * restrict v = vec->v;
   double len = vector_len(vec);
   if (len > 0.0) {
      v[0] /= len;
      v[1] /= len;
      v[2] /= len;
   } // else, all elements are zero, so leave them as is
   return len;
}

static void _unit_vector(
      /* in     */ const vector_type *vec, 
      /*    out */ vector_type *unit)
{
   const double * restrict v = vec->v;
   double * restrict u = unit->v;
   if (fabs(vec->v[0]) + fabs(vec->v[1]) + fabs(vec->v[2]) > 0.0) {
      double scale = 1.0 / sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
      u[0] = v[0] * scale;
      u[1] = v[1] * scale;
      u[2] = v[2] * scale;
   } else {
      u[0] = 0.0;
      u[1] = 0.0;
      u[2] = 0.0;
   }
}

static void swap(double *a, double *b)
{
   double tmp = *a;
   *a = *b;
   *b = tmp;
}

// project vector_type A onto B and returns length of resultant
static double _dot_product(
      /* in     */ const vector_type *a, 
      /* in     */ const vector_type *b)
{
   const double * restrict av = a->v;
   const double * restrict bv = b->v;
   double b_len = vector_len(b);
   if (b_len > 0.0) {
      return (av[0]*bv[0] + av[1]*bv[1] + av[2]*bv[2]) / b_len;
   } else {
      return 0.0;
   }
}

// computes cross product of a and b and returns it in cross
static void _cross_product(
      /* in     */ const vector_type *vec_a, 
      /* in     */ const vector_type *vec_b, 
      /*    out */       vector_type *vec_cross)
{
   const double * restrict a = vec_a->v;
   const double * restrict b = vec_b->v;
   double * restrict cross = vec_cross->v;
   cross[0] = a[1]*b[2] - a[2]*b[1];
   cross[1] = a[2]*b[0] - a[0]*b[2];
   cross[2] = a[0]*b[1] - a[1]*b[0];
}

// project vector_type A onto B and returns length of resultant
// both a and b must be unit vectors
static double _dot_product_unit(
      /* in     */ const vector_type *a, 
      /* in     */ const vector_type *b)
{
   const double * restrict av = a->v;
   const double * restrict bv = b->v;
   return av[0]*bv[0] + av[1]*bv[1] + av[2]*bv[2];
}

// project vector onto the plane defined by the vector_type normal
// NOTE: it's safe if vector and projection point to the same structure
static void _project_onto_plane(
      /* in     */ const vector_type *normal, 
      /* in     */ const vector_type *vector, 
      /*    out */ vector_type *projection)
{
   // project vector onto normal and subtract the result from vector
   //    to get vector's projection onto plane defined by normal
   vector_type b;
   _copy_vector(normal, &b);
   _unitify(&b);
   double dot = _dot_product(vector, &b);
   for (int i=0; i<3; i++) {
      projection->v[i] = vector->v[i] - dot * b.v[i];
   }
}

// calculates rotation between two vectors, represented as output
//    from a gyro
// stores gyro representation in rot
static void _measure_rotation(
      /* in     */ const vector_type *a,
      /* in     */ const vector_type *b, 
      /*    out */       vector_type *axis,
      /*    out */       degree_type *theta
      )
{
   // get axis of rotation
   _cross_product(a, b, axis);
   // get degree of rotation
   double dot = dot_product(a, b);
   theta->degrees = R2D * acos(0.9999 * dot);
}

static void _rotate_vector_about_axis(
      /* in     */ const vector_type *axis, 
      /* in     */ const vector_type *vec, 
      /* in     */ const degree_type theta, 
      /*    out */ vector_type *res)
{
#if 1 == 1
   // implementation here is from multiplying q x v x q' and simplifying
   // alternative approach (from wikipedia, supposedly faster):
   //   t =  2q x v
   //   r = v + q0*v + q x t
   vector_type uaxis;
   unit_vector(axis, &uaxis);
   const double * restrict unit_axis = uaxis.v;
   //
   double rad = D2R * theta.degrees / 2.0;
   double sn, cs;
   sincos(rad, &sn, &cs);
   //double sn = sin(D2R * theta.degrees / 2.0);
   //double cs = cos(D2R * theta.degrees / 2.0);
   const double a = unit_axis[0] * sn;
   const double a2 = a * a;
   const double b = unit_axis[1] * sn;
   const double b2 = b * b;
   const double c = unit_axis[2] * sn;
   const double c2 = c * c;
   const double r = cs;
   const double r2 = r * r;
   const double * restrict v = vec->v;
   const double x = v[0];
   const double y = v[1];
   const double z = v[2];
   double * restrict result = res->v;
   result[0] = x*( a2-b2-c2+r2) + 2*a*(b*y+c*z) + 2*r*(b*z-c*y);
   result[1] = y*(-a2+b2-c2+r2) + 2*b*(a*x+c*z) + 2*r*(c*x-a*z);
   result[2] = z*(-a2-b2+c2+r2) + 2*c*(a*x+b*y) + 2*r*(a*y-b*x);
   // NOTE Rodrigues approach appears to fail when a 
#else
#warning "Rodrigues approach appears to have gimbal lock problems"
   // Rodrigues approach
   // v is vector
   // k is axis
   vector_type kxv;
   double cs, sn;
   //copy_vector(axis, &k);
   //copy_vector(vec, &v);
   _cross_product(axis, vec, &kxv);
   sincos(theta.degrees * D2R, &sn, &cs);
   double dtcs = dot_product(axis, vec) * (1.0 - cs);
   res->v[0] = vec->v[0]*cs + axis->v[0]*dtcs + kxv.v[0]*sn;
   res->v[1] = vec->v[1]*cs + axis->v[1]*dtcs + kxv.v[1]*sn;
   res->v[2] = vec->v[2]*cs + axis->v[2]*dtcs + kxv.v[2]*sn;
#endif   // 1 == ?
   _unitify(res);
}

// degrees between headings a and b
// result on [0,180]
static degree_type _angle_between(
      /* in     */ const degree_type a,
      /* in     */ const degree_type b
      )
{
   double delta = a.degrees - b.degrees;
   // put delta on 0,360
   while (delta < 0.0) {
      delta += 360.0;
   }
   while (delta >= 360.0) {
      delta -= 360.0;
   }
   // radially, 350 is the same as 10. reflect about 180
   if (delta > 180.0) {
      delta = 360.0 - delta;
   }
   degree_type result = { .degrees = delta };
   return result;
}

//// returns direction of north relative to ship
//// correct for attitude changes by rotating sensor's north signal to
////    to ship space
//static degree_type get_bearing(
//      /* in     */ const vector_type *acc,
//      /* in     */ const vector_type *mag
//      )
//{
//   vector_type a, m;
//   copy_vector(acc, &a);
//   copy_vector(mag, &m);
//   _unitify(&a);
//   _unitify(&m);
//   // project mag to plane orthogonal to acc. this provides up/north
//   //    axis of world
//   // get axis-angle necessary to bring acc to +y. this is what's
//   //    necessary to bring world to ship space
//   // apply this axis-angle to north vector to bring into ship space
//   // north as atan2(mag->x, mag->z)
//   vector_type world_north;
//   _project_onto_plane(&a, &m, &world_north);
//   _unitify(&world_north);
//print_vec(&world_north, "mag ortho to acc");
//   vector_type rot_axis;
//   degree_type theta;
//   _measure_rotation(&a, &y_axis, &rot_axis, &theta);
//   _unitify(&rot_axis);
//   vector_type north;
//   _rotate_vector_about_axis(&rot_axis, &world_north, theta, &north);
//   degree_type result = { .degrees = R2D * atan2f(north.v[0], north.v[2]) };
//   if (result.degrees < 0.0f) {
//      result.degrees += 360.0;
//   }
//   return result;
//}

// static definitions
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// explicit unit-test coverage

////////////////////////////////////
// initialization and set operations

void vector_init(
      /* in out */       vector_type *vec,
      /* in     */ const double a,
      /* in     */ const double b,
      /* in     */ const double c)
{
   vec->v[0] = a;
   vec->v[1] = b;
   vec->v[2] = c;
}

void matrix_init(
      /*    out */       matrix_type *m,
      /* in     */ const vector_type *row_1,
      /* in     */ const vector_type *row_2,
      /* in     */ const vector_type *row_3)
{
   m->m[0] = row_1->v[0];
   m->m[1] = row_1->v[1];
   m->m[2] = row_1->v[2];
   m->m[3] = row_2->v[0];
   m->m[4] = row_2->v[1];
   m->m[5] = row_2->v[2];
   m->m[6] = row_3->v[0];
   m->m[7] = row_3->v[1];
   m->m[8] = row_3->v[2];
}

void matrix_init_transpose(
      /*    out */       matrix_type *m,
      /* in     */ const vector_type *row_1,
      /* in     */ const vector_type *row_2,
      /* in     */ const vector_type *row_3)
{
   m->m[0] = row_1->v[0];
   m->m[3] = row_1->v[1];
   m->m[6] = row_1->v[2];
   m->m[1] = row_2->v[0];
   m->m[4] = row_2->v[1];
   m->m[7] = row_2->v[2];
   m->m[2] = row_3->v[0];
   m->m[5] = row_3->v[1];
   m->m[8] = row_3->v[2];
}

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
      /* in     */ const double p22)
{
   m->m[0] = p00;
   m->m[1] = p01;
   m->m[2] = p02;
   m->m[3] = p10;
   m->m[4] = p11;
   m->m[5] = p12;
   m->m[6] = p20;
   m->m[7] = p21;
   m->m[8] = p22;
}


///////////////////////
// comparison functions

// returns 0 if values are too far apart and nonzero (1) if they're close
int close_enough(double x, double y) {
   double delta = fabs(x - y);
   double window = 0.0011 * fabs(fabs(x) > fabs(y) ? x : y);
   return delta <= window;
}

// NOTE: port any changes to this function to the debug function
int close_enough_absolute(double x, double y) {
   double delta = fabs(x - y);
   double window = 0.001;
   return delta <= window;
}

int close_enough_debug(double x, double y) {
   double delta = fabs(x - y);
   double window = 0.001;
   printf("x=%.4g, y=%.4f, delta=%.4g, window=%.4g :: %.4g <= %.4g = %d\n", 
         (double) x, (double) y, (double) delta, (double) window,
         (double) delta, (double) window, (delta<=window));
   return delta <= window;
}

/////////////////////////
// mathematical functions

void mult_vector_matrix(
      /* in      */ const vector_type *vec,
      /* in      */ const matrix_type *m,
      /*     out */       vector_type *result
      )
{
   _mult_vector_matrix(vec, m, result);
}

void mult_matrix_vector(
      /* in      */ const matrix_type *m,
      /* in      */ const vector_type *vec,
      /*     out */       vector_type *result
      )
{
   _mult_matrix_vector(m, vec, result);
}

void mult_matrix(
      /* in     */ const matrix_type *mat_1, 
      /* in     */ const matrix_type *mat_2, 
      /*    out */ matrix_type *res)
{
   _mult_matrix(mat_1, mat_2, res);
}

void square_matrix(
      /* in     */ const matrix_type *mat, 
      /*    out */ matrix_type *res)
{
   _mult_matrix(mat, mat, res);
}

// performs a = a + s*b
void add_weighted_vector(
      /* in out */       vector_type *a, 
      /* in     */ const vector_type *b, 
      /* in     */ const double s
      )
{
   for (uint32_t i=0; i<3; i++) {
      a->v[i] = a->v[i] + s * b->v[i];
   }
}

////////////////////
// building matrices

void build_orthogonal_matrix_xy(
      /* in      */ const vector_type *primary, 
      /* in      */ const vector_type *secondary,
      /*     out */       matrix_type *mat)
{
   vector_type x, y, z;
   _copy_vector(primary, &x);
   _copy_vector(secondary, &y);
   _unitify(&x);
   //unitify(&b);   // redundant
   // vector orthogonal to plane defined by primary and secondary
   _cross_product(&x, &y, &z);
   _unitify(&z);
   // reproject secondary to the plane defined by primary and tertiary,
   //    in case primary and secondary weren't orthogonal
   _cross_product(&z, &x, &y);
   matrix_init(mat, &x, &y, &z);
}

void build_orthogonal_matrix_yx(
      /* in      */ const vector_type *primary, 
      /* in      */ const vector_type *secondary,
      /*     out */       matrix_type *mat)
{
   vector_type x, y, z;
   _copy_vector(primary, &y);
   _copy_vector(secondary, &x);
   _unitify(&y);
   // build orthogonal axis and unitify
   _cross_product(&x, &y, &z);
   _unitify(&z);
   // rebuild secondary axis (x) that's ortho to y and z
   _cross_product(&y, &z, &x);
   matrix_init(mat, &x, &y, &z);
}

void build_orthogonal_matrix_yz(
      /* in      */ const vector_type *primary, 
      /* in      */ const vector_type *secondary,
      /*     out */       matrix_type *mat)
{
   vector_type x, y, z;
   _copy_vector(primary, &y);
   _copy_vector(secondary, &z);
   _unitify(&y);
   //unitify(&c); // redundant
   _cross_product(&y, &z, &x);
   _unitify(&x);
   _cross_product(&x, &y, &z);
   matrix_init(mat, &x, &y, &z);
}


// FIXME algorithm will fail if up vector is at or below horizontal plane
void build_upright_matrix(
      /* in      */ const vector_type *up, 
      /*     out */       matrix_type *mat)
{
   // find axis-angle representation of rotation, then convert to matrix
   //
   // find rotation axis from Y to up
   vector_type axis;
   _cross_product(&y_axis, up, &axis);
   //cross_product(up, &y_axis, &axis); // rotate from up back to Y axis
   // axis-angle to matrix algorithm, from euclideanspace.com
   // www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/index.htm
   // R =   txx+c       txy-zs      txz+ys
   //       txy+zs      tyy+c       tyz-xs
   //       txz-ys      tyz+xs      tzz+c
   // where:
   //    c = cos(angle)
   //    s = sin(angle)
   //    t = 1-c
   //    x,y,z are normalized axis coordinates
   double ss = vector_len(&axis);          // sin is len of cross product
   const double s = ss>1.0 ? 1.0 : ss;   // correct for pathologic error
   const double c = sqrt(1.0 - s * s);   // sin^2 + cos^2 = 1
   const double t = 1.0 - c;
   unitify(&axis);
   const double x = axis.v[0];
   const double y = axis.v[1];
   const double z = axis.v[2];
   // main diagonal
   mat->m[0] = t * x * x + c;
   mat->m[4] = t * y * y + c;
   mat->m[8] = t * z * z + c;
   // reflections
   mat->m[1] = t * x * y - z * s;
   mat->m[3] = t * x * y + z * s;
   //
   mat->m[2] = t * x * z + y * s;
   mat->m[6] = t * x * z - y * s;
   //
   mat->m[5] = t * y * z - x * s;
   mat->m[7] = t * y * z + x * s;
}


/////////////////////////
// unit length operations
//
double unitify(
      /* in out */ vector_type *vec)
{
   return _unitify(vec);
}

void unit_vector(
      /* in     */ const vector_type *vec, 
      /*    out */ vector_type *unit)
{
   _unit_vector(vec, unit);
}

void measure_rotation(
      /* in     */ const vector_type *a,
      /* in     */ const vector_type *b, 
      /*    out */       vector_type *axis,
      /*    out */       degree_type *theta
      )
{
   _measure_rotation(a, b, axis, theta);
}

////////////////////
// pitch, roll and heading functions assume that up and north are unit vectors
// TODO before uncommenting, simplify pitch/roll calculations and fix heading
//degree_type get_pitch(
//      /* in     */ const vector_type *acc
//      )
//{
//   vector_type a;
//   _copy_vector(acc, &a);
//   _unitify(&a);
//   // project ACC to the YZ plane, then normalize resultant vector
//   // pitch is -sin(z)
//   vector_type projection;
//   _project_onto_plane(&x_axis, &a, &projection);
//   _unitify(&projection);
//   double z = projection.v[2];
//   z = z > 1.0f ? 1.0f : z;
//   z = z < -1.0f ? -1.0f : z;
//   // multiply by just under 1.0 to prevent rounding errors from allowing
//   //    Z to be slightly more than 1.0
//   degree_type degs = { .degrees = R2D * asin(z) };
//   return degs;
//}
//
//degree_type get_roll(
//      /* in     */ const vector_type *acc
//      )
//{
//   vector_type a;
//   _copy_vector(acc, &a);
//   _unitify(&a);
//   // project ACC to the XY plane, then normalize resultant vector
//   // pitch is -sin(x)
//   vector_type projection;
//   _project_onto_plane(&z_axis, &a, &projection);
//   _unitify(&projection);
//   double x = projection.v[0];
//   x = x > 1.0f ? 1.0f : x;
//   x = x < -1.0f ? -1.0f : x;
//   // multiply by just under 1.0 to prevent rounding errors from allowing
//   //    X to be slightly more than 1.0
//   degree_type roll = { .degrees = R2D * asin(x) };
//   return roll;
//}
//
//degree_type get_north(
//      /* in     */ const vector_type *acc,
//      /* in     */ const vector_type *mag
//      )
//{
//   degree_type north = get_bearing(acc, mag);
//   // north is inverse of bearing
//   north.degrees = 360.0f - north.degrees;
//   return north;
//}
//
//degree_type get_heading(
//      /* in     */ const vector_type *acc,
//      /* in     */ const vector_type *mag
//      )
//{
//   return get_bearing(acc, mag);
//}

void rotate_vector_about_axis(
      /* in     */ const vector_type *axis, 
      /* in     */ const vector_type *vec, 
      /* in     */ const degree_type theta, 
      /*    out */ vector_type *res)
{
   _rotate_vector_about_axis(axis, vec, theta, res);
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// the below functions lack explicit unit-test coverage

// degrees between headings a and b
// result on [0,180]
degree_type angle_between(
      /* in     */ const degree_type a,
      /* in     */ const degree_type b
      )
{
   return _angle_between(a, b);
}


void matrix_transpose(
      /* in out */ matrix_type *m)
{
   swap(&m->m[1], &m->m[3]);
   swap(&m->m[2], &m->m[6]);
   swap(&m->m[5], &m->m[7]);
}

void copy_matrix(
      /* in     */ const matrix_type *mat, 
      /*    out */       matrix_type *dup)
{
   memcpy(dup->m, mat->m, 9*sizeof(mat->m[0]));
}


//void get_roll_pitch_yaw(
//      /* in     */ matrix_type *m,
//      /*    out */ double *roll,
//      /*    out */ double *pitch,
//      /*    out */ double *yaw)
//{
//   *roll = atan2f(m->m[7], m->m[8]);
//   *pitch = atan2f(-m->m[6], sqrt(m->m[7]*m->m[7] + m->m[8]*m->m[8]));
//   *yaw = atan2f(m->m[3], m->m[0]);
//}

void zero_vector(
      /* in out */ vector_type *vec)
{
   vec->v[0] = 0.0;
   vec->v[1] = 0.0;
   vec->v[2] = 0.0;
}

void zero_matrix(
      /* in out */ matrix_type *mat)
{
   for (int i=0; i<9; i++)
      mat->m[i] = 0.0;
}

void reset_vector(
      /* in out */ vector_type *vec)
{
   vec->v[0] = 0.0;
   vec->v[1] = 0.0;
   vec->v[2] = 0.0;
}

void copy_vector(
      /* in     */ const vector_type *v, 
      /*    out */ vector_type *dup)
{
   _copy_vector(v, dup);
}

void copy_vector_unitify(
      /* in     */ const vector_type *v, 
      /*    out */ vector_type *dup)
{
   memcpy(dup->v, v->v, 3*sizeof *dup->v);
   unitify(dup);
}

void mult_vector_scalar(
      /* in out */ vector_type *vec,
      /* in     */ double scalar)
{
   vec->v[0] *= scalar;
   vec->v[1] *= scalar;
   vec->v[2] *= scalar;
}


double vector_len(const vector_type *vec)
{
   const double * restrict v = vec->v;
   return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// project vector_type A onto B and returns length of resultant
// a must be a unit vector
double dot_product(
      /* in     */ const vector_type *a, 
      /* in     */ const vector_type *b)
{
   return _dot_product(a, b);
}

// project vector_type A onto B and returns length of resultant
// both a and b must be unit vectors
double dot_product_unit(
      /* in     */ const vector_type *a, 
      /* in     */ const vector_type *b)
{
   return _dot_product_unit(a, b);
}

// computes cross product of a and b and returns it in cross
void cross_product(
      /* in     */ const vector_type *vec_a, 
      /* in     */ const vector_type *vec_b, 
      /*    out */       vector_type *vec_cross)
{
   _cross_product(vec_a, vec_b, vec_cross);
}

//// computes cross product of a and b and returns it in cross
//// scales a and b to unit length
//double unit_cross_product(
//      /* in     */ const vector_type *vec_a, 
//      /* in     */ const vector_type *vec_b, 
//      /*    out */       vector_type *vec_cross)
//{
//   const double len = vector_len(vec_a) * vector_len(vec_b);
//   if (len == 0)
//      return 0;
//   const double * restrict a = vec_a->v;
//   const double * restrict b = vec_b->v;
//   double * restrict cross = vec_cross->v;
//   cross[0] = (a[1]*b[2] - a[2]*b[1]) / len;
//   cross[1] = (a[2]*b[0] - a[0]*b[2]) / len;
//   cross[2] = (a[0]*b[1] - a[1]*b[0]) / len;
//   return len;
//}

void identity_matrix(matrix_type *mat)
{
   static const double eye[9] = { 
         1.0f, 0.0f, 0.0f, 
         0.0f, 1.0f, 0.0f, 
         0.0f, 0.0f, 1.0f 
   };
   memcpy(mat, eye, sizeof eye);
}


// apply rotation matrix_type r to v. mathematically, this is r * v'
void rotate_vector(
      /* in     */ const vector_type *vec, 
      /* in     */ const matrix_type *rot, 
      /*    out */ vector_type *res)
{
   const double * restrict v = vec->v;
   const double * restrict r = rot->m;
   double * restrict out = res->v;
   out[0] = v[0]*r[0] + v[1]*r[1] + v[2]*r[2];
   out[1] = v[0]*r[3] + v[1]*r[4] + v[2]*r[5];
   out[2] = v[0]*r[6] + v[1]*r[7] + v[2]*r[8];
}

// project vector onto the plane defined by the vector_type normal
// NOTE: it's safe if vector and projection point to the same structure
void project_onto_plane(
      /* in     */ const vector_type *normal, 
      /* in     */ const vector_type *vector, 
      /*    out */ vector_type *projection)
{
   _project_onto_plane(normal, vector, projection);
}

////////////////////////////////////////////////////////////////////////
// conversion to/from gyro rotation and quaternion rotation about axis
//
// rotation axis is equivalent to vector formed by gyro representation
void gyro_vector_to_rotation_axis(
      /* in     */ const vector_type *gyro, 
      /*    out */       vector_type *axis,
      /*    out */       degree_type *theta)
{
   // gyro rotation values must be in degrees
   theta->degrees = vector_len(gyro);
   // gyro vector is negative of rotation axis
   for (int i=0; i<3; i++)
      axis->v[i] = -gyro->v[i];
   unitify(axis);
}

// unit rotation axis is equivalent to unit of gyro representation
// gyro must be scaled by theta
void rotation_axis_to_gyro_vector(
      /* in     */ const vector_type *axis, 
      /* in     */ const degree_type theta,
      /*    out */ vector_type *gyro)
{
   // NOTE: gyro must be storing degrees
   double len = vector_len(axis);
   if (len > 0) {
      const double * restrict v = axis->v;
      double * restrict g = gyro->v;
      for (int i=0; i<3; i++)
         // gyro representation is world relative to body while
         //    acc,mag frame of reference is body relative to world
         //    (or vice versa).
         //    need to invert vector to flip reference frame
         g[i] = -v[i] * theta.degrees / len;
   }
}

void rotate_vector_about_axis_2(
      /* in     */ const vector_type *axis, 
      /* in     */ const vector_type *vec, 
      /* in     */ const double sn,
      /* in     */ const double cs,
      /*    out */ vector_type *res)
{
   // Rodrigues approach
   // v is vector
   // k is axis
   vector_type kxv;
   _cross_product(axis, vec, &kxv);
   double dtcs = dot_product(axis, vec) * (1.0 - cs);
   res->v[0] = vec->v[0]*cs + axis->v[0]*dtcs + kxv.v[0]*sn;
   res->v[1] = vec->v[1]*cs + axis->v[1]*dtcs + kxv.v[1]*sn;
   res->v[2] = vec->v[2]*cs + axis->v[2]*dtcs + kxv.v[2]*sn;
}

// NOTE: gyro vector assumed to represent world relative to device
void rotate_vector_by_gyro(
      /* in     */ const vector_type *gyro,  // units of degrees
      /* in     */ const vector_type *vec,
      /*    out */ vector_type *res)
{
   vector_type axis;
   degree_type theta;
   gyro_vector_to_rotation_axis(gyro, &axis, &theta);
   rotate_vector_about_axis(&axis, vec, theta, res);
}

void rotate_vector_by_gyro_in_place(
      /* in     */ const vector_type *gyro, 
      /* in out */ vector_type *vec)
{
   vector_type res;
   rotate_vector_by_gyro(gyro, vec, &res);
   for (int i=0; i<3; i++)
      vec->v[i] = res.v[i];
}

////////////////////////////////////////////////////////////////////////

void print_vec_cont(
      /* in     */ const vector_type *v, 
      /* in     */ const char *label)
{
   printf("%s %7.3f%7.3f%7.3f    ", label, 
         (double) (v->v[0]), (double) (v->v[1]), (double) (v->v[2]));
}

void print_vec(
      /* in     */ const vector_type *v, 
      /* in     */ const char* label)
{
   printf("---------  %s = ", label);
   printf("[%9.4f,%9.4f,%9.4f ]\n", 
   //printf("%9.4f%9.4f%9.4f\n", 
         (double) (v->v[0]), (double) (v->v[1]), (double) (v->v[2]));
}

void print_mat(
      /* in     */ const matrix_type *m, 
      /* in     */ const char* label)
{
   int i;
   printf("---------  %s\n", label);
   printf("[");
   for (i=0; i<9; i++) {
      if ((i%3) == 0)
         printf("[");
      printf("%10.5f", (double) (m->m[i]));
      if (i == 8)
         printf("]]\n");
      else if ((i%3) == 2)
         printf("],\n ");
   }
}
      
//void eye3(double m[9])
//{
//   m[0] = 1.0;
//   m[1] = 0.0;
//   m[2] = 0.0;
//   m[3] = 0.0;
//   m[4] = 1.0;
//   m[5] = 0.0;
//   m[6] = 0.0;
//   m[7] = 0.0;
//   m[8] = 1.0;
//}
//
//void square_matrix(const double m[9], double m2[9])
//{
//   m2[0] = m[0]*m[0] + m[1]*m[3] + m[2]*m[6];
//   m2[1] = m[0]*m[1] + m[1]*m[4] + m[2]*m[7];
//   m2[2] = m[0]*m[2] + m[1]*m[5] + m[2]*m[8];
//   //
//   m2[3] = m[3]*m[0] + m[4]*m[3] + m[5]*m[6];
//   m2[4] = m[3]*m[1] + m[4]*m[4] + m[5]*m[7];
//   m2[5] = m[3]*m[2] + m[4]*m[5] + m[5]*m[8];
//   //
//   m2[6] = m[6]*m[0] + m[7]*m[3] + m[8]*m[6];
//   m2[7] = m[6]*m[1] + m[7]*m[4] + m[8]*m[7];
//   m2[8] = m[6]*m[2] + m[7]*m[5] + m[8]*m[8];
//}
//
//// mathematically, this is r * v'
//void rotate_vector(const double v[3], const double r[9], 
//      double out[3])
//{
//   out[0] = v[0]*r[0] + v[1]*r[1] + v[2]*r[2];
//   out[1] = v[0]*r[3] + v[1]*r[4] + v[2]*r[5];
//   out[2] = v[0]*r[6] + v[1]*r[7] + v[2]*r[8];
//}
//
//void vec_sincos(const double vec[3], double s[3], 
//      double c[3])
//{
//   sincos(vec[0] * M_PI / 180.0, &s[0], &c[0]);
//   sincos(vec[1] * M_PI / 180.0, &s[1], &c[1]);
//   sincos(vec[2] * M_PI / 180.0, &s[2], &c[2]);
//}


//// create rotation matrix_type necessary to rotate v1 to v2
//// algorithm from:
////    math.stackexchange.com/questions/180418/calculate-rotation-matrix_type-to-align=vector_type-a-to-vector_type-b-in-3d
//void rotation_matrix_from_vectors(const double v1[3], const double v2[3],
//      double m[9])
//{
//   double z1[3], z2[3];
//   copy_vector(v1, z1);
//   unitify(z1);
//   copy_vector(v2, z2);
//   unitify(z2);
//   double w[3];
//   cross_product(z1, z2, w);
//   double s = vector_len(w);
//   double skew[9];
//   skew[0] = 0.0;
//   skew[1] = -w[2];
//   skew[2] = w[1];
//   skew[3] = w[2];
//   skew[4] = 0.0;
//   skew[5] = -w[0];
//   skew[6] = -w[1];
//   skew[7] = w[0];
//   skew[8] = 0.0;
//   double skew2[9];
//   square_matrix(skew, skew2);
//   double c = dot_product(z1, z2);
//   eye3(m);
//   for (int i=0; i<9; i++)
//      m[i] += skew[i] + skew2[i] * (1.0 - c) / (s * s);
//}


//// project vector_type onto the plane defined by the vector_type normal
//void project_onto_plane(const double normal[3], 
//      const double vector_type[3], double projection[3])
//{
//   // project vector_type onto normal and subtract the result from vector_type
//   //    to get vector_type's projection onto plane defined by normal
//   double dot = dot_product(vector_type, normal);
//   int i;
//   for (i=0; i<3; i++)
//      projection[i] = vector_type[i] - dot * normal[i];
//}
//
////////////////////////////////////////////////////////////////////////
//
// procedures for rotating a vector_type about axes using very small steps
// NOTE: this is superceded by quaternion rotations but remains here
//    to validate algorithm there

static void _construct_rotation_zyx(
      vector_type *vec_s, 
      vector_type *vec_c, 
      matrix_type *rot)
{
   double * restrict r = rot->m;
   double * restrict s = vec_s->v;
   double * restrict c = vec_c->v;
   r[0] = c[1] * c[2];
   r[1] = c[2] * s[0] * s[1] - c[0] * s[2];
   r[2] = c[0] * c[2] * s[1] + s[0] * s[2];
   r[3] = c[1] * s[2];
   r[4] = c[0] * c[2] + s[0] * s[1] * s[2];
   r[5] = -c[2] * s[0] + c[0] * s[1] * s[2];
   r[6] = -s[1];
   r[7] = c[1] * s[0];
   r[8] = c[0] * c[1];
   //
   if (fabs(sqrt(r[0]*r[0] + r[1]*r[1] + r[2]*r[2])-1.0) > 1e-4)
      goto len_err;
   if (fabs(sqrt(r[3]*r[3] + r[4]*r[4] + r[5]*r[5])-1.0) > 1e-4)
      goto len_err;
   if (fabs(sqrt(r[6]*r[6] + r[7]*r[7] + r[8]*r[8])-1.0) > 1e-4)
      goto len_err;
   return;
len_err:
   fprintf(stderr, "Non-unit length found in constructed rotation matrix_type\n");
   print_vec(vec_s, "sin values");
   print_vec(vec_c, "cos values");
   print_mat(rot, "rotation matrix_type");
}

// use series of very small rotations to approximate simultaneous
//    rotation on all axes
// a more efficient approach is to use quaternions. optimize later
void slow_rotate(
      /* in     */ const vector_type *vec, 
      /* in     */ const vector_type *gyro, 
      /*    out */ vector_type *out)
{
   const double MAX_DTHETA = 0.25;
   double len = vector_len(gyro);
   double n = (double) ((int) (0.999 + len / MAX_DTHETA));
   vector_type c, s;
   //double c[3], s[3];
   matrix_type rzyx;
   //double rzyx[9];
   //double tmp[3];
   vector_type tmp;
   int i;
   for (i=0; i<3; i++)
      tmp.v[i] = vec->v[i];
   // negative because gyro is opposite polarity to rotation vector
   for (i=0; i<3; i++)
      sincos((-gyro->v[i] / n) * D2R, &s.v[i], &c.v[i]);
   _construct_rotation_zyx(&s, &c, &rzyx);
   for (i=0; i<n; i++) {
      rotate_vector(&tmp, &rzyx, out);
      copy_vector(out, &tmp);
   }
   unitify(out);
}

//// like slow_rotate(), but processes two vectors at same time
//void slow_rotate2(const double vec1[3], const double vec2[3], 
//      const double dtheta[3], 
//      double out1[3], double out2[3])
//{
//   const double MAX_DTHETA = 0.25;
//   double len = vector_len(dtheta);
//   int n = (int) (0.999 + len / MAX_DTHETA);
//   double c[3], s[3];
//   double rzyx[9];
//   double tmp1[3];
//   double tmp2[3];
//   int i;
//   for (i=0; i<3; i++) {
//      tmp1[i] = vec1[i];
//      tmp2[i] = vec2[i];
//   }
//   for (i=0; i<3; i++)
//      sincos((dtheta[i] / n) * D2R, &s[i], &c[i]);
//      //sincos((-dtheta[i] / n) * D2R, &s[i], &c[i]);
//   _construct_rotation_zyx(s, c, rzyx);
//   for (i=0; i<n; i++) {
//      rotate_vector(tmp1, rzyx, out1);
//      memcpy(tmp1, out1, 3*sizeof(out1[0]));
//      rotate_vector(tmp2, rzyx, out2);
//      memcpy(tmp2, out2, 3*sizeof(out2[0]));
//   }
//   unitify(out1);
//   unitify(out2);
//}

void axis_angle_to_rotation_matrix(
      /* in     */ const vector_type *axis,
      /* in     */ const degree_type angle,
      /*    out */       matrix_type *mat
      )
{
   // algorithm from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
   vector_type ax;
   copy_vector(axis, &ax);
   unitify(&ax);
   double s, c;
   sincos(D2R * angle.degrees, &s, &c);
   double t = 1.0 - c;
   double x = ax.v[0];
   double y = ax.v[1];
   double z = ax.v[2];
   mat->m[0] = t*x*x + c;
   mat->m[1] = t*x*y - z*s;
   mat->m[2] = t*x*z + y*s;
   mat->m[3] = t*y*x + z*s;
   mat->m[4] = t*y*y + c;
   mat->m[5] = t*y*z - x*s;
   mat->m[6] = t*z*x - y*s;
   mat->m[7] = t*z*y + x*s;
   mat->m[8] = t*z*z + c;
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

#if defined(LIN_ALG_TEST)
#include "image.h"

static const vector_type x_axis = { .v = { 1.0f, 0.0f, 0.0f } };
//const static vector_type y_axis = { .v = { 0.0f, 1.0f, 0.0f } };
static const vector_type z_axis = { .v = { 0.0f, 0.0f, 1.0f } };

static uint32_t test_build_upright_matrix(void);
static uint32_t test_vector_init(void);
static uint32_t test_matrix_init(void);
static uint32_t test_mult_vector_matrix(void);
static uint32_t test_mult_matrix_vector(void);
static uint32_t test_mult_matrix(void);
//static uint32_t test_boilerplate(void);
static uint32_t test_build_orthogonal_matrix(void);
static uint32_t test_close_enough(void);
static uint32_t test_unitify(void);

static uint32_t test_measure_rotation(void);
//static uint32_t test_get_pitch(void);
//static uint32_t test_get_roll(void);
//static uint32_t test_get_heading(void);
//static uint32_t test_get_north_heading(void);

static uint32_t test_add_weighted_vector(void);
static uint32_t test_rotate_vector_about_axis(void);

////////////////////////////

static uint32_t test_rotate_vector_about_axis()
{
   uint32_t errs = 0;
   printf("Testing rotate_vector_about_axis\n");
   vector_type a;
   degree_type theta;
   // rotate Z around Y axis by 90 degrees
   theta.degrees = 90.0;
   rotate_vector_about_axis(&y_axis, &z_axis, theta, &a);
   if (fabs(a.v[0] - 1.0) > 0.001) {
      fprintf(stderr, "Z rotated 90deg about Y should have X=1, not %f\n", a.v[0]);
      errs++;
   }
   if (fabs(a.v[2]) > 0.001) {
      fprintf(stderr, "Z rotated 90deg about Y should have Z=0, not %f\n", a.v[2]);
      errs++;
   }
   // line x=y z=0 rotated -45 degrees about -Z should go to +Y
   vector_type b = { .v = { 0.5, 0.5, 0.0 } };
   vector_type neg_z = { .v = { 0.0, 0.0, -1.0 } };
   theta.degrees = -45.0;
   rotate_vector_about_axis(&neg_z, &b, theta, &a);
   if (fabs(a.v[0]) > 0.01) {
      fprintf(stderr, "X=Y rotated -45deg about -Y should have X=1, not %f\n", 
            a.v[0]);
      errs++;
   }
   if (fabs(a.v[1] - 1.0) > 0.001) {
      fprintf(stderr, "X=Y rotated -45deg about -Y should have Y=1, not %f\n", 
            a.v[1]);
      errs++;
   }
   if (fabs(a.v[2]) > 0.01) {
      fprintf(stderr, "X=Y rotated -45deg about -Y should have Z=0, not %f\n", 
            a.v[2]);
      errs++;
   }
   //////////////////
   return errs;
}

static uint32_t test_add_weighted_vector()
{
   uint32_t errs = 0;
   printf("Testing add_weighted_vector\n");
   vector_type a;
   copy_vector(&x_axis, &a);
   add_weighted_vector(&a, &x_axis, 4.0);
   if (fabs(a.v[0] - 5.0) > 0.001) {
      fprintf(stderr, "1 + 4 = 5, not %f\n", a.v[0]);
      errs++;
   }
   add_weighted_vector(&a, &x_axis, -1.0);
   if (fabs(a.v[0] - 4.0) > 0.001) {
      fprintf(stderr, "5 - 1 = 4, not %f\n", a.v[0]);
      errs++;
   }
   add_weighted_vector(&a, &y_axis, 3.0);
   if (fabs(a.v[1] - 3.0) > 0.001) {
      fprintf(stderr, "0 + 3 = 3, not %f\n", a.v[1]);
      errs++;
   }
   double len = vector_len(&a);
   if (fabs(len - 5.0) > 0.01) {
      fprintf(stderr, "3^2 + 4^2 = 5^2, not %f^2\n", a.v[1]);
      errs++;
   }
   //////////////////
   return errs;
}

//static uint32_t check_heading(vector_type *mag, degree_type dir) 
//{
//   degree_type heading = get_heading(&y_axis, mag);
//   degree_type delta = _angle_between(heading, dir);
//   // test values are from uncorrected sensor, so leave a margin
//   //    of error. make sure we at least have the octant right
//   if (delta.degrees > 3.0f) {
//      printf("    Failed -- heading is %f, not %f\n", 
//            (double) dir.degrees, (double) heading.degrees);
//      return 1;
//   }
//   return 0;
//}
//
//static uint32_t check_north(vector_type *mag, degree_type nor) 
//{
//   degree_type north = get_north(&y_axis, mag);
//   degree_type delta = _angle_between(north, nor);
//   // test values are from uncorrected sensor, so leave a margin
//   //    of error. make sure we at least have the octant right
//   if (delta.degrees > 3.0f) {
//      printf("    Failed -- north is %f, not %f\n", 
//            (double) nor.degrees, (double) north.degrees);
//      return 1;
//   }
//   return 0;
//}
//
//static uint32_t test_get_north_heading()
//{
//   uint32_t errs = 0;
//   printf("Testing get_north and get_heading\n");
//   // actual readings from magnetometer
//   //    (Leon, Nicaragua)
//   vector_type mag_000 = { .v = {  0.01f, -0.25f,  0.30f } };
//   vector_type mag_045 = { .v = {  0.23f, -0.24f,  0.22f } };
//   vector_type mag_090 = { .v = {  0.30f, -0.26f, -0.00f } };
//   vector_type mag_135 = { .v = {  0.20f, -0.23f, -0.22f } };
//   vector_type mag_180 = { .v = { -0.01f, -0.23f, -0.32f } };
//   vector_type mag_225 = { .v = { -0.23f, -0.25f, -0.25f } };
//   vector_type mag_270 = { .v = { -0.29f, -0.24f, -0.01f } };
//   vector_type mag_315 = { .v = { -0.20f, -0.25f,  0.20f } };
//   degree_type dir_000 = { .degrees =   0.0f };
//   degree_type dir_045 = { .degrees =  45.0f };
//   degree_type dir_090 = { .degrees =  90.0f };
//   degree_type dir_135 = { .degrees = 135.0f };
//   degree_type dir_180 = { .degrees = 180.0f };
//   degree_type dir_225 = { .degrees = 225.0f };
//   degree_type dir_270 = { .degrees = 270.0f };
//   degree_type dir_315 = { .degrees = 315.0f };
//   degree_type nor_000 = { .degrees =    0.0f };
//   degree_type nor_045 = { .degrees =  -45.0f };
//   degree_type nor_090 = { .degrees =  -90.0f };
//   degree_type nor_135 = { .degrees = -135.0f };
//   degree_type nor_180 = { .degrees = -180.0f };
//   degree_type nor_225 = { .degrees = -225.0f };
//   degree_type nor_270 = { .degrees = -270.0f };
//   degree_type nor_315 = { .degrees = -315.0f };
//   //////////////////
//   errs += check_heading(&mag_000, dir_000);
//   errs += check_heading(&mag_045, dir_045);
//   errs += check_heading(&mag_090, dir_090);
//   errs += check_heading(&mag_135, dir_135);
//   errs += check_heading(&mag_180, dir_180);
//   errs += check_heading(&mag_225, dir_225);
//   errs += check_heading(&mag_270, dir_270);
//   errs += check_heading(&mag_315, dir_315);
//   errs += check_north(&mag_000, nor_000);
//   errs += check_north(&mag_045, nor_045);
//   errs += check_north(&mag_090, nor_090);
//   errs += check_north(&mag_135, nor_135);
//   errs += check_north(&mag_180, nor_180);
//   errs += check_north(&mag_225, nor_225);
//   errs += check_north(&mag_270, nor_270);
//   errs += check_north(&mag_315, nor_315);
//   //////////////////
//   return errs;
//}
//
//static uint32_t test_get_heading()
//{
//   uint32_t errs = 0;
//   printf("Testing get_heading\n");
//   degree_type heading;
//   degree_type tilt;
//   vector_type mag = { .v = { 0.0f, -0.8f, 0.5f } };
//   _unitify(&mag);
//   vector_type sensor_mag, sensor_mag2, sensor_acc, ship_axis;
//   degree_type north = { .degrees = 0.0 };
//   degree_type delta;
//   degree_type expected;
//   //////////////////
//   // north, level
//   heading = get_heading(&y_axis, &mag);
//   delta = _angle_between(heading, north);
//   if (delta.degrees > 0.001f) {
//      printf("    Failed -- flat northward heading is %f, not 0.0\n", 
//            (double) heading.degrees);
//      errs++;
//   }
//   //////////////////
//   // north 30% off left bow, level
//   expected.degrees = 0.0;
//   north.degrees = 30.0;
//   _rotate_vector_about_axis(&y_axis, &mag, north, &sensor_mag);
//   heading = get_heading(&y_axis, &sensor_mag);
//   delta = _angle_between(heading, expected);
//   if (fabs(delta.degrees - 30.0f) > 0.01f) {
//      printf("    Failed -- north at -30 deg has heading %f, not 30.0\n", 
//            (double) heading.degrees);
//      errs++;
//   }
//   //////////////////
//   // north , +30 degree roll
//   expected.degrees = 0.0;
//   tilt.degrees = 30.0;
//   _rotate_vector_about_axis(&z_axis, &mag, tilt, &sensor_mag);
//   _rotate_vector_about_axis(&z_axis, &y_axis, tilt, &sensor_acc);
//   heading = get_heading(&sensor_acc, &sensor_mag);
//   delta = _angle_between(heading, expected);
//   if (fabs(delta.degrees) > 0.001f) {
//      printf("    Failed -- north with 30 roll has heading %f, not 0.0\n", 
//            (double) heading.degrees);
//      errs++;
//   }
//   //////////////////
//   // north 30% off left bow, +30 deg roll
//   // rotate ship axis and mag sensor reading so north is off left bow
//   expected.degrees = 30.0;
//   north.degrees = 30.0;
//   _rotate_vector_about_axis(&y_axis, &mag, north, &sensor_mag);
//   _rotate_vector_about_axis(&y_axis, &z_axis, north, &ship_axis);
//   // rotate acc and mag sensor readings about ship axis for roll
//   // + rotation on Z axis puts X axis up, which is left-up/right-down,
//   //    which is positive roll
//   tilt.degrees = 30.0;
//   _rotate_vector_about_axis(&ship_axis, &y_axis, tilt, &sensor_acc);
//   _rotate_vector_about_axis(&ship_axis, &sensor_mag, tilt, &sensor_mag2);
//   // get heading from mag (which is -north)
//   heading = get_heading(&sensor_acc, &sensor_mag2);
//   delta = _angle_between(heading, expected);
//   if (fabs(delta.degrees) > 0.01) {
//      printf("    Failed -- north at -30 deg, left tilt of 30 deg, has "
//            "heading %f, not 30.0\n", (double) heading.degrees);
//      errs++;
//   }
//   //////////////////
//   return errs;
//}
//
//static uint32_t test_get_roll()
//{
//   uint32_t errs = 0;
//   printf("Testing get_roll\n");
//   degree_type roll;
//   //////////////////
//   roll = get_roll(&y_axis);
//   if (fabs(roll.degrees) > 0.001) {
//      printf("    Failed -- vertical acc has roll %f, not 0.0\n", 
//            (double) roll.degrees);
//      errs++;
//   }
//   //////////////////
//   roll = get_roll(&z_axis);
//   if (fabs(roll.degrees) > 0.001) {
//      printf("    Failed -- nose up has roll %f, not 0.0\n", 
//            (double) roll.degrees);
//      errs++;
//   }
//   //////////////////
//   // acc reads X axis, meaning that left side of boat is pointing up
//   roll = get_roll(&x_axis);
//   if (fabs(roll.degrees - 90.0) > 0.1) {
//      printf("    Failed -- right ear in water has roll %f, not 90.0\n", 
//            (double) roll.degrees);
//      errs++;
//   }
//   //////////////////
//   vector_type a = { .v = { -0.5, 0.5, 0.0 } };
//   unitify(&a);
//   roll = get_roll(&a);
//   if (fabs(roll.degrees + 45.0f) > 0.1f) {
//      printf("    Failed -- left ear down 45 deg has roll %f, not -45.0\n", 
//            (double) roll.degrees);
//      errs++;
//   }
//   //////////////////
//   return errs;
//}
//
//static uint32_t test_get_pitch()
//{
//   uint32_t errs = 0;
//   printf("Testing get_pitch\n");
//   degree_type pitch;
//   //////////////////
//   pitch = get_pitch(&y_axis);
//   if (fabs(pitch.degrees) > 0.001f) {
//      printf("    Failed -- vertical acc has pitch %f, not 0.0\n", 
//            (double) pitch.degrees);
//      errs++;
//   }
//   //////////////////
//   // acc reads +z axis, meaing nose is pointing up
//   pitch = get_pitch(&z_axis);
//   if (fabs(pitch.degrees - 90.0f) > 0.1f) {
//      printf("    Failed -- nose up has pitch %f, not 90.0\n", 
//            (double) pitch.degrees);
//      errs++;
//   }
//   //////////////////
//   // acc reads +x axis, meaning left side is facing up
//   pitch = get_pitch(&x_axis);
//   if (fabs(pitch.degrees) > 0.001f) {
//      printf("    Failed -- right ear in water has pitch %f, not 0.0\n", 
//            (double) pitch.degrees);
//      errs++;
//   }
//   //////////////////
//   vector_type a = { .v = { 0.0f, 0.5f, -0.5f } };
//   unitify(&a);
//   pitch = get_pitch(&a);
//   if (fabs(pitch.degrees + 45.0f) > 0.1f) {
//      printf("    Failed -- nose down 45 deg has pitch %f, not -45.0\n", 
//            (double) pitch.degrees);
//      errs++;
//   }
//   //////////////////
//   return errs;
//}

static uint32_t test_measure_rotation()
{
   uint32_t errs = 0;
   printf("Testing measure_rotation\n");
   vector_type rot;
   degree_type theta;
   measure_rotation(&y_axis, &z_axis, &rot, &theta);
   double len = vector_len(&rot);
   if ((fabs(len - 1.0)) > 0.0001) {
      printf("    Failed -- |Y x Z| = %f, not 1.0\n", (double) len);
      errs++;
   }
   double dot = dot_product(&rot, &x_axis);
   if (fabs(dot - 1.0) > 0.0001) {
      printf("    Failed -- dot(YxZ, X) = %f, not 1.0\n", (double) dot);
      errs++;
   }
   //////////////////
   measure_rotation(&z_axis, &y_axis, &rot, &theta);
   dot = dot_product(&rot, &x_axis);
   if (fabs(dot + 1.0) > 0.0001) {
      printf("    Failed -- dot(ZxY, X) = %f, not -1.0\n", (double) dot);
      errs++;
   }
   //////////////////
   vector_type a = { .v = { 0.866025, 0.5, 0.0 } };
   measure_rotation(&a, &y_axis, &rot, &theta);
   if (fabs(theta.degrees - 60.0) > 0.01) {
      printf("    Failed -- A -> Y degrees = %f, not 60.0\n", 
            (double) theta.degrees);
      errs++;
   }
   //////////////////
   return errs;
}


static uint32_t test_unitify()
{
   uint32_t errs = 0;
   printf("Testing unitify\n");
   vector_type a = { .v = { 2.0, 0.4, 0.2 } };
   vector_type b;
   unit_vector(&a, &b);
   double len = vector_len(&b);
   if ((fabs(len) - 1.0) > 0.0001) {
      printf("    Failed -- vector length = %f\n", (double) len);
      errs++;
   }
   unitify(&a);
   len = vector_len(&a);
   if ((fabs(len) - 1.0) > 0.0001) {
      printf("    Failed -- vector length = %f\n", (double) len);
      errs++;
   }
   return errs;
}


static uint32_t test_close_enough()
{
   uint32_t errs = 0;
   printf("Testing close_enough\n");
   double a, b;
   a = 0.999; b = 1.0;
   if (!close_enough(a, b)) {
      fprintf(stderr, "  Fail: %f is close enough to %f\n", 
            (double) a, (double) b);
      printf("%f  \t%f\n", fabs(a-b), 0.001 * fabs(a)+fabs(b));
      errs++;
   }
   a = 0.99; b = 1.0;
   if (close_enough(a, b) != 0) {
      fprintf(stderr, "  Fail: %f is not close enough to %f\n",
            (double) a, (double) b);
      errs++;
   }
   a = -0.999; b = 1.0;
   if (close_enough(a, b) != 0) {
      fprintf(stderr, "  Fail: %f is close enough to %f\n",
            (double) a, (double) b);
      errs++;
   }
   return errs;
}

static uint32_t test_build_orthogonal_matrix()
{
   uint32_t errs = 0;
   printf("Testing build_orthogonal_matrix\n");
   vector_type a = { .v = { 1.0, 0.0, 0.0 } };
   vector_type b = { .v = { 0.5, 0.5, 0.0 } };
   vector_type c = { .v = { 0.0, 0.5, 0.0 } };
   vector_type d = { .v = { 0.0, 0.5, 1.0 } };
   matrix_type m;
   build_orthogonal_matrix_xy(&a, &b, &m);
   if (  !CE(m.m[0], 1.0) || !CE(m.m[1], 0.0) || !CE(m.m[2], 0.0) ||
         !CE(m.m[3], 0.0) || !CE(m.m[4], 1.0) || !CE(m.m[5], 0.0) ||
         !CE(m.m[6], 0.0) || !CE(m.m[7], 0.0) || !CE(m.m[8], 1.0)) {
      fprintf(stderr, "  Failed XY. Expected identity matrix\n");
      print_mat(&m, "resultant matrix");
      errs++;
   }
   build_orthogonal_matrix_yx(&b, &a, &m);
   if (  !CE(m.m[0], 0.707) || !CE(m.m[1],-0.707) || !CE(m.m[2], 0.0) ||
         !CE(m.m[3], 0.707) || !CE(m.m[4], 0.707) || !CE(m.m[5], 0.0) ||
         !CE(m.m[6], 0.0) || !CE(m.m[7], 0.0) || !CE(m.m[8], 1.0)) {
      fprintf(stderr, "  Failed YX. Expected matrix rotated -pi/4 about Z\n");
      print_mat(&m, "resultant matrix");
      errs++;
   }
   build_orthogonal_matrix_yz(&c, &d, &m);
   if (  !CE(m.m[0], 1.0) || !CE(m.m[1], 0.0) || !CE(m.m[2], 0.0) ||
         !CE(m.m[3], 0.0) || !CE(m.m[4], 1.0) || !CE(m.m[5], 0.0) ||
         !CE(m.m[6], 0.0) || !CE(m.m[7], 0.0) || !CE(m.m[8], 1.0)) {
      fprintf(stderr, "  Failed YZ. Expected identity matrix\n");
      print_mat(&m, "resultant matrix");
      errs++;
   }
   return errs;
}


static uint32_t test_build_upright_matrix()
{
   uint32_t errs = 0;
   printf("Testing build_upright_matrix\n");
   // applying matrix to vector creating it should generate 0,1,0 vector
   // applying matrix to Y axis should generate vector with -x,-y versus 'up'
   vector_type up1 = { .v = { -0.3, 0.8, 0.5 } };
   vector_type up2 = { .v = { 0.3, 0.9, 0.0 } };
   vector_type up3 = { .v = { 0.0, 0.9, -0.2 } };
   unitify(&up1);
   unitify(&up2);
   unitify(&up3);
   matrix_type m;
   vector_type res;
   // make sure inverse of up vectors are rotated back to Y axis
   // up1
   build_upright_matrix(&up1, &m);
   up1.v[0] *= -1.0;
   up1.v[2] *= -1.0;
   mult_matrix_vector(&m, &up1, &res);
   if (!CA(res.v[0], 0.0) || !CA(res.v[1], 1.0) || !CA(res.v[2], 0.0)) {
      fprintf(stderr, "  Failed. Expected [0,1,0]\n");
      print_vec(&res, "actual result");
      print_vec(&up1, "'up' vector");
      errs++;
   }
   // up2
   build_upright_matrix(&up2, &m);
   up2.v[0] *= -1.0;
   up2.v[2] *= -1.0;
   mult_matrix_vector(&m, &up2, &res);
   if (!CA(res.v[0], 0.0) || !CA(res.v[1], 1.0) || !CA(res.v[2], 0.0)) {
      fprintf(stderr, "  Failed. Expected [0,1,0]\n");
      print_vec(&res, "actual result");
      print_vec(&up2, "'up' vector");
      errs++;
   }
   // up3
   build_upright_matrix(&up3, &m);
   up3.v[0] *= -1.0;
   up3.v[2] *= -1.0;
   mult_matrix_vector(&m, &up3, &res);
   if (!CA(res.v[0], 0.0) || !CA(res.v[1], 1.0) || !CA(res.v[2], 0.0)) {
      fprintf(stderr, "  Failed. Expected [0,1,0]\n");
      print_vec(&res, "actual result");
      print_vec(&up3, "'up' vector");
      errs++;
   }
   return errs;
}

static uint32_t test_vector_init()
{
   uint32_t errs = 0;
   printf("Testing vector_init\n");
   vector_type a;
   vector_init(&a, 1.0, 2.0, 3.0);
   if ((a.v[0] != 1.0) || (a.v[1] != 2.0) || (a.v[2] != 3.0)) {
      fprintf(stderr, "  Failed. Expected [1,2,3]\n");
      print_vec(&a, "actual result");
      errs++;
   }
   return errs;
}

static uint32_t test_matrix_init()
{
   uint32_t errs = 0;
   printf("Testing matrix_init\n");
   vector_type a, b, c;
   vector_init(&a, 1.0, 2.0, 3.0);
   vector_init(&b, 4.0, 5.0, 6.0);
   vector_init(&c, 7.0, 8.0, 9.0);
   matrix_type m;
   matrix_init(&m, &a, &b, &c);
   if (  (m.m[0] != 1.0) || (m.m[1] != 2.0) || (m.m[2] != 3.0) ||
         (m.m[3] != 4.0) || (m.m[4] != 5.0) || (m.m[5] != 6.0) ||
         (m.m[6] != 7.0) || (m.m[7] != 8.0) || (m.m[8] != 9.0)) {
      fprintf(stderr, "  Failed. Expected [[1,2,3][4,5,6][7,8,9]]\n");
      print_mat(&m, "actual result");
      errs++;
   }
   zero_matrix(&m);
   if (  (m.m[0] != 0.0) || (m.m[1] != 0.0) || (m.m[2] != 0.0) ||
         (m.m[3] != 0.0) || (m.m[4] != 0.0) || (m.m[5] != 0.0) ||
         (m.m[6] != 0.0) || (m.m[7] != 0.0) || (m.m[8] != 0.0)) {
      fprintf(stderr, "  Failed. Expected [[0,0,0][0,0,0][0,0,0]]\n");
      print_mat(&m, "actual result");
      errs++;
   }
   matrix_init_raw(&m, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
   if (  (m.m[0] != 1.0) || (m.m[1] != 2.0) || (m.m[2] != 3.0) ||
         (m.m[3] != 4.0) || (m.m[4] != 5.0) || (m.m[5] != 6.0) ||
         (m.m[6] != 7.0) || (m.m[7] != 8.0) || (m.m[8] != 9.0)) {
      fprintf(stderr, "  Failed. Expected [[1,2,3][4,5,6][7,8,9]]\n");
      print_mat(&m, "actual result");
      errs++;
   }
   identity_matrix(&m);
   if (  (m.m[0] != 1.0) || (m.m[1] != 0.0) || (m.m[2] != 0.0) ||
         (m.m[3] != 0.0) || (m.m[4] != 1.0) || (m.m[5] != 0.0) ||
         (m.m[6] != 0.0) || (m.m[7] != 0.0) || (m.m[8] != 1.0)) {
      fprintf(stderr, "  Failed. Expected [[0,0,0][0,0,0][0,0,0]]\n");
      print_mat(&m, "actual result");
      errs++;
   }
   return errs;
}


static uint32_t test_mult_vector_matrix(void)
{
   uint32_t errs = 0;
   printf("Testing mult_vector_matrix\n");
   vector_type vec = { .v = { 0.3, 0.4, 0.5 } };
   vector_type x = { .v = { 1.0,  2.0, 0.0 } };
   vector_type y = { .v = { 1.0, -0.5, 0.5 } };
   matrix_type m;
   build_orthogonal_matrix_xy(&x, &y, &m);
   vector_type res;
   // results calculated through octave
   mult_vector_matrix(&vec, &m, &res);
   if (!CE(res.v[0], 0.64334) || !CE(res.v[1], 0.01374) || 
         !CE(res.v[2], -0.29316)) {
      fprintf(stderr, "  Failed. Expected [0.643, 0.014, -0.293]\n");
      print_vec(&res, "actual result");
      errs++;
   }
   return errs;
}

static uint32_t test_mult_matrix_vector(void)
{
   uint32_t errs = 0;
   printf("Testing mult_matrix_vector\n");
   vector_type vec = { .v = { 0.3, 0.4, 0.5 } };
   vector_type x = { .v = { 1.0,  2.0, 0.0 } };
   vector_type y = { .v = { 1.0, -0.5, 0.5 } };
   matrix_type m;
   build_orthogonal_matrix_xy(&x, &y, &m);
   vector_type res;
   // results calculated through octave
   mult_matrix_vector(&m, &vec, &res);
   if (!CE(res.v[0], 0.49193) || !CE(res.v[1], 0.28577) || 
         !CE(res.v[2], -0.41992)) {
      fprintf(stderr, "  Failed. Expected [0.492, 0.286, -0.420]\n");
      print_vec(&res, "actual result");
      errs++;
   }
   return errs;
}

static uint32_t test_mult_matrix()
{
   uint32_t errs = 0;
   printf("Testing mult_matrix\n");
   vector_type a, b, c;
   vector_init(&a, 1.0, 2.0, 3.0);
   vector_init(&b, 2.0, 3.0,-1.0);
   vector_init(&c, 1.0,-1.0, 2.0);
   matrix_type m1, m2, m3;
   matrix_init(&m1, &a, &b, &c);
   matrix_init(&m2, &b, &c, &a);
   mult_matrix(&m1, &m2, &m3);
   // values taken from calculation in octave
   if (  (m3.m[0] != 7.0) || (m3.m[1] != 7.0) || (m3.m[2] != 12.0) ||
         (m3.m[3] != 6.0) || (m3.m[4] != 1.0) || (m3.m[5] !=  1.0) ||
         (m3.m[6] != 3.0) || (m3.m[7] != 8.0) || (m3.m[8] !=  3.0)) {
      fprintf(stderr, "  Failed\n");
      print_mat(&m1, "matrix A");
      print_mat(&m2, "matrix B");
      print_mat(&m3, "A * B");
      errs++;
   }
   square_matrix(&m1, &m3);
   // values taken from calculation in octave
   if (  (m3.m[0] != 8.0) || (m3.m[1] !=  5.0) || (m3.m[2] != 7.0) ||
         (m3.m[3] != 7.0) || (m3.m[4] != 14.0) || (m3.m[5] != 1.0) ||
         (m3.m[6] != 1.0) || (m3.m[7] != -3.0) || (m3.m[8] != 8.0)) {
      fprintf(stderr, "  Failed\n");
      print_mat(&m1, "matrix A");
      print_mat(&m3, "A * A");
      errs++;
   }
   return errs;
}


//static uint32_t test_boilerplate()
//{
//   uint32_t errs = 0;
//   printf("Testing boilerplate\n");
//   if (1 == 0) {
//      fprintf(stderr, "  Failed\n");
//      errs++;
//   }
//   matrix_type m;
//   identity_matrix(&m);
//   vector_type *v1 = &m.row1;
//   vector_type *v2 = &m.row2;
//   vector_type *v3 = &m.row3;
//   if ((sizeof m) != (3 * sizeof *v1)) {
//      fprintf(stderr, "Matrix type must be clean multiple of vector type\n");
//      errs++;
//   }
//   if (((v1->v[0] != 1.0f) || (v1->v[1] != 0.0f) || (v1->v[2] != 0.0f)) ||
//         ((v2->v[0] != 0.0f) || (v2->v[1] != 1.0f) || (v2->v[2] != 0.0f)) ||
//         ((v3->v[0] != 0.0f) || (v3->v[1] != 0.0f) || (v3->v[2] != 1.0f))) {
//      fprintf(stderr, "Matrix doesn't decompose cleanly to subvectors\n");
//      errs++;
//   }
//   return errs;
//}


// check downsample
int main(int argc, char **argv)
{
   (void) argc;
   (void) argv;
   ////////
   uint32_t errs = 0;
   //errs += test_boilerplate();
   errs += test_close_enough();
   //
   errs += test_unitify();
   errs += test_vector_init();
   errs += test_matrix_init();
   //
   errs += test_mult_matrix();
   errs += test_mult_vector_matrix();
   errs += test_mult_matrix_vector();
   errs += test_build_orthogonal_matrix();
   errs += test_build_upright_matrix();
   //
   errs += test_measure_rotation();
//   errs += test_get_pitch();
//   errs += test_get_roll();
//   errs += test_get_heading();
//   errs += test_get_north_heading();
   //
   errs += test_add_weighted_vector();
   errs += test_rotate_vector_about_axis();
   //
   if (errs == 0) {
      printf("--------------------\n");
      printf("--  Tests passed  --\n");
      printf("--------------------\n");
   } else {
      printf("---------------------------------\n");
      printf("***  One or more tests failed ***\n");
   }
   return (int) errs;
}

#endif // LIN_ALG_TEST

