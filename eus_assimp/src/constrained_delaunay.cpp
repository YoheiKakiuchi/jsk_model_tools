// for Eus
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <setjmp.h>
#include <errno.h>

#include <list>
#include <vector>
#include <set>
#include <string>
#include <map>
#include <sstream>
#include <cstdio>
#include <iostream>
#include <cctype>
// End for Eus

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/version_macros.h>

#include <cassert>
#include <iostream>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Exact_predicates_tag                               Itag;
#if (CGAL_VERSION_MAJOR == 4) and (CGAL_VERSION_MINOR < 8)
typedef CGAL::Triangulation_vertex_base_2<K>                     Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K>           Fb;
typedef CGAL::Triangulation_data_structure_2<Vb,Fb>              TDS;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;
#elif (CGAL_VERSION_MAJOR >= 5) or ((CGAL_VERSION_MAJOR == 4) and (CGAL_VERSION_MINOR >= 8))
typedef CGAL::Constrained_Delaunay_triangulation_2<K, CGAL::Default, Itag> CDT;
#endif

typedef CDT::Point Point;
typedef CDT::Edge  Edge;
typedef CDT::Edge_iterator  Edge_iterator;
typedef CDT::Face_iterator  Face_iterator;

// for eus.h
#define class   eus_class
#define throw   eus_throw
#define export  eus_export
#define vector  eus_vector
#define string  eus_string
#define iostream eus_iostream
#define complex  eus_complex

#include "eus.h"
extern "C" {
  pointer ___constrained_delaunay(register context *ctx, int n, pointer *argv, pointer env);
  void register_constrained_delaunay(){
    char modname[] = "___constrained_delaunay";
    return add_module_initializer(modname, (pointer (*)())___constrained_delaunay);}
}

#undef class
#undef throw
#undef export
#undef vector
#undef string
#undef iostream
#undef complex
// End for eus.h

typedef struct {
  double x;
  double y;
  double z;
} eus_point;

pointer CONSTRAINED_DELAUNAY(register context *ctx,int n,pointer *argv)
{
  int num = intval( argv[0]->c.fvec.length );
  double *in_vec = argv[0]->c.fvec.fv;

  CDT cdt;

  for (int i = 0; i < num/4; i++) {
    int idx = i * 4;
    cdt.insert_constraint( Point(in_vec[idx + 0], in_vec[idx + 1]),
                           Point(in_vec[idx + 2], in_vec[idx + 3]) );
  }

  int sz = cdt.number_of_faces();
  //printf ("num_of_faces = %d\n", sz);
  pointer res_fvec = makefvector (sz*3*3);// face * vetices(3) * points(3)
  vpush(res_fvec);

  double* pfvec = res_fvec->c.fvec.fv;
  int cntr = 0;

  //  for (const Face_iterator& f : cdt.finite_face_handles() ) {
  for (Face_iterator f = cdt.finite_faces_begin();
       f != cdt.finite_faces_end(); f++) {
    int idx = cntr * 9;
    Point a = f->vertex(0)->point();
    Point b = f->vertex(1)->point();
    Point c = f->vertex(2)->point();

    pfvec[idx + 0] = a[0];
    pfvec[idx + 1] = a[1];
    pfvec[idx + 2] = 0.0;

    pfvec[idx + 3] = b[0];
    pfvec[idx + 4] = b[1];
    pfvec[idx + 5] = 0.0;

    pfvec[idx + 6] = c[0];
    pfvec[idx + 7] = c[1];
    pfvec[idx + 8] = 0.0;

    cntr++;
  }

  return vpop();
}

pointer ___constrained_delaunay(register context *ctx, int n, pointer *argv, pointer env)
{
  defun(ctx,"C-CONSTRAINED-DELAUNAY-TRIANGULATION", argv[0], (pointer (*)())CONSTRAINED_DELAUNAY, NULL);

  return 0;
}
