//
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"
#include "fcl/collision.h"
#include "fcl/distance.h"
#include "fcl/BV/BV.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"

#include <errno.h>
#include <sys/time.h>
#include <unistd.h>

template<typename BV>
bool collide_test(const fcl::Transform3f& tf,
                  const std::vector<fcl::Vec3f>& vertices1, const std::vector<fcl::Triangle>& triangles1,
                  const std::vector<fcl::Vec3f>& vertices2, const std::vector<fcl::Triangle>& triangles2,
                  fcl::SplitMethodType split_method, bool verbose = true);
template<typename BV>
double distance_test(const fcl::Transform3f& tf,
                     const std::vector<fcl::Vec3f>& vertices1, const std::vector<fcl::Triangle>& triangles1,
                     const std::vector<fcl::Vec3f>& vertices2, const std::vector<fcl::Triangle>& triangles2,
                     double *n1, double *n2, fcl::SplitMethodType split_method, bool verbose = true);

int num_max_contacts = std::numeric_limits<int>::max();
bool enable_contact = true;

std::vector<fcl::Contact> global_pairs;
std::vector<fcl::Contact> global_pairs_now;

double collideCheck (std::vector<fcl::Vec3f> &p1, std::vector<fcl::Vec3f> &p2,
                     std::vector<fcl::Triangle> &t1, std::vector<fcl::Triangle> &t2,
                     double *pos, double *rot, double *n1, double *n2,
                     long distance, long objtype, long splitmethod, long verbose) {

  fcl::Vec3f trs(pos[0], pos[1], pos[2]);
  fcl::Quaternion3f qu(rot[0], rot[1], rot[2], rot[3]);
  fcl::Transform3f in_transform (qu, trs);

  bool vb = (verbose !=0) ? true : false;

  global_pairs.clear();
  global_pairs_now.clear();

  if (vb) {
    std::cout << "(make-coords :pos (float-vector " << (in_transform.getTranslation())[0];
    std::cout << " " << (in_transform.getTranslation())[1];
    std::cout << " " << (in_transform.getTranslation())[2] << ") ";
    std::cout << ":rot (quaternion2matrix (float-vector " << in_transform.getQuatRotation().getW();
    std::cout << " " << in_transform.getQuatRotation().getX();
    std::cout << " " << in_transform.getQuatRotation().getY();
    std::cout << " " << in_transform.getQuatRotation().getZ()  << ")))" << std::endl;
  }

  fcl::SplitMethodType stype;
  switch(splitmethod) {
  case 0:
    if (vb) std::cout << "use fcl::SPLIT_METHOD_BV_CENTER" << std::endl;
    stype = fcl::SPLIT_METHOD_BV_CENTER;
    break;
  case 1:
    if (vb) std::cout << "use fcl::SPLIT_METHOD_MEDIAN" << std::endl;
    stype = fcl::SPLIT_METHOD_MEDIAN;
    break;
  case 2:
    if (vb) std::cout << "use fcl::SPLIT_METHOD_MEAN" << std::endl;
    stype = fcl::SPLIT_METHOD_MEAN;
    break;
  default:
    if (vb) std::cout << "use fcl::SPLIT_METHOD_BV_CENTER" << std::endl;
    stype = fcl::SPLIT_METHOD_BV_CENTER;
    break;
  }

  if ( distance == 0 ) {
    //AABB, OBB, RSS, kIOS, OBBRSS, KDOP16, KDOP18, kDOP24
    bool r;
    switch(objtype) {
    case 0:
      if (vb) std::cout << "use fcl::AABB" << std::endl;
      r = collide_test<fcl::AABB>(in_transform, p1, t1, p2, t2, stype, vb);
      break;
    case 1:
      if (vb) std::cout << "use fcl::OBB" << std::endl;
      r = collide_test<fcl::OBB>(in_transform, p1, t1, p2, t2, stype, vb);
      break;
    case 2:
      if (vb) std::cout << "use fcl::RSS" << std::endl;
      r = collide_test<fcl::RSS>(in_transform, p1, t1, p2, t2, stype, vb);
      break;
    case 3:
      if (vb) std::cout << "use fcl::kIOS" << std::endl;
      r = collide_test<fcl::kIOS>(in_transform, p1, t1, p2, t2, stype, vb);
      break;
    case 4:
      if (vb) std::cout << "use fcl::OBBRSS" << std::endl;
      r = collide_test<fcl::OBBRSS>(in_transform, p1, t1, p2, t2, stype, vb);
      break;
    case 5:
      if (vb) std::cout << "use fcl::KDOP16" << std::endl;
      r = collide_test<fcl::KDOP<16> >(in_transform, p1, t1, p2, t2, stype, vb);
      break;
    case 6:
      if (vb) std::cout << "use fcl::KDOP18" << std::endl;
      r = collide_test<fcl::KDOP<18> >(in_transform, p1, t1, p2, t2, stype, vb);
      break;
    case 7:
      if (vb) std::cout << "use fcl::KDOP24" << std::endl;
      r = collide_test<fcl::KDOP<24> >(in_transform, p1, t1, p2, t2, stype, vb);
      break;
    default:
      if (vb) std::cout << "use fcl::OBBRSS" << std::endl;
      r = collide_test<fcl::OBBRSS>(in_transform, p1, t1, p2, t2, stype, vb);
      break;
    }
    return (r ? 1.0 : 0.0);
  } else {
    switch(objtype) {
    case 0:
      if (vb) std::cout << "use fcl::AABB" << std::endl;
      return distance_test<fcl::AABB>(in_transform, p1, t1, p2, t2, n1, n2, stype, vb);
      break;
    case 1:
      if (vb) std::cout << "use fcl::OBB" << std::endl;
      return distance_test<fcl::OBB>(in_transform, p1, t1, p2, t2, n1, n2, stype, vb);
      break;
    case 2:
      if (vb) std::cout << "use fcl::RSS" << std::endl;
      return distance_test<fcl::RSS>(in_transform, p1, t1, p2, t2, n1, n2, stype, vb);
      break;
    case 3:
      if (vb) std::cout << "use fcl::kIOS" << std::endl;
      return distance_test<fcl::kIOS>(in_transform, p1, t1, p2, t2, n1, n2, stype, vb);
      break;
    case 4:
      if (vb) std::cout << "use fcl::OBBRSS" << std::endl;
      return distance_test<fcl::OBBRSS>(in_transform, p1, t1, p2, t2, n1, n2, stype, vb);
      break;
    case 5:
      if (vb) std::cout << "use fcl::KDOP16" << std::endl;
      return distance_test<fcl::KDOP<16> >(in_transform, p1, t1, p2, t2, n1, n2, stype, vb);
      break;
    case 6:
      if (vb) std::cout << "use fcl::KDOP18" << std::endl;
      return distance_test<fcl::KDOP<18> >(in_transform, p1, t1, p2, t2, n1, n2, stype, vb);
      break;
    case 7:
      if (vb) std::cout << "use fcl::KDOP24" << std::endl;
      return distance_test<fcl::KDOP<24> >(in_transform, p1, t1, p2, t2, n1, n2, stype, vb);
      break;
    }
  }
  if (vb) std::cout << "use fcl::OBBRSS" << std::endl;
  return distance_test<fcl::OBBRSS>(in_transform, p1, t1, p2, t2, n1, n2, stype, vb);
}

template<typename BV>
bool collide_test(const fcl::Transform3f& tf,
                  const std::vector<fcl::Vec3f>& vertices1, const std::vector<fcl::Triangle>& triangles1,
                  const std::vector<fcl::Vec3f>& vertices2, const std::vector<fcl::Triangle>& triangles2,
                     fcl::SplitMethodType split_method, bool verbose)
{
  fcl::BVHModel<BV> *m1 = new fcl::BVHModel<BV>();
  fcl::BVHModel<BV> *m2 = new fcl::BVHModel<BV>();
  m1->bv_splitter.reset(new fcl::BVSplitter<BV>(split_method));
  m2->bv_splitter.reset(new fcl::BVSplitter<BV>(split_method));

  m1->beginModel();
  m1->addSubModel(vertices1, triangles1);
  m1->endModel();

  m2->beginModel();
  m2->addSubModel(vertices2, triangles2);
  m2->endModel();

  fcl::Transform3f pose1(tf), pose2;
  //fcl::CollisionGeometry *g1 = (fcl::CollisionGeometry *)m1;
  //fcl::CollisionGeometry *g2 = (fcl::CollisionGeometry *)m2;
  //fcl::CollisionObject *o1, *o2;
  //o1 = new fcl::CollisionObject(g1, pose1);
  //o1 = new fcl::CollisionObject(g2, pose2);

  fcl::CollisionRequest request(num_max_contacts, enable_contact);
  // result will be returned via the collision result structure
  fcl::CollisionResult local_result;
  // perform collision test
  //fcl::collide(o1, o2, request, local_result);
  fcl::collide(m1, pose1, m2, pose2, request, local_result);

#if 0
  fcl::CollisionResult local_result;
  fcl::MeshCollisionTraversalNode<BV> node;
  if(!fcl::initialize<BV>(node, m1, pose1, m2, pose2,
                          fcl::CollisionRequest(num_max_contacts, enable_contact), local_result))
    std::cout << "initialize error" << std::endl;
  node.enable_statistics = verbose;
  fcl::collide(&node);
#endif

  if(local_result.numContacts() > 0) {
#if 0
    if(global_pairs.size() == 0) {
      local_result.getContacts(global_pairs);
      std::sort(global_pairs.begin(), global_pairs.end());
    } else {
      local_result.getContacts(global_pairs_now);
      std::sort(global_pairs_now.begin(), global_pairs_now.end());
    }
#endif
    if(verbose) {
      std::cout << "in collision : " << local_result.numContacts() << std::endl;
    }
    return true;
  } else {
    if(verbose) {
      std::cout << "collision free " << std::endl;
    }
    return false;
  }
}

template<typename BV>
double distance_test(const fcl::Transform3f& tf,
                     const std::vector<fcl::Vec3f>& vertices1, const std::vector<fcl::Triangle>& triangles1,
                     const std::vector<fcl::Vec3f>& vertices2, const std::vector<fcl::Triangle>& triangles2,
                     double *n1, double *n2, fcl::SplitMethodType split_method, bool verbose)
{
  fcl::BVHModel<BV> *m1 = new fcl::BVHModel<BV>();
  fcl::BVHModel<BV> *m2 = new fcl::BVHModel<BV>();
  m1->bv_splitter.reset(new fcl::BVSplitter<BV>(split_method));
  m2->bv_splitter.reset(new fcl::BVSplitter<BV>(split_method));

  m1->beginModel();
  m1->addSubModel(vertices1, triangles1);
  m1->endModel();

  m2->beginModel();
  m2->addSubModel(vertices2, triangles2);
  m2->endModel();

  fcl::Transform3f pose1(tf), pose2;
  //fcl::CollisionGeometry *g1 = (fcl::CollisionGeometry *)m1;
  //fcl::CollisionGeometry *g2 = (fcl::CollisionGeometry *)m2;
  //fcl::CollisionObject *o1, *o2;
  //o1 = new fcl::CollisionObject(g1, pose1);
  //o1 = new fcl::CollisionObject(g2, pose2);
  bool enable_nearest_points = true;
  //double rel_error = 0.0, abs_error = 0.0;

  //fcl::DistanceRequest request(enable_nearest_points, rel_error, abs_error);
  fcl::DistanceRequest request(enable_nearest_points);

  // result will be returned via the collision result structure
  fcl::DistanceResult local_result;
  // perform collision test
  //fcl::collide(o1, o2, request, local_result);

  struct timeval tv_s, tv_e;
  gettimeofday(&tv_s, NULL);
  fcl::distance(m1, pose1, m2, pose2, request, local_result);
  gettimeofday(&tv_e, NULL);
  double msec = (tv_e.tv_sec - tv_s.tv_sec) * 1000.0 + (tv_e.tv_usec - tv_s.tv_usec) / 1000.0;

  n1[0] = local_result.nearest_points[0][0];
  n1[1] = local_result.nearest_points[0][1];
  n1[2] = local_result.nearest_points[0][2];
  n2[0] = local_result.nearest_points[1][0];
  n2[1] = local_result.nearest_points[1][1];
  n2[2] = local_result.nearest_points[1][2];

  if (verbose) {
    std::cout << "time: " << msec << "milli second" std::endl;
    std::cout << "#f(" << local_result.nearest_points[0][0] << " ";
    std::cout << local_result.nearest_points[0][1] << " ";
    std::cout << local_result.nearest_points[0][2] << ")";
    std::cout << " #f(" << local_result.nearest_points[1][0] << " ";
    std::cout << local_result.nearest_points[1][1] << " ";
    std::cout << local_result.nearest_points[1][2] << ")" << std::endl;
  }

  return local_result.min_distance;
}

extern "C" {
  double eus_col_check(double *vertices1, long vert_len1, long *indices1, long ind_len1,
                       double *vertices2, long vert_len2, long *indices2, long ind_len2,
                       double *pos, double *rot, double *n1, double *n2,
                       long distance, long objtype, long splitmethod, long verbose) {

    // length of vertices should be (3 * vert_len).
    // length of indices shoudl be (3 * ind_len).

    std::vector<fcl::Vec3f> p1, p2;
    std::vector<fcl::Triangle> t1, t2;

    for(long i = 0; i < vert_len1; i++) {
      fcl::Vec3f p(vertices1[3*i + 0], vertices1[3*i + 1], vertices1[3*i + 2]);
      p1.push_back(p);
    }
    for(long i = 0; i < ind_len1; i++) {
      fcl::Triangle tri;
      tri[0] = indices1[3*i + 0];
      tri[1] = indices1[3*i + 1];
      tri[2] = indices1[3*i + 2];
      t1.push_back(tri);
    }
    for(long i = 0; i < vert_len2; i++) {
      fcl::Vec3f p(vertices2[3*i + 0], vertices2[3*i + 1], vertices2[3*i + 2]);
      p2.push_back(p);
    }
    for(long i = 0; i < ind_len2; i++) {
      fcl::Triangle tri;
      tri[0] = indices2[3*i + 0];
      tri[1] = indices2[3*i + 1];
      tri[2] = indices2[3*i + 2];
      t2.push_back(tri);
    }

    return collideCheck (p1, p2, t1, t2, pos, rot, n1, n2,
                         distance, objtype, splitmethod, verbose);
  }
}
