//
#include "fcl/traversal/traversal_node_bvhs.h"
#include "fcl/traversal/traversal_node_setup.h"
#include "fcl/collision_node.h"
#include "fcl/collision.h"
#include "fcl/BV/BV.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"

fcl::FCL_REAL rand_interval(fcl::FCL_REAL rmin, fcl::FCL_REAL rmax)
{
  fcl::FCL_REAL t = rand() / ((fcl::FCL_REAL)RAND_MAX + 1);
  return (t * (rmax - rmin) + rmin);
}

void eulerToMatrix(fcl::FCL_REAL a, fcl::FCL_REAL b, fcl::FCL_REAL c, fcl::Matrix3f& R)
{
  fcl::FCL_REAL c1 = cos(a);
  fcl::FCL_REAL c2 = cos(b);
  fcl::FCL_REAL c3 = cos(c);
  fcl::FCL_REAL s1 = sin(a);
  fcl::FCL_REAL s2 = sin(b);
  fcl::FCL_REAL s3 = sin(c);

  R.setValue(c1 * c2, - c2 * s1, s2,
             c3 * s1 + c1 * s2 * s3, c1 * c3 - s1 * s2 * s3, - c2 * s3,
             s1 * s3 - c1 * c3 * s2, c3 * s1 * s2 + c1 * s3, c2 * c3);
}

void generateRandomTransforms(fcl::FCL_REAL extents[6], std::vector<fcl::Transform3f>& transforms, std::size_t n)
{
  transforms.resize(n);
  for(std::size_t i = 0; i < n; ++i)
  {
    fcl::FCL_REAL x = rand_interval(extents[0], extents[3]);
    fcl::FCL_REAL y = rand_interval(extents[1], extents[4]);
    fcl::FCL_REAL z = rand_interval(extents[2], extents[5]);

    const fcl::FCL_REAL pi = 3.1415926;
    fcl::FCL_REAL a = rand_interval(0, 2 * pi);
    fcl::FCL_REAL b = rand_interval(0, 2 * pi);
    fcl::FCL_REAL c = rand_interval(0, 2 * pi);

    {
      fcl::Matrix3f R;
      eulerToMatrix(a, b, c, R);
      fcl::Vec3f T(x, y, z);
      transforms[i].setTransform(R, T);
    }
  }
}
#if 1
void loadOBJFile(const char* filename, std::vector<fcl::Vec3f>& points, std::vector<fcl::Triangle>& triangles)
{

  FILE* file = fopen(filename, "rb");
  if(!file)
  {
    std::cerr << "file not exist" << std::endl;
    return;
  }

  bool has_normal = false;
  bool has_texture = false;
  char line_buffer[2000];
  while(fgets(line_buffer, 2000, file))
  {
    char* first_token = strtok(line_buffer, "\r\n\t ");
    if(!first_token || first_token[0] == '#' || first_token[0] == 0)
      continue;

    switch(first_token[0])
    {
    case 'v':
      {
        if(first_token[1] == 'n')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_normal = true;
        }
        else if(first_token[1] == 't')
        {
          strtok(NULL, "\t ");
          strtok(NULL, "\t ");
          has_texture = true;
        }
        else
        {
          fcl::FCL_REAL x = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
          fcl::FCL_REAL y = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
          fcl::FCL_REAL z = (fcl::FCL_REAL)atof(strtok(NULL, "\t "));
          fcl::Vec3f p(x, y, z);
          points.push_back(p);
        }
      }
      break;
    case 'f':
      {
        fcl::Triangle tri;
        char* data[30];
        int n = 0;
        while((data[n] = strtok(NULL, "\t \r\n")) != NULL)
        {
          if(strlen(data[n]))
            n++;
        }

        for(int t = 0; t < (n - 2); ++t)
        {
          if((!has_texture) && (!has_normal))
          {
            tri[0] = atoi(data[0]) - 1;
            tri[1] = atoi(data[1]) - 1;
            tri[2] = atoi(data[2]) - 1;
          }
          else
          {
            const char *v1;
            for(int i = 0; i < 3; i++)
            {
              // vertex ID
              if(i == 0)
                v1 = data[0];
              else
                v1 = data[t + i];

              tri[i] = atoi(v1) - 1;
            }
          }
          triangles.push_back(tri);
        }
      }
    }
  }
}
#endif

template<typename BV>
bool collide_test(const fcl::Transform3f& tf,
                  const std::vector<fcl::Vec3f>& vertices1, const std::vector<fcl::Triangle>& triangles1,
                  const std::vector<fcl::Vec3f>& vertices2, const std::vector<fcl::Triangle>& triangles2,
                  fcl::SplitMethodType split_method, bool verbose = true);
template<typename BV>
bool distance_test(const fcl::Transform3f& tf,
                   const std::vector<fcl::Vec3f>& vertices1, const std::vector<fcl::Triangle>& triangles1,
                   const std::vector<fcl::Vec3f>& vertices2, const std::vector<fcl::Triangle>& triangles2,
                   fcl::SplitMethodType split_method, bool verbose = true);

int num_max_contacts = std::numeric_limits<int>::max();
bool enable_contact = true;

std::vector<fcl::Contact> global_pairs;
std::vector<fcl::Contact> global_pairs_now;

int collideCheck (std::vector<fcl::Vec3f> &p1, std::vector<fcl::Vec3f> &p2,
                  std::vector<fcl::Triangle> &t1, std::vector<fcl::Triangle> &t2,
                  double *pos, double *rot) {

  fcl::Vec3f trs(pos[0], pos[1], pos[2]);
  fcl::Quaternion3f qu(rot[0], rot[1], rot[2], rot[3]);
  fcl::Transform3f in_transform (qu, trs);

  bool verbose = false;

  global_pairs.clear();
  global_pairs_now.clear();

  if (verbose) {
    std::cout << "(make-coords :pos (float-vector " << (in_transform.getTranslation())[0];
    std::cout << " " << (in_transform.getTranslation())[1];
    std::cout << " " << (in_transform.getTranslation())[2] << ") ";
    std::cout << ":rot (quaternion2matrix (float-vector " << in_transform.getQuatRotation().getW();
    std::cout << " " << in_transform.getQuatRotation().getX();
    std::cout << " " << in_transform.getQuatRotation().getY();
    std::cout << " " << in_transform.getQuatRotation().getZ()  << ")))" << std::endl;
  }

  return
    collide_test<fcl::OBBRSS>(in_transform, p1, t1, p2, t2,
                              fcl::SPLIT_METHOD_BV_CENTER, verbose);
}

template<typename BV>
bool collide_test(const fcl::Transform3f& tf,
                  const std::vector<fcl::Vec3f>& vertices1, const std::vector<fcl::Triangle>& triangles1,
                  const std::vector<fcl::Vec3f>& vertices2, const std::vector<fcl::Triangle>& triangles2,
                  fcl::SplitMethodType split_method, bool verbose)
{
  fcl::BVHModel<BV> m1;
  fcl::BVHModel<BV> m2;
  m1.bv_splitter.reset(new fcl::BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new fcl::BVSplitter<BV>(split_method));

  m1.beginModel();
  m1.addSubModel(vertices1, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  fcl::Transform3f pose1(tf), pose2;

  fcl::CollisionResult local_result;
  fcl::MeshCollisionTraversalNode<BV> node;

  if(!fcl::initialize<BV>(node, m1, pose1, m2, pose2,
                          fcl::CollisionRequest(num_max_contacts, enable_contact), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  fcl::collide(&node);

  if(local_result.numContacts() > 0) {
    if(global_pairs.size() == 0) {
      local_result.getContacts(global_pairs);
      std::sort(global_pairs.begin(), global_pairs.end());
    } else {
      local_result.getContacts(global_pairs_now);
      std::sort(global_pairs_now.begin(), global_pairs_now.end());
    }
    if(verbose) {
      std::cout << "in collision " << local_result.numContacts() << ": " << std::endl;
      std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    }
    return true;
  } else {
    if(verbose) {
      std::cout << "collision free " << std::endl;
      std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    }
    return false;
  }
}

template<typename BV>
bool distance_Test(const fcl::Transform3f& tf,
                   const std::vector<fcl::Vec3f>& vertices1, const std::vector<fcl::Triangle>& triangles1,
                   const std::vector<fcl::Vec3f>& vertices2, const std::vector<fcl::Triangle>& triangles2, fcl::SplitMethodType split_method, bool verbose)
{
  fcl::BVHModel<BV> m1;
  fcl::BVHModel<BV> m2;
  m1.bv_splitter.reset(new fcl::BVSplitter<BV>(split_method));
  m2.bv_splitter.reset(new fcl::BVSplitter<BV>(split_method));

  // transform vertices
  std::vector<fcl::Vec3f> vertices1_new(vertices1.size());
  for(unsigned int i = 0; i < vertices1_new.size(); ++i) {
    vertices1_new[i] = tf.transform(vertices1[i]);
  }

  m1.beginModel();
  m1.addSubModel(vertices1_new, triangles1);
  m1.endModel();

  m2.beginModel();
  m2.addSubModel(vertices2, triangles2);
  m2.endModel();

  fcl::Transform3f pose1, pose2;

  fcl::CollisionResult local_result;
  fcl::MeshCollisionTraversalNode<BV> node;

  if(!fcl::initialize<BV>(node, m1, pose1, m2, pose2,
                          fcl::CollisionRequest(num_max_contacts, enable_contact), local_result))
    std::cout << "initialize error" << std::endl;

  node.enable_statistics = verbose;

  fcl::collide(&node);

  if(local_result.numContacts() > 0) {
    if(global_pairs.size() == 0) {
      local_result.getContacts(global_pairs);
      std::sort(global_pairs.begin(), global_pairs.end());
    } else {
      local_result.getContacts(global_pairs_now);
      std::sort(global_pairs_now.begin(), global_pairs_now.end());
    }
    if(verbose) {
      std::cout << "in collision " << local_result.numContacts() << ": " << std::endl;
      std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    }
    return true;
  } else {
    if(verbose) {
      std::cout << "collision free " << std::endl;
      std::cout << node.num_bv_tests << " " << node.num_leaf_tests << std::endl;
    }
    return false;
  }
}

extern "C" {
  bool eus_col_check(double *vertices1, long vert_len1, long *indices1, long ind_len1,
                     double *vertices2, long vert_len2, long *indices2, long ind_len2,
                     double *pos, double *rot) {
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

    return  collideCheck (p1, p2, t1, t2, pos, rot);
  }
}

#if 0
    collide_Test<fcl::OBB>(transforms[i], p1, t1, p2, t2,
                          fcl::SPLIT_METHOD_MEDIAN, verbose);
    collide_Test<fcl::OBB>(transforms[i], p1, t1, p2, t2,
                          fcl::SPLIT_METHOD_BV_CENTER, verbose);
    collide_Test<fcl::OBB>(transforms[i], p1, t1, p2, t2,
                           fcl::SPLIT_METHOD_MEAN, verbose);

    collide_Test2<fcl::OBB>(transforms[i], p1, t1, p2, t2,
                           fcl::SPLIT_METHOD_MEDIAN, verbose);
    collide_Test2<fcl::OBB>(transforms[i], p1, t1, p2, t2,
                           fcl::SPLIT_METHOD_BV_CENTER, verbose);
    collide_Test2<fcl::OBB>(transforms[i], p1, t1, p2, t2,
                               fcl::SPLIT_METHOD_MEAN, verbose);
#endif
