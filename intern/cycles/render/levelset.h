/*
 * Copyright 2011-2013 Blender Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __LEVELSET_H__
#define __LEVELSET_H__

#include <openvdb/openvdb.h>
#include <openvdb/io/File.h>
#include <openvdb/tools/RayIntersector.h>

#include "kernel_types.h"
#include "util_map.h"
#include "util_thread.h"

#include <functional>

CCL_NAMESPACE_BEGIN

class Device;
class DeviceScene;
class Progress;
class Scene;
class LevelSet;

void OpenVDB_initialize();
void OpenVDB_file_info(const char* filename);
LevelSet* OpenVDB_file_read(const char* filename, Scene* scene);
void OpenVDB_file_read_to_levelset(const char* filename, Scene* scene, LevelSet* levelset, int shader );
void OpenVDB_use_level_mesh(Scene* scene);

#if defined(CYCLES_TR1_UNORDERED_MAP) || defined(CYCLES_STD_UNORDERED_MAP) || defined(CYCLES_STD_UNORDERED_MAP_IN_TR1_NAMESPACE)
struct pthread_hash {
  size_t operator() (const pthread_t& val){
    /* not really sure how to hash a pthread_t, since it could be implemented as a struct */
    size_t res;
    memcpy( &res, &val, sizeof(size_t)>sizeof(pthread_t)?sizeof(pthread_t):sizeof(size_t));
    return res;
  };
};

struct pthread_equal_to : std::binary_function <pthread_t,pthread_t,bool> {
  bool operator() (const pthread_t& x, const pthread_t& y) const {return pthread_equal(x, y);}
};
#endif

// If this value is too low, nasty artifacts appear
// Too high, and render times are adversly affected
#define OPENVDB_LEVELSET_SEARCH_ITERATIONS 1

class LevelSet {
public:
       LevelSet( );
       LevelSet(openvdb::FloatGrid::Ptr gridPtr, int shader_);
       LevelSet(const LevelSet& levelset);
       ~LevelSet();

       void tag_update(Scene *scene);

       void initialize(openvdb::FloatGrid::Ptr& gridPtr, int shader_);             
       bool intersect(const Ray* ray, Intersection *isect);

	   openvdb::FloatGrid::Ptr grid;
       int shader;

	   typedef openvdb::tools::LinearSearchImpl<openvdb::FloatGrid, OPENVDB_LEVELSET_SEARCH_ITERATIONS, float> LinearSearchImpl;
	   typedef openvdb::math::Ray<float> vdb_ray_t;
       typedef openvdb::tools::LevelSetRayIntersector<openvdb::FloatGrid,
	                                                  LinearSearchImpl,
	                                                  openvdb::FloatTree::RootNodeType::ChildNodeType::LEVEL,
	                                                  vdb_ray_t> isect_t;

   /* used to ensure each thread gets its own intersector */
   /* if we really have true unordered_maps, use more appropriate hash and equality operators */
#if defined(CYCLES_TR1_UNORDERED_MAP) || defined(CYCLES_STD_UNORDERED_MAP) || defined(CYCLES_STD_UNORDERED_MAP_IN_TR1_NAMESPACE)
           typedef unordered_map<pthread_t, isect_t *,
				 pthread_hash, pthread_equal_to > isect_map_t;
#else
           typedef unordered_map<pthread_t, isect_t * > isect_map_t;
#endif
	   isect_map_t isect_map;
};

class LevelSetManager {
public:
       bool need_update;

       LevelSetManager();
       ~LevelSetManager();

       void device_update(Device *device, DeviceScene *dscene, Scene *scene, Progress& progress);
       void device_free(Device *device, DeviceScene *dscene);

       void tag_update(Scene *scene);
};

CCL_NAMESPACE_END

#endif /* __LEVELSET_H__ */
