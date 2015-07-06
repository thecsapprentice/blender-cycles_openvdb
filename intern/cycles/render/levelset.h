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

CCL_NAMESPACE_BEGIN

class Device;
class DeviceScene;
class Progress;
class Scene;

void OpenVDB_initialize();
void OpenVDB_file_info(const char* filename);
void OpenVDB_file_read(const char* filename, Scene* scene);
void OpenVDB_use_level_mesh(Scene* scene);

class LevelSet {
public:
       LevelSet(openvdb::FloatGrid::Ptr gridPtr, int shader_);
       ~LevelSet();

       void tag_update(Scene *scene);

       bool intersect(const Ray* ray, Intersection *isect);

	   openvdb::FloatGrid::Ptr grid;
       int shader;

	   typedef openvdb::tools::LinearSearchImpl<openvdb::FloatGrid, 0, float> LinearSearchImpl;
	   typedef openvdb::math::Ray<float> vdb_ray_t;
       typedef openvdb::tools::LevelSetRayIntersector<openvdb::FloatGrid,
	                                                  LinearSearchImpl,
	                                                  openvdb::FloatTree::RootNodeType::ChildNodeType::LEVEL,
	                                                  vdb_ray_t> isect_t;

	   /* used to ensure each thread gets its own intersector */
	   typedef unordered_map<pthread_t, isect_t *> isect_map_t;
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
