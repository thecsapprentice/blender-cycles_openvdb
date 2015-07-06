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

#include "levelset.h"
#include "scene.h"
#include "util_progress.h"
#include <stdio.h>

openvdb::FloatGrid::Ptr last_level_set;

CCL_NAMESPACE_BEGIN

void OpenVDB_initialize()
{
	openvdb::initialize();
}

void OpenVDB_file_info(const char* filename)
{
	using namespace openvdb;

	OpenVDB_initialize();

	io::File file(filename);
	file.open();

	size_t size = file.getSize();

	printf("Opening %s, is %lu bytes!\n", filename, size);

	for(io::File::NameIterator iter = file.beginName();
	    iter != file.endName();
	    ++iter)
	{
		printf("Contains grid %s!\n", iter.gridName().c_str());
	}

	file.close();
}

openvdb::FloatGrid::Ptr loaded_level_set;

void OpenVDB_file_read(const char* filename, Scene* scene)
{
	using namespace openvdb;

	OpenVDB_initialize();

	if (!loaded_level_set.use_count()) {
		try {
			io::File file(filename);
			file.open();

			size_t size = file.getSize();
			printf("Opening %s, is %lu bytes!\n", filename, size);

			for (io::File::NameIterator iter = file.beginName(); iter != file.endName(); ++iter) {
				printf("Reading grid %s!\n", iter.gridName().c_str());

				GridBase::Ptr grid = file.readGrid(iter.gridName());
				grid->print();

				if(grid->getGridClass() != GRID_LEVEL_SET)
					continue;

				if (grid->isType<FloatGrid>())
					loaded_level_set = gridPtrCast<openvdb::FloatGrid>(grid);
				else
					printf("No FloatGrid, ignoring!\n");
			}

			file.close();
		}
		catch (const IoError &e) {
			std::cerr << e.what() << "\n";
		}
	}

	scene->level_sets.push_back(new LevelSet(loaded_level_set, 0));
}

void OpenVDB_use_level_mesh(Scene* scene)
{
	if (last_level_set.use_count()) {
		last_level_set->print();
		uint shader = 0;//scene->shader_manager->get_shader_id(scene->default_surface, NULL, false);
		printf("Used shader OVDB: %d\n", shader);
		scene->level_sets.push_back(new LevelSet(last_level_set, shader));
	}
}

LevelSet::LevelSet(openvdb::FloatGrid::Ptr gridPtr, int shader_)
    : grid(gridPtr), shader(shader_)
{
	grid->print();
}

LevelSet::~LevelSet()
{
	for(isect_map_t::iterator iter = isect_map.begin();
	    iter != isect_map.end();
	    ++iter)
	{
		delete iter->second;
	}
}

void LevelSet::tag_update(Scene *scene)
{
	scene->level_set_manager->need_update = true;
}

bool LevelSet::intersect(const Ray* ray, Intersection *isect)
{
	pthread_t thread = pthread_self();
	isect_map_t::iterator iter = isect_map.find(thread);
	isect_t *isector;

	if(iter == isect_map.end()) {
		isector = new isect_t(*grid);
		pair<pthread_t, isect_t *> isect(thread, isector);
		isect_map.insert(isect);
	}
	else
		isector = iter->second;

	vdb_ray_t::Vec3Type P(ray->P.x, ray->P.y, ray->P.z);
	vdb_ray_t::Vec3Type D(ray->D.x, ray->D.y, ray->D.z);
	D.normalize();

	vdb_ray_t vdbray(P, D, 1e-5f, ray->t);
	vdb_ray_t::Vec3Type pos, normal;
	float t;

	if(isector->intersectsWS(vdbray, pos, normal, t)) {
		isect->t = t;
		isect->u = isect->v = 1.0f;
		isect->type = PRIMITIVE_LEVEL_SET;
		isect->shad = shader;
		isect->norm = normalize(make_float3(normal.x(), normal.y(), normal.z()));
		isect->prim = 0;
		isect->object = 0;
		return true;
	}

	return false;
}

LevelSetManager::LevelSetManager()
{
	need_update = true;
}

LevelSetManager::~LevelSetManager()
{
}

void LevelSetManager::device_update(Device *device, DeviceScene *dscene, Scene *scene, Progress& progress)
{
	if(!need_update)
		return;

	device_free(device, dscene);

	progress.set_status("Updating Level Sets", "Copying Level Sets to device");

	dscene->data.tables.num_level_sets = scene->level_sets.size();
	if (scene->level_sets.size() > 0)
		dscene->data.tables.level_sets = (void*) scene->level_sets[0];

	if(progress.get_cancel()) return;

	need_update = false;
}

void LevelSetManager::device_free(Device */*device*/, DeviceScene */*dscene*/)
{
}

void LevelSetManager::tag_update(Scene */*scene*/)
{
	need_update = true;
}

CCL_NAMESPACE_END
