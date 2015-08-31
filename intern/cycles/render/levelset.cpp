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
#include "util_task.h"
#include <stdio.h>
#include "util_foreach.h"

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

LevelSet* OpenVDB_file_read(const char* filename, Scene* scene)
{
	using namespace openvdb;

	OpenVDB_initialize();
	//openvdb::FloatGrid::Ptr level_set_ptr;
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

	return new LevelSet(loaded_level_set, 0);

	//if( level_set_ptr.use_count() == 0 )
	//  return NULL;
	//else
	//  return new LevelSet(level_set_ptr, 0);
}

void OpenVDB_file_read_to_levelset(const char* filename, Scene* scene, LevelSet* levelset, int shader )
{
	using namespace openvdb;
	OpenVDB_initialize();
	openvdb::FloatGrid::Ptr level_set_ptr;
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
	      level_set_ptr = gridPtrCast<openvdb::FloatGrid>(grid);
	    else
	      printf("No FloatGrid, ignoring!\n");
	  }
	  
	  file.close();
	}
	catch (const IoError &e) {
	  std::cerr << e.what() << "\n";
	}

	
	
	levelset->initialize( level_set_ptr, shader );
}



void OpenVDB_use_level_mesh(Scene* scene)
{
  /* We don't use any of this code, so lets disable now for safety */
  /*
	if (last_level_set.use_count()) {
		last_level_set->print();
		uint shader = 0;//scene->shader_manager->get_shader_id(scene->default_surface, NULL, false);
		printf("Used shader OVDB: %d\n", shader);
		scene->level_sets.push_back(new LevelSet(last_level_set, shader));
	}
  */
}

LevelSet::LevelSet( ) {
  //printf( "LevelSet Default Constructor\n" ); 
}

LevelSet::LevelSet(openvdb::FloatGrid::Ptr gridPtr, int shader_)
    : grid(gridPtr), shader(shader_)
{
  //printf( "LevelSet Explicit Constructor\n" ); 

	/* grid->print(); */
	// printf( "Initializing thread accessor mapping from thread %u.\n", pthread_self() ); 
	vector<pthread_t> ids = TaskScheduler::thread_ids();
	pthread_t my_thread = pthread_self();
	foreach( pthread_t id, ids) {
	  // printf( "New RayIntersector required for thread %u. Creating...\n", id);	  
	  isect_t* isector = new isect_t(*grid);
	  pair<pthread_t, isect_t *> isect(id, isector);		
	  isect_map.insert(isect);
	}
	/* Always add this thread, as it also seems to be used for rendering */
	if( isect_map.find(my_thread) == isect_map.end() ){
	  isect_t* isector = new isect_t(*grid);
	  pair<pthread_t, isect_t *> isect(my_thread, isector);		
	  isect_map.insert(isect);
	}
	  
}

LevelSet::LevelSet( const LevelSet& levelset )
  : grid( levelset.grid ), shader( levelset.shader )
{
  //printf( "LevelSet Copy Constructor\n" ); 

  // printf( "Initializing thread accessor mapping from thread %u.\n", pthread_self() ); 
  vector<pthread_t> ids = TaskScheduler::thread_ids();
  pthread_t my_thread = pthread_self();
  foreach( pthread_t id, ids) {
    // printf( "New RayIntersector required for thread %u. Creating...\n", id);	  
    isect_t* isector = new isect_t(*grid);
    pair<pthread_t, isect_t *> isect(id, isector);		
    isect_map.insert(isect);
  }
  /* Always add this thread, as it also seems to be used for rendering */
  if( isect_map.find(my_thread) == isect_map.end() ){
    isect_t* isector = new isect_t(*grid);
    pair<pthread_t, isect_t *> isect(my_thread, isector);		
    isect_map.insert(isect);
  }
  
}

LevelSet::~LevelSet()
{
	for(isect_map_t::iterator iter = isect_map.begin();
	    iter != isect_map.end();
	    ++iter)
	{
		delete iter->second;
	}
	isect_map.clear();
}

void LevelSet::initialize(openvdb::FloatGrid::Ptr& gridPtr, int shader_)
{
  // printf( "LevelSet Post-Construction Initializer\n" ); 
  grid.swap(gridPtr);
  shader = shader_;

  for(isect_map_t::iterator iter = isect_map.begin();
      iter != isect_map.end();
      ++iter)
    {
      delete iter->second;
    }

  //printf( "Initializing thread accessor mapping from thread %u.\n", pthread_self() ); 

  vector<pthread_t> ids = TaskScheduler::thread_ids();
  pthread_t my_thread = pthread_self();
  foreach( pthread_t id, ids) {
    // printf( "New RayIntersector required for thread %u. Creating...\n", id);	  
    isect_t* isector = new isect_t(*grid);
    pair<pthread_t, isect_t *> isect(id, isector);		
    isect_map.insert(isect);
  }
  /* Always add this thread, as it also seems to be used for rendering */
  if( isect_map.find(my_thread) == isect_map.end() ){
    isect_t* isector = new isect_t(*grid);
    pair<pthread_t, isect_t *> isect(my_thread, isector);		
    isect_map.insert(isect);
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
	assert( iter != isect_map.end() );
	isector = iter->second;

	vdb_ray_t::Vec3Type P(ray->P.x, ray->P.y, ray->P.z);
	vdb_ray_t::Vec3Type D(ray->D.x, ray->D.y, ray->D.z);
	D.normalize();

	vdb_ray_t vdbray(P, D, 1e-5f, ray->t);
	vdb_ray_t::Vec3Type pos, normal;
	float t;

	bool intersects = isector->intersectsWS(vdbray, pos, normal, t);


	if(intersects) {
                isect->t = t;
		isect->u = 0.333f;
		isect->v = 0.333f;
		isect->type = PRIMITIVE_LEVEL_SET;
		isect->shad = shader;
		//isect->norm = normalize(make_float3(normal.x(), normal.y(), normal.z()));
		isect->norm = make_float3(normal.x(), normal.y(), normal.z());
		isect->prim = 0;
		isect->object = OBJECT_NONE; // Do this to avoid instancing code.
                                             // Since levelsets are not part of the BVH,
                                             // object instancing does not apply.
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
	if (scene->level_sets.size() > 0){
	  
	  /* Allocate a memory pool big enough for all LevelSets */
	  void* levelset_pool = operator new (sizeof(LevelSet) * dscene->data.tables.num_level_sets );
	  
	  for( int ls = 0; ls < dscene->data.tables.num_level_sets; ls++ ){
	    /* Move into the pool by the appropriate amount */
	    void* pool_offset = levelset_pool + (sizeof( LevelSet ) * ls);
	    
	    /* We need to protect against potential leaks due to failed construction */
	    try {
	      LevelSet* temp_levelset_ptr = new(pool_offset) LevelSet( *(scene->level_sets[ls]) );
	    }
	    catch (...) {
	      /* if something goes wrong, rewind all constructed entries... */
	      for( int rls = ls-1; rls >= 0; rls-- ){
		void* rev_pool_offset = levelset_pool + (sizeof( LevelSet ) * rls);
		LevelSet* levelset_ptr = (LevelSet*)(rev_pool_offset);
		levelset_ptr->~LevelSet();
	      }

	      /* then destroy the pool itself. */
	      operator delete(levelset_pool);
	      
	      /* finally, toss the exception upward */
	      throw;
	    }
	  }
	  dscene->data.tables.level_sets = levelset_pool;

	}

	if(progress.get_cancel()) return;

	need_update = false;
}

void LevelSetManager::device_free(Device *device, DeviceScene *dscene)
{
  if( dscene->data.tables.num_level_sets > 0 ){
    /*we've allocated all the levelsets in a pool using placement new, special care is required */
    for( int ls = 0; ls < dscene->data.tables.num_level_sets; ls++ ){
      /* Move into the pool by the appropriate amount */
      void* pool_offset = dscene->data.tables.level_sets + (sizeof( LevelSet ) * ls);
      LevelSet* levelset_ptr = (LevelSet*)(pool_offset);
      /* explicitly call the destructor to clean up the placed object instance */
      levelset_ptr->~LevelSet();
    }
    /* now delete the memory pool */
    operator delete( dscene->data.tables.level_sets );
    dscene->data.tables.level_sets = NULL;
    dscene->data.tables.num_level_sets = 0;
  }
}

void LevelSetManager::tag_update(Scene */*scene*/)
{
	need_update = true;
}

CCL_NAMESPACE_END
