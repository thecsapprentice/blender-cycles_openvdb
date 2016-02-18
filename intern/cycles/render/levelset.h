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


namespace openvdb{
OPENVDB_USE_VERSION_NAMESPACE
  namespace OPENVDB_VERSION_NAME {
  namespace tools {

//////////////////////////////////////// CyclesLinearSearchImpl ////////////////////////////////////////


/// @brief Implements linear iterative search for an iso-value of
/// the level set along along the direction of the ray. Provides customized functionality for
/// cycles renderer
///
/// @note Since this class is used internally in
/// LevelSetRayIntersector (define above) and LevelSetHDDA (defined below)
/// client code should never interact directly with its API. This also
/// explains why we are not concerned with the fact that several of
/// its methods are unsafe to call unless roots were already detected.
///
/// @details It is approximate due to the limited number of iterations
/// which can can be defined with a template parameter. However the default value
/// has proven surprisingly accurate and fast. In fact more iterations
/// are not guaranteed to give significantly better results.
///
/// @warning Since the root-searching algorithm is approximate
/// (first-order) it is possible to miss intersections if the
/// iso-value is too close to the inside or outside of the narrow
/// band (typically a distance less then a voxel unit).
///
/// @warning Since this class internally stores a ValueAccessor it is NOT thread-safe,
/// so make sure to give each thread its own instance.  This of course also means that
/// the cost of allocating an instance should (if possible) be amortized over
/// as many ray intersections as possible.
    template<typename GridT, typename StencilT, int Iterations, typename RealT, bool use_bisection_secant_root=false>
class CyclesLinearSearchImpl
{
public:
    typedef math::Ray<RealT>              RayT;
    typedef typename GridT::ValueType     ValueT;
    typedef typename GridT::ConstAccessor AccessorT;
    typedef typename StencilT::Vec3Type   Vec3T;

    /// @brief Constructor from a grid.
    /// @throw RunTimeError if the grid is empty.
    /// @throw ValueError if the isoValue is not inside the narrow-band.
    CyclesLinearSearchImpl(const GridT& grid, const ValueT& isoValue = zeroVal<ValueT>())
        : mStencil(grid),
          mIsoValue(isoValue),
          mMinValue(isoValue - ValueT(2 * grid.voxelSize()[0])),
          mMaxValue(isoValue + ValueT(2 * grid.voxelSize()[0])),
	  mDX(grid.voxelSize()[0])
      {
          if ( grid.empty() ) {
              OPENVDB_THROW(RuntimeError, "LinearSearchImpl does not supports empty grids");
          }
          if (mIsoValue<= -grid.background() ||
              mIsoValue>=  grid.background() ){
              OPENVDB_THROW(ValueError, "The iso-value must be inside the narrow-band!");
          }
          grid.tree().root().evalActiveBoundingBox(mBBox, /*visit individual voxels*/false);
      }

    /// @brief Return the iso-value used for ray-intersections
    const ValueT& getIsoValue() const { return mIsoValue; }

    /// @brief Return @c false the ray misses the bbox of the grid.
    /// @param iRay Ray represented in index space.
    /// @warning Call this method before the ray traversal starts.
    inline bool setIndexRay(const RayT& iRay)
    {
        mRay = iRay;
        return mRay.clip(mBBox);//did it hit the bbox
    }

    /// @brief Return @c false the ray misses the bbox of the grid.
    /// @param wRay Ray represented in world space.
    /// @warning Call this method before the ray traversal starts.
    inline bool setWorldRay(const RayT& wRay)
    {
        mRay = wRay.worldToIndex(mStencil.grid());
        return mRay.clip(mBBox);//did it hit the bbox
    }

    /// @brief Get the intersection point in index space.
    /// @param xyz The position in index space of the intersection.
    inline void getIndexPos(Vec3T& xyz) const { xyz = mRay(mTime); }

    /// @brief Get the intersection point in world space.
    /// @param xyz The position in world space of the intersection.
    inline void getWorldPos(Vec3T& xyz) const { xyz = mStencil.grid().indexToWorld(mRay(mTime)); }

    /// @brief Get the intersection point and normal in world space
    /// @param xyz The position in world space of the intersection.
    /// @param nml The surface normal in world space of the intersection.
    inline void getWorldPosAndNml(Vec3T& xyz, Vec3T& nml)
    {
        this->getIndexPos(xyz);
        mStencil.moveTo(xyz);
        nml = mStencil.gradient(xyz);
        nml.normalize();
        xyz = mStencil.grid().indexToWorld(xyz);
    }

    /// @brief Return the time of intersection along the index ray.
    inline RealT getIndexTime() const { return mTime; }

    /// @brief Return the time of intersection along the world ray.
    inline RealT getWorldTime() const
    {
        return mTime*mStencil.grid().transform().baseMap()->applyJacobian(mRay.dir()).length();
    }

private:

    /// @brief Initiate the local voxel intersection test.
    /// @warning Make sure to call this method before the local voxel intersection test.
    inline void init(RealT t0)
    {
        mT[0] = t0;
        mV[0] = static_cast<ValueT>(this->interpValue(t0));
    }

    inline void setRange(RealT t0, RealT t1) { mRay.setTimes(t0, t1); }

    /// @brief Return a const reference to the ray.
    inline const RayT& ray() const { return mRay; }

    /// @brief Return true if a node of the the specified type exists at ijk.
    template <typename NodeT>
    inline bool hasNode(const Coord& ijk)
    {
        return mStencil.accessor().template probeConstNode<NodeT>(ijk) != NULL;
    }

    /// @brief Return @c true if an intersection is detected.
    /// @param ijk Grid coordinate of the node origin or voxel being tested.
    /// @param time Time along the index ray being tested.
    /// @warning Only if and intersection is detected is it safe to
    /// call getIndexPos, getWorldPos and getWorldPosAndNml!
    inline bool operator()(const Coord& ijk, RealT time)
    {
      if(use_bisection_secant_root){
	// Implementation is derived from the PhysBAM Bisection_Secant_Root code
	ValueT V;
	if (mStencil.accessor().probeValue(ijk, V) &&
	    V > mMinValue && V < mMaxValue ){
	  mT[1] = time;
	  mV[1] = static_cast<ValueT>(this->interpValue(time));

	  if (math::ZeroCrossing(mV[0], mV[1])) {
	    RealT tolerance = 1e-6;
	    int max_iterations = 100;
	    
	    RealT a=mT[0];
	    RealT b=mT[1];
	    int iterations=0;
	    ValueT Fa=mV[0];
	    ValueT Fb=mV[1];
	    RealT x_old=a;
	    RealT x=b;
	    ValueT Fx_old=Fa;
	    ValueT Fx=Fb;
	    while( b-a > tolerance && iterations++<max_iterations ){
	      if(abs(Fx-Fx_old)<tolerance){ // bisection method
		RealT m=0.5*(a+b);
		ValueT Fm=static_cast<ValueT>(this->interpValue(m));
		if(Fa*Fm<=0){b=m;Fb=Fm;}
		else{a=m;Fa=Fm;}
		x_old=a;x=b;Fx_old=Fa;Fx=Fb; // Update secant points
	      }
	      else{ // secant method
		RealT x_temp=x;
		x-=Fx*(x-x_old)/(Fx-Fx_old);
		if(a<x && x<b){ // Update bisection points
		  x_old=x_temp;
		  Fx_old=Fx;
		  Fx=static_cast<ValueT>(this->interpValue(x));
		  RealT m=x;
		  ValueT Fm=Fx;
		  if(Fa*Fm<=0){b=m;Fb=Fm;}
		  else{a=m;Fa=Fm;}
		} 
		else{ // throw out secant root - do bisection instead
		  RealT m=0.5*(a+b);
		  ValueT Fm=static_cast<ValueT>(this->interpValue(m));
		  if(Fa*Fm<=0){b=m;Fb=Fm;}
		  else{a=m;Fa=Fm;}
		  x_old=a;x=b;Fx_old=Fa;Fx=Fb;
		}
	      }
	    }  	
	    if(iterations==max_iterations){
	      printf("max iterations reached, tolerance not reached.");
	    }
	    mTime=x;
	    return true;
	  }
	  mT[0] = mT[1];
	  mV[0] = mV[1];
	}
	return false;
      }
      else{     
        ValueT V;
        if (mStencil.accessor().probeValue(ijk, V) &&//within narrow band
            V>mMinValue && V<mMaxValue) {// and close to iso-value?
            mT[1] = time;
            mV[1] = static_cast<ValueT>(this->interpValue(time));
            if (math::ZeroCrossing(mV[0], mV[1])) {
                mTime = this->interpTime();
		if( mTime != mTime )
		  throw 1;
                OPENVDB_NO_UNREACHABLE_CODE_WARNING_BEGIN
                for (int n=0; Iterations>0 && n<Iterations; ++n) {//resolved at compile-time
                    V = static_cast<ValueT>(this->interpValue(mTime));
                    const int m = math::ZeroCrossing(mV[0], V) ? 1 : 0;
                    mV[m] = V;
                    mT[m] = mTime;
                    mTime = this->interpTime();
                }
                OPENVDB_NO_UNREACHABLE_CODE_WARNING_END
                return true;
            }
            mT[0] = mT[1];
            mV[0] = mV[1];
        }
        return false;
      }
    }

    inline RealT interpTime()
    {
        assert(math::isApproxLarger(mT[1], mT[0], RealT(1e-6)));
        return mT[0]+(mT[1]-mT[0])*mV[0]/(mV[0]-mV[1]);
    }

    inline RealT interpValue(RealT time)
    {
        const Vec3T pos = mRay(time);
        mStencil.moveTo(pos);
        return mStencil.interpolation(pos) - mIsoValue;
    }

    template<typename, int> friend struct math::LevelSetHDDA;

    RayT            mRay;
    StencilT        mStencil;
    RealT           mTime;//time of intersection
    ValueT          mV[2];
    RealT           mT[2];
    const ValueT    mIsoValue, mMinValue, mMaxValue, mDX;
    math::CoordBBox mBBox;
};// CyclesLinearSearchImpl
  } // namespace math
} // namespace OPENVDB_VERSION_NAME
} // namespace openvdb


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
  size_t operator() (const pthread_t& val) const {
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
#define OPENVDB_LEVELSET_SEARCH_ITERATIONS 0

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

       typedef openvdb::tools::CyclesLinearSearchImpl<openvdb::FloatGrid, openvdb::math::BoxStencil<openvdb::FloatGrid>, 
						      OPENVDB_LEVELSET_SEARCH_ITERATIONS, float, true> LinearSearchImpl;
       typedef openvdb::math::Ray<float> vdb_ray_t;
       typedef openvdb::tools::LevelSetRayIntersector<openvdb::FloatGrid,
	                                                  LinearSearchImpl,
	                                                  openvdb::FloatTree::RootNodeType::ChildNodeType::LEVEL,
	                                                  vdb_ray_t> isect_t;
       typedef boost::shared_ptr<isect_t> isect_t_ptr;

   /* used to ensure each thread gets its own intersector */
   /* if we really have true unordered_maps, use more appropriate hash and equality operators */
#if defined(CYCLES_TR1_UNORDERED_MAP) || defined(CYCLES_STD_UNORDERED_MAP) || defined(CYCLES_STD_UNORDERED_MAP_IN_TR1_NAMESPACE)
           typedef unordered_map<pthread_t, isect_t_ptr,
				 pthread_hash, pthread_equal_to > isect_map_t;
#else
           typedef unordered_map<pthread_t, isect_t_ptr > isect_map_t;
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
