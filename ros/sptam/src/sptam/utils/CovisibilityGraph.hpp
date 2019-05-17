/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2015 Taihú Pire and Thomas Fischer
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire <tpire at dc dot uba dot ar>
 *           Thomas Fischer <tfischer at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */
#pragma once

#include <list>
#include "Iterable.hpp"

#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/lock_types.hpp>

////////////////////////////////////////////////////////////////////////
// std::list< std::reference_wrapper<E> > iterator implementation
////////////////////////////////////////////////////////////////////////

template<typename E>
class RefListIterator : public IteratorBase<E>
{
  public:

    RefListIterator(typename std::list< std::reference_wrapper<E> >::iterator it)
      : it_( it )
    {}

    virtual void operator ++ ()
    { it_++; }

    virtual void operator ++ (int i)
    { assert( false ); }

    virtual E& operator * ()
    { return *it_; }

    const virtual E& operator * () const
    { return *it_; }

    virtual IteratorBase<E>* clone() const
    { return new RefListIterator( it_ ); }

  protected:

    virtual bool equal(const IteratorBase<E>& other) const
    { return it_ == ((RefListIterator&)other).it_; }

  private:

    typename std::list< std::reference_wrapper<E> >::iterator it_;
};

template<typename E>
class ConstRefListIterator : public ConstIteratorBase<E>
{
  public:

    ConstRefListIterator(typename std::list< std::reference_wrapper<E> >::const_iterator it)
      : it_( it )
    {}

    virtual void operator ++ ()
    { it_++; }

    virtual void operator ++ (int i)
    { assert( false ); }

  const virtual E& operator * () const
    { return *it_; }

  protected:

    virtual bool equal(const ConstIteratorBase<E>& other) const
    {
      return it_ == ((ConstRefListIterator&)other).it_;
    }

    virtual ConstIteratorBase<E>* clone() const
    {
      return new ConstRefListIterator( it_ );
    }

  private:

    typename std::list< std::reference_wrapper<E> >::const_iterator it_;
};

template<typename E>
class RefListIterable : public IterableBase<E>
{
  public:

    virtual Iterator<E> begin()
    {
      return Iterator<E>( std::unique_ptr< RefListIterator<E> >( new RefListIterator<E>( container_.begin() ) ) );
    }

    virtual ConstIterator<E> begin() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstRefListIterator<E>( container_.begin() ) ));
    }

    virtual Iterator<E> end()
    {
      return Iterator<E>( std::unique_ptr< RefListIterator<E> >( new RefListIterator<E>( container_.end() ) ) );
    }

    virtual ConstIterator<E> end() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstRefListIterator<E>( container_.end() ) ));
    }

    virtual bool empty() const
    { return container_.empty(); }

    virtual size_t size() const
    { return container_.size(); }

    static Iterable<E> from( std::list< std::reference_wrapper<E> >& container )
    {
      return Iterable<E>( std::unique_ptr< IterableBase<E> >( new RefListIterable<E>( container ) ) );
    }

  protected:

    RefListIterable( std::list< std::reference_wrapper<E> >& container ) : container_( container )
    {}

    std::list< std::reference_wrapper<E> >& container_;
};

template<typename E>
class ConstRefListIterable : public ConstIterableBase<E>
{
  public:

    virtual ConstIterator<E> begin() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstRefListIterator<E>( container_.begin() ) ));
    }

    virtual ConstIterator<E> end() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstRefListIterator<E>( container_.end() ) ));
    }

    virtual bool empty() const
    { return container_.empty(); }

    virtual size_t size() const
    { return container_.size(); }

    static ConstIterable<E> from( const std::list< std::reference_wrapper<E> >& container )
    {
      return ConstIterable<E>( std::unique_ptr< ConstIterableBase<E> >( new ConstRefListIterable<E>( container ) ) );
    }

  protected:

    ConstRefListIterable( const std::list< std::reference_wrapper<E> >& container ) : container_( container )
    {}

    const std::list< std::reference_wrapper<E> >& container_;
};

////////////////////////////////////////////////////////////////////////
// std::list< std::shared_ptr<E> > iterator implementation
////////////////////////////////////////////////////////////////////////

template<typename E>
class SharedPtrListIterator : public IteratorBase<E>
{
  public:

    SharedPtrListIterator(typename std::list< std::shared_ptr<E> >::iterator it)
      : it_( it )
    {}

    virtual void operator ++ ()
    { it_++; }

    virtual void operator ++ (int i)
    { assert( false ); }

    virtual E& operator * ()
    { return **it_; }

    const virtual E& operator * () const
    { return **it_; }

    virtual IteratorBase<E>* clone() const
    { return new SharedPtrListIterator( it_ ); }

  protected:

    virtual bool equal(const IteratorBase<E>& other) const
    { return it_ == ((SharedPtrListIterator&)other).it_; }

  private:

    typename std::list< std::shared_ptr<E> >::iterator it_;
};

template<typename E>
class ConstSharedPtrListIterator : public ConstIteratorBase<E>
{
  public:

    ConstSharedPtrListIterator(typename std::list< std::shared_ptr<E> >::const_iterator it)
      : it_( it )
    {}

    virtual void operator ++ ()
    { it_++; }

    virtual void operator ++ (int i)
    { assert( false ); }

  const virtual E& operator * () const
    { return **it_; }

  protected:

    virtual bool equal(const ConstIteratorBase<E>& other) const
    {
      return it_ == ((ConstSharedPtrListIterator&)other).it_;
    }

    virtual ConstIteratorBase<E>* clone() const
    {
      return new ConstSharedPtrListIterator( it_ );
    }

  private:

    typename std::list< std::shared_ptr<E> >::const_iterator it_;
};

template<typename E>
class SharedPtrListIterable : public IterableBase<E>
{
  public:

    virtual Iterator<E> begin()
    {
      return Iterator<E>( std::unique_ptr< SharedPtrListIterator<E> >( new SharedPtrListIterator<E>( container_.begin() ) ) );
    }

    virtual ConstIterator<E> begin() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSharedPtrListIterator<E>( container_.begin() ) ));
    }

    virtual Iterator<E> end()
    {
      return Iterator<E>( std::unique_ptr< SharedPtrListIterator<E> >( new SharedPtrListIterator<E>( container_.end() ) ) );
    }

    virtual ConstIterator<E> end() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSharedPtrListIterator<E>( container_.end() ) ));
    }

    virtual bool empty() const
    { return container_.empty(); }

    virtual size_t size() const
    { return container_.size(); }

    static Iterable<E> from( std::list< std::shared_ptr<E> >& container )
    {
      return Iterable<E>( std::unique_ptr< IterableBase<E> >( new SharedPtrListIterable<E>( container ) ) );
    }

  protected:

    SharedPtrListIterable( std::list< std::shared_ptr<E> >& container ) : container_( container )
    {}

    std::list< std::shared_ptr<E> >& container_;
};

template<typename E>
class ConstSharedPtrListIterable : public ConstIterableBase<E>
{
  public:

    virtual ConstIterator<E> begin() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSharedPtrListIterator<E>( container_.begin() ) ));
    }

    virtual ConstIterator<E> end() const
    {
      return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSharedPtrListIterator<E>( container_.end() ) ));
    }

    virtual bool empty() const
    { return container_.empty(); }

    virtual size_t size() const
    { return container_.size(); }

    static ConstIterable<E> from( const std::list< std::shared_ptr<E> >& container )
    {
      return ConstIterable<E>( std::unique_ptr< ConstIterableBase<E> >( new ConstSharedPtrListIterable<E>( container ) ) );
    }

  protected:

    ConstSharedPtrListIterable( const std::list< std::shared_ptr<E> >& container ) : container_( container )
    {}

    const std::list< std::shared_ptr<E> >& container_;
};

////////////////////////////////////////////////////////////////////////

// push_back/emplace_back methods that return an iterator
// to the newly inserted element.
#define INSERT_BACK( ls, x ) (ls).insert( (ls).end(), x )
#define EMPLACE_BACK( ls, ... ) (ls).emplace( (ls).end(), __VA_ARGS__ )

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
class CovisibilityGraph
{
  public:
    // forward declarations
    class KeyFrame;
    class MapPoint;
    class Measurement;

    typedef std::shared_ptr<MapPoint> SharedMapPoint;
    typedef std::shared_ptr<KeyFrame> SharedKeyFrame;
    typedef std::shared_ptr<Measurement> SharedMeasurement;

  private:

    std::list<SharedKeyFrame> keyframes_;
    std::list<SharedMapPoint> mappoints_;

    struct compareSharedMapPoints {
       bool operator() (const typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPoint& p1, const typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPoint& p2) const
      { return p1.get() < p2.get(); }
    };

    struct compareSharedKeyFrames {
       bool operator() (const typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedKeyFrame& k1, const typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedKeyFrame& k2) const
      { return k1.get() < k2.get(); }
    };

    struct compareSharedMeasurements {
       bool operator() (const typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMeasurement& m1, const typename CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMeasurement& m2) const
      { return m1.get() < m2.get(); }
    };

  public:

    // This needs to be public because we need to use the same
    // comparison function in multiple places to search for duplicates
    // in a set :(
    typedef  std::map< SharedMapPoint, size_t, compareSharedMapPoints> SharedMapPointMap ;
    typedef  std::pair<CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedMapPoint,size_t> SharedPointIntPair ;

    
    typedef std::set<SharedMapPoint, compareSharedMapPoints> SharedMapPointSet;
    typedef std::set<SharedKeyFrame, compareSharedKeyFrames> SharedKeyFrameSet;
    typedef std::set<SharedMeasurement, compareSharedMeasurements> SharedMeasurementSet;

    typedef std::list<SharedKeyFrame> SharedKeyFrameList;
    typedef std::list<SharedMapPoint> SharedMapPointList;

    SharedKeyFrame addKeyFrame( const KEYFRAME_T& keyFrame );
    void removeKeyFrame( const SharedKeyFrame& keyFrame );

    SharedMapPoint addMapPoint( const MAP_POINT_T& mapPoint );
    void removeMapPoint( const SharedMapPoint& mapPoint );

    void addMeasurement( SharedKeyFrame& keyFrame, SharedMapPoint& mapPoint, const MEAS_T& edge );
    void removeMeasurement( const SharedMeasurement& edge );

    static inline ConstIterable<SharedKeyFrame> createIterable( SharedKeyFrameSet& set )
    { return ConstSetIterable<SharedKeyFrame, compareSharedKeyFrames>::from( set ); }

    SharedMapPointSet getLocalMap(/*const */KeyFrame& keyFrame);

    void getLocalMap(const SharedMapPointSet& trackedPoints, SharedMapPointSet& localMap, SharedKeyFrameSet& localKeyFrames,
                     SharedKeyFrame& referenceKeyFrame);

    const SharedKeyFrameList& getKeyframes() const
    { return keyframes_; }

    const SharedMapPointList& getMapPoints() const
    { return mappoints_; }

  // Extensions for data classes

    class KeyFrame : public KEYFRAME_T, public std::enable_shared_from_this<KeyFrame>
    {
      public:

        KeyFrame( const KEYFRAME_T& elem ) : KEYFRAME_T( elem ) {}

        KeyFrame( const KeyFrame& other ) = delete; // non construction-copyable
        KeyFrame& operator=( const KeyFrame& ) = delete; // non copyable

        std::list< SharedMeasurement > measurements() const
        {
          boost::shared_lock<boost::shared_mutex> lock( meas_mutex_ );
          return  measurements_;
        }

        std::map< SharedKeyFrame, size_t, compareSharedKeyFrames> covisibilityKeyFrames()
        {
          boost::shared_lock<boost::shared_mutex> lock( covisibilityKeyframes_mutex_ );
          return covisibilityKeyFrames_;
        }

        std::vector< std::pair<SharedKeyFrame, size_t> > covisibilityKeyFramesVector()
        {
          boost::shared_lock<boost::shared_mutex> lock( covisibilityKeyframes_mutex_ );
          std::vector<std::pair<SharedKeyFrame, size_t> > vCovisibilityKeyframes(covisibilityKeyFrames_.begin(), covisibilityKeyFrames_.end());
          return vCovisibilityKeyframes;
        }

      private:

        void addCovisibilityKeyframe(SharedKeyFrame covisibilityKeyFrame) {
            boost::unique_lock<boost::shared_mutex> lock( covisibilityKeyframes_mutex_ );

            auto it = covisibilityKeyFrames_.find( covisibilityKeyFrame );

            if (it != covisibilityKeyFrames_.end())
              it->second++;   // Increase the covisibility value if the edge already exists
            else // was not found, create covisibility edge if it does not exists
              covisibilityKeyFrames_.insert(std::pair<CovisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::SharedKeyFrame,size_t>(covisibilityKeyFrame, 1));
        }


        void addMeasurement(SharedMeasurement& meas) {
          boost::unique_lock<boost::shared_mutex> lock( meas_mutex_ );
          meas->it_keyFrame_ = INSERT_BACK(measurements_, meas);
        }

        mutable boost::shared_mutex meas_mutex_;

        mutable boost::shared_mutex covisibilityKeyframes_mutex_;

        mutable std::list< SharedMeasurement > measurements_;

        mutable std::map< SharedKeyFrame, size_t, compareSharedKeyFrames> covisibilityKeyFrames_;

        void setIteratorToContainer(const typename std::list<SharedKeyFrame>::iterator& it){ to_container_ = it; }

        typename std::list<SharedKeyFrame>::iterator to_container_;

        friend class CovisibilityGraph;
    };

    class MapPoint : public MAP_POINT_T
    {
       public:

        MapPoint( const MAP_POINT_T& elem ) : MAP_POINT_T( elem ) {}

        MapPoint( const MapPoint& other ) = delete; // non construction-copyable
        MapPoint& operator=( const MapPoint& ) = delete; // non copyable

        std::list< SharedMeasurement > measurements() const
        {
          boost::shared_lock<boost::shared_mutex> lock( meas_mutex_ );
          return measurements_;
        }

      private:

        void addMeasurement(SharedMeasurement& meas) {
          boost::unique_lock<boost::shared_mutex> lock( meas_mutex_ );
          meas->it_mapPoint_ = INSERT_BACK(measurements_, meas);
        }

        mutable boost::shared_mutex meas_mutex_;

        mutable std::list< SharedMeasurement > measurements_;

        void setIteratorToContainer(const typename std::list<SharedMapPoint>::iterator& it){ to_container_ = it; }

        typename std::list<SharedMapPoint>::iterator to_container_;

        friend class CovisibilityGraph;
    };

    class Measurement : public MEAS_T
    {
      public:

        Measurement(const MEAS_T& edge, SharedKeyFrame& keyFrame, SharedMapPoint& mapPoint)
          : MEAS_T( edge ), keyFrame_( keyFrame ), mapPoint_( mapPoint ) {}

        Measurement( const Measurement& other ) = delete; // non construction-copyable
        Measurement& operator=( const Measurement& ) = delete; // non copyable

        inline SharedKeyFrame& keyFrame()
        { return keyFrame_; }

        inline const SharedKeyFrame& keyFrame() const
        { return keyFrame_; }

        inline SharedMapPoint& mapPoint()
        { return mapPoint_; }

        inline const SharedMapPoint& mapPoint() const
        { return mapPoint_; }

      private:

        SharedKeyFrame keyFrame_;
        typename std::list< SharedMeasurement >::/*const_*/iterator it_keyFrame_;

        SharedMapPoint mapPoint_;
        typename std::list< SharedMeasurement >::/*const_*/iterator it_mapPoint_;

        friend class CovisibilityGraph;
    };
};




