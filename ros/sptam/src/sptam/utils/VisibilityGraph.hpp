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

// push_back/emplace_back methods that return an iterator
// to the newly inserted element.
#define INSERT_BACK( ls, x ) (ls).insert( (ls).end(), x );
#define EMPLACE_BACK( ls, ... ) (ls).emplace( (ls).end(), __VA_ARGS__ );

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
class VisibilityGraph
{
  public:

    // forward declaration
    class KeyFrame;
    class MapPoint;
    class Measurement;

    KeyFrame& addKeyFrame( const KEYFRAME_T& keyFrame );
    void removeKeyFrame( const KeyFrame& keyFrame );

    MapPoint& addMapPoint( const MAP_POINT_T& mapPoint );
    void removeMapPoint( const MapPoint& mapPoint );

    const Measurement& addMeasurement( KeyFrame& keyFrame, MapPoint& mapPoint, const MEAS_T& edge );
    void removeMeasurement( const Measurement& edge );

    std::list<KeyFrame>& keyFrames()
    { return keyframes_; }

    const std::list<KeyFrame>& keyFrames() const
    { return keyframes_; }

    std::list<MapPoint>& mapPoints()
    { return mappoints_; }

    const std::list<MapPoint>& mapPoints() const
    { return mappoints_; }

  // Extensions for data classes

    class KeyFrame : public KEYFRAME_T
    {
      public:

        KeyFrame( const KEYFRAME_T& elem ) : KEYFRAME_T( elem ) {}

        KeyFrame( const KeyFrame& other ) = delete; // non construction-copyable
        KeyFrame& operator=( const KeyFrame& ) = delete; // non copyable

        Iterable<Measurement> measurements()
        { return RefListIterable<Measurement>::from( measurements_ ); }

        ConstIterable<Measurement> measurements() const
        { return ConstRefListIterable<Measurement>::from( measurements_ ); }

      private:

        mutable std::list< std::reference_wrapper<Measurement> > measurements_;

        typename std::list<KeyFrame>::/*const_*/iterator it_;

        friend class VisibilityGraph;
    };

    class MapPoint : public MAP_POINT_T
    {
       public:

        MapPoint( const MAP_POINT_T& elem ) : MAP_POINT_T( elem ) {}

        MapPoint( const MapPoint& other ) = delete; // non construction-copyable
        MapPoint& operator=( const MapPoint& ) = delete; // non copyable

        Iterable<Measurement> measurements()
        { return RefListIterable<Measurement>::from( measurements_ ); }

        ConstIterable<Measurement> measurements() const
        { return ConstRefListIterable<Measurement>::from( measurements_ ); }

      private:

        mutable std::list< std::reference_wrapper<Measurement> > measurements_;

        typename std::list<MapPoint>::/*const_*/iterator it_;

        friend class VisibilityGraph;
    };

    class Measurement : public MEAS_T
    {
      public:

        Measurement(const MEAS_T& edge, KeyFrame& keyFrame, MapPoint& mapPoint)
          : MEAS_T( edge ), keyFrame_( keyFrame ), mapPoint_( mapPoint ) {}

        Measurement( const Measurement& other ) = delete; // non construction-copyable
        Measurement& operator=( const Measurement& ) = delete; // non copyable

        inline KeyFrame& keyFrame()
        { return keyFrame_; }

        inline const KeyFrame& keyFrame() const
        { return keyFrame_; }

        inline MapPoint& mapPoint()
        { return mapPoint_; }

        inline const MapPoint& mapPoint() const
        { return mapPoint_; }

      private:

        KeyFrame& keyFrame_;
        typename std::list< std::reference_wrapper<Measurement> >::/*const_*/iterator it_keyFrame_;

        MapPoint& mapPoint_;
        typename std::list< std::reference_wrapper<Measurement> >::/*const_*/iterator it_mapPoint_;

        typename std::list<Measurement>::/*const_*/iterator it_;

        friend class VisibilityGraph;
    };

  private:

    std::list<KeyFrame> keyframes_;
    std::list<MapPoint> mappoints_;
    std::list<Measurement> measurements_;
};

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
typename VisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::KeyFrame& VisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::addKeyFrame( const KEYFRAME_T& keyFrame )
{
  auto it = EMPLACE_BACK( keyframes_, keyFrame );
  it->it_ = it;
  return *it;
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
void VisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::removeKeyFrame( const VisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::KeyFrame& keyFrame )
{
  while ( not keyFrame.measurements_.empty() )
    removeMeasurement( keyFrame.measurements_.front() );

  keyframes_.erase( keyFrame.it_ );
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
typename VisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::MapPoint& VisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::addMapPoint( const MAP_POINT_T& mapPoint )
{
  auto it = EMPLACE_BACK(mappoints_, mapPoint );
  it->it_ = it;
  return *it;
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
void VisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::removeMapPoint( const VisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::MapPoint& mapPoint )
{
  while ( not mapPoint.measurements_.empty() )
    removeMeasurement( mapPoint.measurements_.front() );

  mappoints_.erase( mapPoint.it_ );
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
const typename VisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::Measurement& VisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::addMeasurement( KeyFrame& keyFrame, MapPoint& mapPoint, const MEAS_T& edge )
{
  auto it = EMPLACE_BACK( measurements_, edge, keyFrame, mapPoint );
  it->it_ = it;

  it->it_keyFrame_ = INSERT_BACK(keyFrame.measurements_, *it);
  it->it_mapPoint_ = INSERT_BACK(mapPoint.measurements_, *it);

  return *it;
}

template<typename KEYFRAME_T, typename MAP_POINT_T, typename MEAS_T>
void VisibilityGraph<KEYFRAME_T, MAP_POINT_T, MEAS_T>::removeMeasurement( const Measurement& edge )
{
  edge.keyFrame_.measurements_.erase( edge.it_keyFrame_ );
  edge.mapPoint_.measurements_.erase( edge.it_mapPoint_ );
  measurements_.erase( edge.it_ );
}
