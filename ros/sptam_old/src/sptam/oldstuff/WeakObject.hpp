#pragma once

#include <memory>

/**
 * @brief TODO
 */
template<class T>
class WeakObject
{
  public:

    class Ptr
    {
      public:

        Ptr( const Ptr& other )
          : object_( other.object_ ), active_( other.active_ )
        {}

        Ptr& operator=( const Ptr& ) = delete; // non copyable

        bool isActive() const
        { return *active_; }

        // dereference operator
        T& operator * ()
        { assert( *active_ ); return (T&) object_; }

      private:

        friend Ptr WeakObject::getWeakPtr();

        // Only the WeakObject may create its pointers.
        explicit Ptr( WeakObject& object )
          : object_( object ), active_( object_.active_ )
        {}

        WeakObject& object_;

        /**
         * Shared memory between the object and its pointers.
         * Weak Ptr should not modify the value, only query it.
         * Only the WeakObject can mark it as active when it is created
         * or inactive when it it destroyed.
         */
        std::shared_ptr<bool> active_;
    };

    WeakObject()
      : active_( new bool( true ) )
    {}

    WeakObject( const WeakObject& other )
      : active_( new bool( true ) )
    {}

    WeakObject& operator=( const WeakObject& ) = delete; // non copyable

    virtual ~WeakObject()
    { *active_ = false; }

    Ptr getWeakPtr()
    { return Ptr( *this ); }

  private:

    // Shared memory between the object and its weak pointers.
    std::shared_ptr<bool> active_;
}; 
