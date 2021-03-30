// Copyright (c) 2006-2011  GeometryFactory (France). All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL$
// $Id$
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Fernando Cacciola <fernando.cacciola@geometryfactory.com>
//
#ifndef CGAL_MODIFIABLE_PRIORITY_QUEUE_H
#define CGAL_MODIFIABLE_PRIORITY_QUEUE_H

#include <climits> // Neeeded by the following Boost header for CHAR_BIT.
#include <boost/optional.hpp>
#ifdef CGAL_SURFACE_MESH_SIMPLIFICATION_USE_RELAXED_HEAP
#include <boost/pending/relaxed_heap.hpp>
#else
#include <set>



namespace CGAL {
  namespace internal {
template <
      class IndexedType,
      class Comp
         >
class set_based_heap : public std::set<IndexedType, Comp>
{
  typedef std::set<IndexedType, Comp> Base;
public:
  typedef typename Base::size_type size_type;
  typedef typename Base::iterator handle;
  typedef typename Base::value_type value_type;

  set_based_heap(const Comp& x=Comp()) : Base(x)
  {}

  void remove(const IndexedType& x){
    this->erase(x);
  }

  bool contains(const IndexedType& x) const {
    return (this->find(x) != this->end());
  }

  handle update ( value_type const& v, handle h ) {
    auto node = this->extract(h);
    node.value() = v;
    this->insert(move(node));
    return h;
  }

  void update ( value_type const& v) {
    auto node = this->extract(v);
    this->insert(move(node));
  }

  void pop() { this->erase(this->begin()); }

  value_type top() const { return *this->begin(); }

  handle push ( value_type const& v ) { return this->insert(v).first; }
};

} } //namespace CGAL::internal
#endif //CGAL_SURFACE_MESH_SIMPLIFICATION_USE_RELAXED_HEAP

namespace CGAL {

template <class IndexedType_
         ,class Compare_ = std::less<IndexedType_>
         ,class ID_      = boost::identity_property_map
         >
class Modifiable_priority_queue
{
public:

  typedef Modifiable_priority_queue Self;

  typedef IndexedType_ IndexedType ;
  typedef Compare_     Compare;
  typedef ID_          ID ;

  #ifdef CGAL_SURFACE_MESH_SIMPLIFICATION_USE_RELAXED_HEAP
  typedef boost::relaxed_heap<IndexedType,Compare,ID> Heap;
  typedef bool handle ;
  #else
  typedef  internal::set_based_heap<IndexedType,Compare> Heap;
  typedef typename Heap::handle handle;
  #endif //CGAL_SURFACE_MESH_SIMPLIFICATION_USE_RELAXED_HEAP
  typedef typename Heap::value_type value_type;
  typedef typename Heap::size_type  size_type;


public:

  Modifiable_priority_queue( size_type, Compare const& c, ID const&) : mHeap(c) {}

  handle push ( value_type const& v ) { return mHeap.push(v); }

  handle update ( value_type const& v, handle h ) { return mHeap.update(v, h); }
  handle update ( value_type const& v, bool) { return mHeap.update(v); return null_handle();}

  handle erase ( value_type const& v, handle  ) { mHeap.remove(v); return null_handle() ; }
  handle erase ( value_type const& v  ) { mHeap.remove(v); return null_handle() ; }

  value_type top() const { return mHeap.top() ; }

  void pop() { mHeap.pop(); }

  bool empty() const { return mHeap.empty() ; }

  bool contains ( value_type const& v ) { return mHeap.contains(v) ; }

  boost::optional<value_type> extract_top()
  {
    boost::optional<value_type> r ;
    if ( !empty() )
    {
      value_type v = top();
      pop();
      r = boost::optional<value_type>(v) ;
    }
    return r ;
  }

  static handle null_handle() { return handle(); }

private:

  Heap mHeap ;

} ;

} //namespace CGAL

#endif

