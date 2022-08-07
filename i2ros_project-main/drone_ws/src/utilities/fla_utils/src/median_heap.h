// median_heap.h

#ifndef MEDIAN_HEAP_H
#define MEDIAN_HEAP_H

#include <algorithm>
#include <vector>

namespace fla_utils {

template <typename T>
class MedianHeap
{
public:
  MedianHeap()
  {
    clear();
  }

  T get_median()
  {
    if ( bottom_.size() > top_.size() ) {
      return bottom_[0];
    } else {
      return (bottom_[0] - top_[0]) / 2;
    }
  }

  void clear()
  {
    bottom_.clear();
    top_.clear();
  }

  bool empty()
  {
    return (bottom_.empty() && top_.empty());
  }

  std::size_t size()
  {
    return bottom_.size() + top_.size();
  }

  void push(T v)
  {
    if ( ( bottom_.empty() && top_.empty() ) || ( !top_.empty() && v < -top_[0] ) || ( !bottom_.empty() && v <= bottom_[0] ) ) {
      bottom_.push_back( v );
      std::push_heap( bottom_.begin(), bottom_.end() );  // heapify
    } else {
      top_.push_back( -v );
      std::push_heap( top_.begin(), top_.end() );  // heapify
    }
    rebalance();
  }

  bool pop(T v)
  {
    if ( !bottom_.empty() && v <= bottom_[0] ) {  // remove from top list if it exists
      auto it = std::find(bottom_.begin(), bottom_.end(), v);
      if ( it != bottom_.end() ) {
        bottom_.erase( it );  // remove value we found
        std::make_heap(bottom_.begin(), bottom_.end());  // need to remake entire heap!
        rebalance();
        return true;
      }
    } else if ( !top_.empty() ) {
      auto it = std::find(top_.begin(), top_.end(), -v);
      if ( it != top_.end() ) {        
        top_.erase( it );  // remove value we found
        std::make_heap(top_.begin(), top_.end() );  // need to remake entire heap!
        rebalance();
        return true;
      }
    }
    return false;
  }

  void print()
  {
    std::cout << "Bottom: ";
    for (int i=0; i < bottom_.size(); ++i) {
      std::cout << " " << bottom_[i];
    }
    std::cout << "\nTop: ";
    for (int i=0; i < top_.size(); ++i) {
      std::cout << " " << -top_[i];
    }
    std::cout << std::endl;
  }

private:
  void rebalance()
  {
    // Enforces both heaps are equal in size (or bottom is 1 bigger)
    while ( top_.size() > bottom_.size() ) {
      // Remove value from top
      T v = -top_[0];
      std::pop_heap(top_.begin(), top_.end());  // push max value to back of vector
      top_.pop_back();  // actually shorten vector

      // Now add to bottom
      bottom_.push_back(v);
      std::push_heap(bottom_.begin(), bottom_.end());
    }

    while ( bottom_.size() > top_.size()+1 ) {
      // Remove value from bottom
      T v = bottom_[0];
      std::pop_heap(bottom_.begin(), bottom_.end());  // push max value to back of vector
      bottom_.pop_back();  // actually shorten vector

      // Now add to top
      top_.push_back(-v);
      std::push_heap(top_.begin(), top_.end());
    }
  }

private:
  std::vector<T> bottom_;
  std::vector<T> top_;
};

}

#endif
