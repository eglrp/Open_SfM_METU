#ifndef OPEN_SFM_METU_SOLVERS_UTIL_UTIL_HPP
#define OPEN_SFM_METU_SOLVERS_UTIL_UTIL_HPP

namespace Open_SfM_METU {
namespace solvers {



	typedef unsigned char uchar;

	// A macro to disallow the copy constructor and operator= functions
	// This should be used in the private: declarations for a class
	#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
	  TypeName(const TypeName&);               \
	  void operator=(const TypeName&)

	// Determines the array size an array a.
	#define THEIA_ARRAYSIZE(a) \
	  ((sizeof(a) / sizeof(*(a))) / \
	  static_cast<size_t>(!(sizeof(a) % sizeof(*(a)))))

	// Deletes all pointers in a container.
	template <class ForwardIterator>
	void STLDeleteContainerPointers(ForwardIterator begin,
	                                ForwardIterator end) {
	  while (begin != end) {
	    ForwardIterator temp = begin;
	    ++begin;
	    delete *temp;
	  }
	}

	// Deletes all pointers in an STL container (anything that has a begin() and
	// end() function)
	template <class T>
	void STLDeleteElements(T *container) {
	  if (!container) return;
	  STLDeleteContainerPointers(container->begin(), container->end());
	  container->clear();
	}


}
}


#endif // OPEN_SFM_METU_SOLVERS_UTIL_UTIL_HPP