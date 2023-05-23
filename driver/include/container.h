#pragma once

#include "const_xslam.h"
#include "ccommon.h"

void *aligned_malloc(int size);
void *aligned_calloc(int nitems, int size);
void aligned_free(void *ptr);
void heapStatInit ();
void displayHeapStat();
void abort_if_heap_is_not_empty();

Vector create_vector(int nb_element, int element_size);

void reserve(Vector*, int reserved);
void free_vector(Vector*);
void resize(Vector* vector, int new_size);
void clear(Vector* vector);
void push_backc(Vector* vector, void const* value);
int capacity(Vector const*);
int size(Vector const*);
void* data(Vector*);
void* cdata(Vector const*);
void* front(Vector*);
void* back(Vector*);
void push_back(Vector*, void * value);
void set(Vector* , int index, void * value);
void setc(Vector* vector, int index, void const* value);
void* get(Vector* , int index);
void* cget(Vector const* , int index);
void disp_vector_info(Vector const*);
void fill( Vector* , void * value);
void free_vector_of_vector(Vector*);


VectorInt create_vector_int(int nb_element);
void iset(VectorInt* vint, int index, int value);
int* get_int(VectorInt* vint, int index);
void free_vector_int(VectorInt*);
void ipush_back(VectorInt*,int);
int isize(VectorInt*);
int const* cget_int(VectorInt const* vint, int index);

void free_vector_of_vector_int(Vector*);

void ipush_back_int2(VectorInt2*,IVec2);
VectorInt2 create_vector_int2(int nb_element);
void free_vector_int2(VectorInt2*);

#if 0

#include <iostream>

template<typename T> struct TArray
{
  T const* v;
  int size;
};

template<typename T> struct TVector
{
  Vector const &v;
};

template<typename T> TVector<T> tv( Vector const& v)
{
  return TVector<T>{v};
}

inline TVector<int> tv( VectorInt const& v)
{
  return TVector<int>{v.vector};
}

inline TArray<int> tv( int const * array, int size)
{
  return TArray<int>{array,size};
}

template<typename T> std::ostream& operator<<(std::ostream& o, TArray<T> const& ta)
{
  auto v = ta.v;
  //o << " Disp vector " << v.capacity << "; " <<v.nb_element << " / " << v.capacity << std::endl;

  for(int i = 0 ; i < ta.size ; ++i)
  {
    //o << "[" << i << "]:" << *((T*)get(&v,i)) << " ";
    o << ta.v[i] << " ";
  }
  return o;
}

template<typename T> std::ostream& operator<<(std::ostream& o, TVector<T> const& tv)
{
  auto& v = tv.v;
  //o << " Disp vector " << v.capacity << "; " <<v.nb_element << " / " << v.capacity << std::endl;

  for(int i = 0 ; i < v.nb_element ; ++i)
  {
    //o << "[" << i << "]:" << *((T*)get(&v,i)) << " ";
    o << *((T*)get(&v,i)) << " ";
  }
  return o;
}

inline std::ostream& operator<<(std::ostream& o, VectorInt const& vint)
{
  auto& v = vint.vector;
  //o << " Disp vector " << v.capacity << "; " <<v.nb_element << " / " << v.capacity << std::endl;

  for(int i = 0 ; i < v.nb_element ; ++i)
  {
    //o << "[" << i << "]:" << *((T*)get(&v,i)) << " ";
    o << *((int*)get(&v,i)) << " ";
  }
  return o;
}

inline std::ostream& operator<<(std::ostream& o, TVector<VectorInt> const& tv)
{
  auto& v = tv.v;
  //o << " Disp vector " << v.capacity << "; " <<v.nb_element << " / " << v.capacity << std::endl;

  for(int i = 0 ; i < v.nb_element ; ++i)
  {
    o << *((VectorInt*)get(&v,i));// << std::endl;
  }
  return o;
}

#endif
