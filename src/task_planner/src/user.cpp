#include <iostream>
#include <sstream>
#include <algorithm>
#include <stdexcept>

#include "partition.h"

int main(int argc, const char *const *argv)
{
  // optional argument specifies size of the set (1--26)
  unsigned size = 4;
  if (argc >= 2) {
    std::stringstream stream; stream << argv[1];
    unsigned new_size;        stream >> new_size;
    
    if (new_size >= 1 && new_size <= 26)
      size = new_size;
  }

  // initialize the set
  std::vector<char> v(size);
  for (unsigned i = 0; i < size; ++i)
    v[i] = 'a' + i;

  // optional second argument is the partition size (1--size)
  // if (argc <= 2) {
    try {
      partition::iterator it(size);

      while (true) {
        std::cout << it << " : " << it.subsets() << " : ";

        std::auto_ptr<std::vector<std::vector<char> > >
          part = it[v];
        std::cout << *part << '\n';

        ++it;
      }
    } catch (std::overflow_error&) {}
  /* } else {
    unsigned psize = size / 2 + 1;

    std::stringstream stream;  stream << argv[2];
    unsigned new_psize;        stream >> new_psize;
    
    if (new_psize >= 1 && new_psize <= size)
      psize = new_psize;

    try {
      partition::iterator_k it(size, psize);

      while (true) {
        std::cout << it << " : " << it.subsets() << " : ";

        std::auto_ptr<std::vector<std::vector<char> > >
          part = it[v];
        std::cout << *part << '\n';

        ++it;
      }
    } catch (std::overflow_error&) {}
  } */
}
