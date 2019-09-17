#pragma once

#include <iterator>
#include <vector>
#include <iostream>
#include <memory>
#include <algorithm>
#include <stdexcept>

template <typename Elem>
std::ostream &
operator<<(std::ostream &out, std::vector<Elem> &s)
{
  out << '(';

  for (unsigned i = 0; i < s.size() - 1; ++i)
  {
    out << s[i] << ' ';
  }
  out << *(s.end() - 1) << ')';

  return out;
}

template <typename Elem>
std::ostream &
operator<<(std::ostream &out,
           std::vector<std::vector<Elem>> &part)
{
  out << '(';

  for (unsigned i = 0; i < part.size() - 1; ++i)
  {
    out << part[i] << ' ';
  }
  out << *(part.end() - 1) << ')';

  return out;
}


class partition
{
public:
  class iterator
      : public std::iterator<std::bidirectional_iterator_tag,
                             const std::vector<unsigned>>
  {
  public:
    iterator(unsigned n, bool first = true);

    reference operator*() const { return kappa; }
    pointer operator->() const { return &kappa; }

    unsigned size() const { return kappa.size(); }
    unsigned subsets() const { return M[size() - 1] + 1; }

    iterator &operator++();
    iterator &operator--();

    template <typename Elem>
    std::auto_ptr<std::vector<std::vector<Elem>>>
    operator[](const std::vector<Elem> &v) const;

  protected:
    std::vector<unsigned> kappa, M;

    void integrityCheck();
  };

  class iterator_k
      : public iterator
  {
  public:
    iterator_k(unsigned n, unsigned psize, bool first = true);

    // optimized version
    unsigned subsets() const { return psize; }

    iterator_k &operator++();
    iterator_k &operator--();

  private:
    const unsigned psize;

    void integrityCheck();
  };
};

extern std::ostream &operator<<(std::ostream &out,
                                partition::iterator &it);

#include "impl/partition.i.hpp"
