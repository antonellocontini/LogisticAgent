
#pragma once

#include <algorithm>
#include <iostream>
#include <iterator>
#include <memory>
#include <vector>

template <typename T>
using auto_ptr = std::unique_ptr<T>;

class partition
{
public:
  class iterator : public std::iterator<std::bidirectional_iterator_tag, const std::vector<unsigned>>
  {
  public:
    iterator(unsigned n, bool first = true);

    reference operator*() const
    {
      return kappa;
    }
    pointer operator->() const
    {
      return &kappa;
    }

    unsigned size() const
    {
      return kappa.size();
    }
    unsigned subsets() const
    {
      return M[size() - 1] + 1;
    }

    iterator &operator++();
    iterator &operator--();

    template <typename Elem>
    std::auto_ptr<std::vector<std::vector<Elem>>> operator[](const std::vector<Elem> &v) const;

  protected:
    std::vector<unsigned> kappa, M;

    void integrityCheck();
  };

  class iterator_k : public iterator
  {
  public:
    iterator_k(unsigned n, unsigned psize, bool first = true);

    // optimized version
    unsigned subsets() const
    {
      return psize;
    }

    iterator_k &operator++();
    iterator_k &operator--();

  private:
    const unsigned psize;

    void integrityCheck();
  };
};

// extern std::ostream &operator<<(std::ostream &out, partition::iterator &it);

template <typename Elem>
std::auto_ptr<std::vector<std::vector<Elem>>> partition::iterator::operator[](const std::vector<Elem> &v) const
{
  std::vector<std::vector<Elem>> *part = new std::vector<std::vector<Elem>>(subsets());

  for (unsigned i = 0; i < size(); ++i)
    (*part)[kappa[i]].push_back(v[i]);

  return std::auto_ptr<std::vector<std::vector<Elem>>>(part);
}

template <typename Elem>
std::ostream &operator<<(std::ostream &out, std::vector<Elem> &s)
{
  out << '(';

  // out << "merda"
      // << "\n";

  for (unsigned i = 0; i < s.size() - 1; ++i)
  {
    out << s[i] << ' ';
  }
  out << *(s.end() - 1) << ')';

  return out;
}

template <typename Elem>
std::ostream &operator<<(std::ostream &out, std::vector<std::vector<Elem>> &part)
{
  out << '(';

  // out << "diocane"
      // << "\n";

  for (unsigned i = 0; i < part.size() - 1; ++i)
  {
    out << part[i] << ' ';
  }
  out << *(part.end() - 1) << ')';

  return out;
}
