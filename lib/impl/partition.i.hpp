#pragma once

template <typename Elem>
std::unique_ptr<std::vector<std::vector<Elem>>>
    partition::iterator::
    operator[](const std::vector<Elem> &v)
        const
{
  std::vector<std::vector<Elem>> *part =
      new std::vector<std::vector<Elem>>(subsets());

  for (unsigned i = 0; i < size(); ++i)
    (*part)[kappa[i]].push_back(v[i]);

  return std::unique_ptr<std::vector<std::vector<Elem>>>(part);
}
