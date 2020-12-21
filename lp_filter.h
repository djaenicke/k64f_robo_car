#ifndef LP_FILTER_H_
#define LP_FILTER_H_

template <class T>
T LpFilter(T x, T y_n_1, float alpha)
{
  T y;

  y = ((1 - alpha) * y_n_1) + (x * alpha);

  return y;
}

#endif /* LP_FILTER_H_ */
