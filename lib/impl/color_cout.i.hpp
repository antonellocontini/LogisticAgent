#pragma once

const string black("30m");
const string red("31m");
const string green("32m");
const string yellow("33m");
const string blue("34m");
const string magenta("35m");
const string cyan("36m");
const string white("37m");

const string env_in("\033[");
const string env_out("\033[0m");

// const string bold    ("1;"       );
// const string normal  ("0;"      );

template <typename T>
void c_print(T t)
{
  cout << t << "\n";
}

template <typename T>
void c_print(T t, const string color)
{
  cout << env_in << color << t << env_out << "\n";
}

template <typename T, typename R>
void c_print(T t, R r, const string color)
{
  cout << env_in << color << t << r << env_out << "\n";
}

template <typename T, typename R, typename P>
void c_print(T t, R r, P p, const string color)
{
  cout << env_in << color << t << r << p << env_out << "\n";
}

template <typename T, typename R, typename P, typename F>
void c_print(T t, R r, P p, F f, const string color)
{
  cout << env_in << color << t << r << p << f << env_out << "\n";
}

template <typename T, typename R, typename P, typename F, typename Q>
void c_print(T t, R r, P p, F f, Q q, const string color)
{
  cout << env_in << color << t << r << p << f << q << env_out << "\n";
}

template <typename T, typename R, typename P, typename F, typename Q, typename Z>
void c_print(T t, R r, P p, F f, Q q, Z z, const string color)
{
  cout << env_in << color << t << r << p << f << q << z << env_out << "\n";
}

template <typename T>
void c_print(T t, bool print)
{
  if (print == true)
    cout << t << "\n";
}

template <typename T>
void c_print(T t, const string color, bool print)
{
  if (print == true)
    cout << env_in << color << t << env_out << "\n";
}

template <typename T, typename R>
void c_print(T t, R r, const string color, bool print)
{
  if (print == true)
    cout << env_in << color << t << r << env_out << "\n";
}

template <typename T, typename R, typename P>
void c_print(T t, R r, P p, const string color, bool print)
{
  if (print == true)
    cout << env_in << color << t << r << p << env_out << "\n";
}

template <typename T, typename R, typename P, typename F>
void c_print(T t, R r, P p, F f, const string color, bool print)
{
  if (print == true)
    cout << env_in << color << t << r << p << f << env_out << "\n";
}

template <typename T, typename R, typename P, typename F, typename Q>
void c_print(T t, R r, P p, F f, Q q, const string color, bool print)
{
  if (print == true)
    cout << env_in << color << t << r << p << f << q << env_out << "\n";
}

template <typename T, typename R, typename P, typename F, typename Q, typename Z>
void c_print(T t, R r, P p, F f, Q q, Z z, const string color, bool print)
{
  if (print == true)
    cout << env_in << color << t << r << p << f << q << z << env_out << "\n";
}

// template <typename T>
// void a_print(T t)
// {
//   cout << "\n";
//   for (auto i = 0; i < t.size(); i++)
//   {
//     cout << t[i];
//   }
//   cout << "\n";
// }

// template <typename T>
// void a_print(T t, const string color)
// {
//   cout << "\n";
//   for (auto i = 0; i < t.size(); i++)
//   {
//     cout << env_in << color << t[i] << env_out;
//   }
//   cout << "\n";
// }
