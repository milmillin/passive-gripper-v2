#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

namespace psg {
namespace core {
namespace serialization {

// Arithmetic types
template <typename T>
inline typename std::enable_if<std::is_arithmetic<T>::value ||
                               std::is_enum<T>::value>::type
Serialize(const T& obj, std::ofstream& f);
template <typename T>
inline typename std::enable_if<std::is_arithmetic<T>::value ||
                               std::is_enum<T>::value>::type
Deserialize(T& obj, std::ifstream& f);

// std::string
inline void Serialize(const std::string& obj, std::ofstream& f);
inline void Deserialize(std::string& obj, std::ifstream& f);

// std::vector
template <typename T1, typename T2>
inline void Serialize(const std::vector<T1, T2>& obj, std::ofstream& f);
template <typename T1, typename T2>
inline void Deserialize(std::vector<T1, T2>& obj, std::ifstream& f);

// std::map
template <typename T1, typename T2>
inline void Serialize(const std::map<T1, T2>& obj, std::ofstream& f);
template <typename T1, typename T2>
inline void Deserialize(std::map<T1, T2>& obj, std::ifstream& f);

// Eigen::Array
template <typename T, int R, int C, int P, int MR, int MC>
inline void Serialize(const Eigen::Array<T, R, C, P, MR, MC>& obj,
                      std::ofstream& f);
template <typename T, int R, int C, int P, int MR, int MC>
inline void Deserialize(Eigen::Array<T, R, C, P, MR, MC>& obj,
                        std::ifstream& f);

// Eigen::Matrix
template <typename T, int R, int C, int P, int MR, int MC>
inline void Serialize(const Eigen::Matrix<T, R, C, P, MR, MC>& obj,
                      std::ofstream& f);
template <typename T, int R, int C, int P, int MR, int MC>
inline void Deserialize(Eigen::Matrix<T, R, C, P, MR, MC>& obj,
                        std::ifstream& f);

//============== IMPLEMENTATION =================

// Arithmetic types
template <typename T>
typename std::enable_if<std::is_arithmetic<T>::value ||
                        std::is_enum<T>::value>::type
Serialize(const T& obj, std::ofstream& f) {
  f.write(reinterpret_cast<const char*>(&obj), sizeof(obj));
}

template <typename T>
typename std::enable_if<std::is_arithmetic<T>::value ||
                          std::is_enum<T>::value>::type
Deserialize(T& obj, std::ifstream& f) {
  f.read((char*)&obj, sizeof(obj));
}

// std::string
void Serialize(const std::string& obj, std::ofstream& f) {
  Serialize(obj.size(), f);
  f.write(obj.c_str(), obj.size());
}

void Deserialize(std::string& obj, std::ifstream& f) {
  size_t size = 0;
  Deserialize(size, f);
  char* buf = new char[size + 1];
  f.read(buf, size);
  buf[size] = 0;
  obj = buf;
  delete buf;
}

// std::vector
template <typename T1, typename T2>
void Serialize(const std::vector<T1, T2>& obj, std::ofstream& f) {
  Serialize(obj.size(), f);
  for (size_t i = 0; i < obj.size(); i++) {
    Serialize(obj[i], f);
  }
}

template <typename T1, typename T2>
void Deserialize(std::vector<T1, T2>& obj, std::ifstream& f) {
  size_t size;
  Deserialize(size, f);
  obj.resize(size);
  for (size_t i = 0; i < size; i++) {
    Deserialize(obj[i], f);
  }
}

// std::map
template <typename T1, typename T2>
void Serialize(const std::map<T1, T2>& obj, std::ofstream& f) {
  Serialize(obj.size());
  for (const auto& kv : obj) {
    Serialize(kv.first, f);
    Serialize(kv.second, f);
  }
}

template <typename T1, typename T2>
void Deserialize(std::map<T1, T2>& obj, std::ifstream& f) {
  size_t size;
  Deserialize(size, f);
  obj.clear();
  for (size_t i = 0; i < size; i++) {
    T1 key;
    T2 value;
    Deserialize(key, f);
    Deserialize(value, f);
    obj.insert(std::make_pair(key, value));
  }
}

// Eigen::Array

template <typename T, int R, int C, int P, int MR, int MC>
void Serialize(const Eigen::Array<T, R, C, P, MR, MC>& obj, std::ofstream& f) {
  Serialize(obj.rows(), f);
  Serialize(obj.cols(), f);
  for (long long i = 0; i < obj.size(); i++) {
    Serialize(obj(i), f);
  }
}

template <typename T, int R, int C, int P, int MR, int MC>
void Deserialize(Eigen::Array<T, R, C, P, MR, MC>& obj, std::ifstream& f) {
  long long rows;
  long long cols;
  Deserialize(rows, f);
  Deserialize(cols, f);
  obj.resize(rows, cols);
  for (long long i = 0; i < obj.size(); i++) {
    Deserialize(obj(i), f);
  }
}

// Eigen::Matrix

template <typename T, int R, int C, int P, int MR, int MC>
void Serialize(const Eigen::Matrix<T, R, C, P, MR, MC>& obj, std::ofstream& f) {
  Serialize(obj.rows(), f);
  Serialize(obj.cols(), f);
  for (long long i = 0; i < obj.size(); i++) {
    Serialize(obj(i), f);
  }
}

template <typename T, int R, int C, int P, int MR, int MC>
void Deserialize(Eigen::Matrix<T, R, C, P, MR, MC>& obj, std::ifstream& f) {
  long long rows;
  long long cols;
  Deserialize(rows, f);
  Deserialize(cols, f);
  obj.resize(rows, cols);
  for (long long i = 0; i < obj.size(); i++) {
    Deserialize(obj(i), f);
  }
}

}  // namespace serialization
}  // namespace core
}  // namespace psg

#define DECL_SERIALIZE(Type, obj)                                  \
  namespace psg {                                                  \
  namespace core {                                                 \
  namespace serialization {                                        \
  inline void Serialize(const Type& obj, std::ofstream& f);        \
  }                                                                \
  }                                                                \
  }                                                                \
  inline void psg::core::serialization::Serialize(const Type& obj, \
                                                  std::ofstream& f)
#define DECL_DESERIALIZE(Type, obj)                     \
  namespace psg {                                       \
  namespace core {                                      \
  namespace serialization {                             \
  inline void Deserialize(Type& obj, std::ifstream& f); \
  }                                                     \
  }                                                     \
  }                                                     \
  inline void psg::core::serialization::Deserialize(Type& obj, std::ifstream& f)
#define SERIALIZE(obj) psg::core::serialization::Serialize(obj, f)
#define DESERIALIZE(obj) psg::core::serialization::Deserialize(obj, f)
