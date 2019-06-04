#pragma once

namespace alpaca {

template<class T, uint16_t N>
class Array {
    T data_[N];
    uint16_t n_;
  public:
    Array() : n_(0) {}
    Array(T v, uint16_t n = N) : n_(0) {
      fill(v, n);
    }

    bool empty() const {
      return n_ == 0;
    }

    uint16_t size() const {
      return n_;
    }

    T& operator[](uint16_t idx) {
      return data_[idx];
    }

    const T& operator[](uint16_t idx) const {
      return data_[idx];
    }

    void fill(T v, uint16_t n = N) {
      uint16_t total = min(n, N);
      for (uint16_t i = 0; i < total; i++)
        data_[i] = v;
    }

    bool push_back(T v) {
      if (n_ >= N) return false;
      data_[n_++] = v;
      return true;
    }

    T pop_back() {
      return data_[--n_];
    }

    T* last() {
      return n_ == 0 ? nullptr : &data_[n_ - 1];
    }

    uint16_t maxSize() const {
      return N;
    }

    void clear()
    {
      n_ = 0;
    }
};

}
