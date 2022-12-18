template<typename T>
class Array {
 public:
 Array(size_t sz) : size(sz) {
	data = new T[size];
    }
    ~Array() {
	delete[] data;
    }
    size_t Size() const { return size; }
    T& operator[](size_t index) { return data[index]; }
    const T& operator[](size_t index) const { return data[index]; }
 private:
    size_t size;
    T* data;
};
